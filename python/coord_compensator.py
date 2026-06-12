#!/usr/bin/env python3
"""
coord_compensator.py — Thin-Plate Spline coordinate error compensation.

§3: After antenna calibration, learn systematic spatial error from
(expected, measured) position pairs and apply correction at runtime.

TPS (Thin-Plate Spline) is the best choice for 2D UWB error compensation:
  • Exact at control points (zero error at calibration positions)
  • Smooth interpolation — no oscillation between points
  • Minimal parameters: ~N+3 for N control points
  • Well-suited for the typical 9-point calibration grid

Mathematical model
------------------
For each coordinate (X, Y) independently:

    f(x, y) = a₀ + a₁x + a₂y + Σᵢ wᵢ U(‖(x,y) - pᵢ‖)

where U(r) = r² ln(r)  (radial basis function, with U(0) = 0)

The correction is:
    corrected_x = measured_x + f_x(measured_x, measured_y)
    corrected_y = measured_y + f_y(measured_x, measured_y)

Usage (standalone self-test):
    python coord_compensator.py
"""

from __future__ import annotations

import sys
import json
import math
import numpy as np
from pathlib import Path
from dataclasses import dataclass
from typing import Optional


@dataclass
class CompensationResult:
    """Result of training the TPS model."""
    n_control_points: int
    rmse_before: float           # RMSE of raw position errors (metres)
    rmse_after: float            # RMSE after TPS correction (at control points → ~0)
    max_error_before: float
    max_error_after: float
    per_point_errors_before: list
    per_point_errors_after: list
    success: bool


class CoordCompensator:
    """
    Thin-Plate Spline (TPS) error compensation for 2D UWB localization.

    Learns the systematic spatial error pattern from (expected, measured)
    position pairs and applies a smooth correction at runtime.
    """

    def __init__(self):
        self.trained = False
        self.control_pts: Optional[np.ndarray] = None   # (N, 2) measured positions
        self.target_pts: Optional[np.ndarray] = None     # (N, 2) expected positions
        self.weights_x: Optional[np.ndarray] = None      # TPS weights for X correction
        self.weights_y: Optional[np.ndarray] = None      # TPS weights for Y correction
        self.enabled = False

    @staticmethod
    def _tps_basis(r: float) -> float:
        """TPS radial basis function: U(r) = r² ln(r), with U(0) = 0."""
        if r < 1e-12:
            return 0.0
        return r * r * math.log(r)

    @staticmethod
    def _tps_basis_vec(pts: np.ndarray, center: np.ndarray) -> np.ndarray:
        """Compute U(‖pts[i] - center‖) for all points."""
        diffs = pts - center
        r = np.sqrt(np.sum(diffs**2, axis=1))
        result = np.zeros_like(r)
        mask = r > 1e-12
        result[mask] = r[mask]**2 * np.log(r[mask])
        return result

    def _build_kernel_matrix(self, pts: np.ndarray) -> np.ndarray:
        """Build the N×N TPS kernel matrix K[i,j] = U(‖pᵢ - pⱼ‖)."""
        N = pts.shape[0]
        K = np.zeros((N, N))
        for i in range(N):
            for j in range(i + 1, N):
                r = np.linalg.norm(pts[i] - pts[j])
                u = self._tps_basis(r)
                K[i, j] = u
                K[j, i] = u
        return K

    def train(
        self,
        expected_positions: np.ndarray,
        measured_positions: np.ndarray,
    ) -> CompensationResult:
        """
        Train the TPS model.

        The model learns: correction(measured) → expected.
        i.e., given a measured position, output the corrected (expected) position.

        Parameters
        ----------
        expected_positions : (N, 2) — true/known positions
        measured_positions : (N, 2) — positions reported by localization

        Returns
        -------
        CompensationResult with training diagnostics.
        """
        expected = np.atleast_2d(expected_positions)
        measured = np.atleast_2d(measured_positions)
        N = expected.shape[0]

        if N < 3:
            raise ValueError(f"Need ≥3 control points (have {N})")
        if expected.shape != measured.shape:
            raise ValueError("expected and measured must have same shape")

        # Error vectors: how much to correct (expected - measured)
        errors = expected - measured

        # RMSE before correction
        err_norms = np.linalg.norm(errors, axis=1)
        rmse_before = float(np.sqrt(np.mean(err_norms**2)))
        max_err_before = float(np.max(err_norms))

        # Build TPS system
        # [K  P] [w]   [v]
        # [Pᵀ 0] [a] = [0]
        #
        # where K = kernel matrix, P = [1, x, y], v = error values

        K = self._build_kernel_matrix(measured)
        P = np.column_stack([np.ones(N), measured])  # (N, 3)

        # Assemble the full system
        L = np.zeros((N + 3, N + 3))
        L[:N, :N] = K
        L[:N, N:N+3] = P
        L[N:N+3, :N] = P.T

        # Add small regularization to K for numerical stability
        L[:N, :N] += np.eye(N) * 1e-10

        # Solve for X corrections
        rhs_x = np.zeros(N + 3)
        rhs_x[:N] = errors[:, 0]
        wx = np.linalg.solve(L, rhs_x)

        # Solve for Y corrections
        rhs_y = np.zeros(N + 3)
        rhs_y[:N] = errors[:, 1]
        wy = np.linalg.solve(L, rhs_y)

        # Store
        self.control_pts = measured.copy()
        self.target_pts = expected.copy()
        self.weights_x = wx
        self.weights_y = wy
        self.trained = True

        # Verify: compute errors at control points (should be ~0)
        corrected = np.array([self.correct(measured[i]) for i in range(N)])
        errors_after = np.linalg.norm(corrected - expected, axis=1)
        rmse_after = float(np.sqrt(np.mean(errors_after**2)))
        max_err_after = float(np.max(errors_after))

        return CompensationResult(
            n_control_points=N,
            rmse_before=rmse_before,
            rmse_after=rmse_after,
            max_error_before=max_err_before,
            max_error_after=max_err_after,
            per_point_errors_before=err_norms.tolist(),
            per_point_errors_after=errors_after.tolist(),
            success=rmse_after < 0.01,  # should be near-zero at control points
        )

    def correct(self, measured_xy: np.ndarray) -> np.ndarray:
        """
        Apply TPS correction to a measured position.

        Parameters
        ----------
        measured_xy : (2,) — measured (x, y) position

        Returns
        -------
        corrected : (2,) — corrected (x, y) position
        """
        if not self.trained or not self.enabled:
            return np.asarray(measured_xy).ravel()

        xy = np.asarray(measured_xy).ravel()
        N = self.control_pts.shape[0]

        # Compute kernel values from this point to all control points
        u = self._tps_basis_vec(self.control_pts, xy)

        # Correction for X
        wx = self.weights_x
        dx = wx[N] + wx[N+1] * xy[0] + wx[N+2] * xy[1] + np.dot(wx[:N], u)

        # Correction for Y
        wy = self.weights_y
        dy = wy[N] + wy[N+1] * xy[0] + wy[N+2] * xy[1] + np.dot(wy[:N], u)

        return np.array([xy[0] + dx, xy[1] + dy])

    def save(self, path: str = None):
        """Save trained model to JSON."""
        if not self.trained:
            raise RuntimeError("Model not trained")

        if path is None:
            from config import COORD_COMPENSATION_FILE
            path = COORD_COMPENSATION_FILE

        data = {
            "type": "tps_coord_compensation",
            "n_control_points": int(self.control_pts.shape[0]),
            "control_pts": self.control_pts.tolist(),
            "target_pts": self.target_pts.tolist(),
            "weights_x": self.weights_x.tolist(),
            "weights_y": self.weights_y.tolist(),
        }

        with open(path, "w") as f:
            json.dump(data, f, indent=2)
        print(f"[COORD] Compensation model saved → {path}")

    @classmethod
    def load(cls, path: str = None) -> "CoordCompensator":
        """Load a previously trained model."""
        if path is None:
            from config import COORD_COMPENSATION_FILE
            path = COORD_COMPENSATION_FILE

        with open(path) as f:
            data = json.load(f)

        comp = cls()
        comp.control_pts = np.array(data["control_pts"])
        comp.target_pts = np.array(data["target_pts"])
        comp.weights_x = np.array(data["weights_x"])
        comp.weights_y = np.array(data["weights_y"])
        comp.trained = True
        comp.enabled = True
        print(f"[COORD] Loaded compensation model from {path} "
              f"({comp.control_pts.shape[0]} control points)")
        return comp


# ── Self-test ────────────────────────────────────────────────────────────────

def _self_test():
    """
    Synthetic test: create known non-linear distortion, train TPS,
    verify correction accuracy.
    """
    np.random.seed(42)

    from config import CALIB_POINTS

    # True positions (calibration grid)
    expected = np.array(CALIB_POINTS)

    # Simulate systematic non-linear distortion:
    # The UWB system has a position-dependent error
    def distortion(xy):
        x, y = xy
        # Quadratic + cross term distortion (realistic for UWB)
        dx = 0.05 * x**2 + 0.03 * x * y - 0.02 * y
        dy = -0.04 * y**2 + 0.02 * x + 0.01 * x * y
        return np.array([x + dx, y + dy])

    # Measured positions = expected + systematic distortion + small noise
    noise_sigma = 0.003  # 3 mm random noise
    measured = np.array([distortion(p) + np.random.randn(2) * noise_sigma
                         for p in expected])

    print("=" * 60)
    print("Coordinate Compensator (TPS) — Synthetic Self-Test")
    print(f"  Control points: {len(expected)}")
    print(f"  Distortion: quadratic + cross-term")
    print(f"  Noise σ: {noise_sigma*1000:.0f} mm")
    print("=" * 60)

    # Train
    comp = CoordCompensator()
    comp.enabled = True
    result = comp.train(expected, measured)

    print(f"\nTraining result:")
    print(f"  RMSE before correction: {result.rmse_before*100:.2f} cm")
    print(f"  RMSE after  correction: {result.rmse_after*100:.4f} cm")
    print(f"  Max error before: {result.max_error_before*100:.2f} cm")
    print(f"  Max error after:  {result.max_error_after*100:.4f} cm")

    # Test at control points
    print("\n  At control points:")
    for i, (ex, me) in enumerate(zip(expected, measured)):
        corrected = comp.correct(me)
        err_before = np.linalg.norm(ex - me)
        err_after = np.linalg.norm(ex - corrected)
        print(f"    ({ex[0]:+.1f},{ex[1]:+.1f}): "
              f"err before={err_before*100:5.2f} cm → after={err_after*100:5.4f} cm")

    # Test interpolation at points BETWEEN control grid
    test_pts = [(0.5, 0.5), (-0.5, -0.5), (0.3, -0.7), (-0.8, 0.2)]
    print("\n  Interpolation test (between grid points):")
    interp_errors = []
    for pt in test_pts:
        pt = np.array(pt)
        meas = distortion(pt) + np.random.randn(2) * noise_sigma
        corrected = comp.correct(meas)
        err_before = np.linalg.norm(pt - meas)
        err_after = np.linalg.norm(pt - corrected)
        interp_errors.append(err_after)
        print(f"    ({pt[0]:+.1f},{pt[1]:+.1f}): "
              f"err before={err_before*100:5.2f} cm → after={err_after*100:5.2f} cm")

    # Test save/load
    test_path = Path(__file__).parent / "_test_compensation.json"
    comp.save(str(test_path))
    comp2 = CoordCompensator.load(str(test_path))
    comp2.enabled = True
    reload_ok = True
    for i, me in enumerate(measured):
        c1 = comp.correct(me)
        c2 = comp2.correct(me)
        if np.linalg.norm(c1 - c2) > 1e-10:
            reload_ok = False
            break
    if test_path.exists():
        test_path.unlink()
    print(f"\n  Save/Load round-trip: {'PASS' if reload_ok else 'FAIL'}")

    # Overall verdict
    ok = (result.rmse_after < 0.001 and          # ~0 at control points
          max(interp_errors) < 0.05 and           # <5 cm interpolation
          reload_ok)
    print(f"\n{'PASS' if ok else 'FAIL'}: "
          f"control RMSE={result.rmse_after*100:.4f} cm, "
          f"max interp err={max(interp_errors)*100:.2f} cm")
    return ok


if __name__ == "__main__":
    ok = _self_test()
    sys.exit(0 if ok else 1)
