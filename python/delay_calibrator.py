#!/usr/bin/env python3
"""
delay_calibrator.py — Automatic antenna delay calibration for UWB anchors.

§2: Given known anchor and tag positions at calibration points, estimates
per-anchor antenna delay corrections by comparing measured UWB distances
against expected geometric distances.

The tag delay is fixed (compensated as a known offset); only anchor delays
are calibrated.

Mathematical model
------------------
DS-TWR measured distance includes a bias from antenna delays:

    d_measured ≈ d_true + (δ_anchor + δ_tag) × DELAY_TO_METRES

For anchor-only calibration (tag delay fixed):

    bias_i = d_measured_i - d_expected_i = δ_i × DELAY_TO_METRES + ε

This is LINEAR in δ — direct least-squares solution.

Usage (standalone self-test):
    python delay_calibrator.py
"""

from __future__ import annotations

import sys
import json
import math
import numpy as np
from pathlib import Path
from dataclasses import dataclass, field
from typing import Optional

try:
    from scipy.optimize import least_squares
except ImportError:
    print("Missing dependency: pip install scipy")
    sys.exit(1)


@dataclass
class DelayCalibResult:
    """Result of the antenna delay calibration."""
    anchor_corrections: dict         # {"A1": +45, "A2": -12, ...} delay units
    new_delays: dict                 # {"A1": 16601, ...} absolute delay values
    initial_delays: dict             # {"A1": 16556, ...} starting delays
    tag_correction: float            # tag correction in delay units (0 for anchor_only)
    rmse_before: float               # distance RMSE before correction (metres)
    rmse_after: float                # distance RMSE after correction (metres)
    per_point_errors_before: list    # per-observation (metres)
    per_point_errors_after: list     # per-observation (metres)
    n_observations: int
    n_points: int
    success: bool
    method: str


# DW1000 constant: 1 delay unit ≈ 0.4691 mm
DELAY_TO_METRES: float = 0.4691e-3


class DelayCalibrator:
    """
    Estimates per-anchor antenna delay corrections from calibration captures.

    Mode 'anchor_only': Fix tag delay, solve 3 anchor delays (3 unknowns).
    Mode 'joint': Solve all 4 with zero-mean constraint on tag + anchors.
    """

    def __init__(
        self,
        anchor_coords: dict,
        anchor_ids: list[str] = None,
        mode: str = "anchor_only",
        delay_bound: int = 500,
    ):
        """
        Parameters
        ----------
        anchor_coords : dict
            {"A1": (x, y), "A2": (x, y), "A3": (x, y)}
        anchor_ids : list
            Ordered anchor names, default ["A1", "A2", "A3"]
        mode : str
            "anchor_only" or "joint"
        delay_bound : int
            Maximum correction in delay units (±bound)
        """
        self.anchor_coords = anchor_coords
        self.anchor_ids = anchor_ids or ["A1", "A2", "A3"]
        self.mode = mode
        self.delay_bound = delay_bound
        self.captures: list[dict] = []  # [{point: (x,y), dists: {A1: d, ...}}]

    def add_capture(self, point_xy: tuple, mean_distances: dict):
        """
        Add one calibration point's averaged distances.

        Parameters
        ----------
        point_xy : (float, float)
            Known tag position
        mean_distances : dict
            {"A1": 2.34, "A2": 1.56, ...} in metres (mean of N samples)
        """
        self.captures.append({
            "point": point_xy,
            "dists": dict(mean_distances),
        })

    def clear(self):
        self.captures.clear()

    def solve(self, initial_delays: dict = None) -> DelayCalibResult:
        """
        Run the delay calibration optimizer.

        Parameters
        ----------
        initial_delays : dict, optional
            {"A1": 16556, ...} — current delay values. Used for absolute output.

        Returns
        -------
        DelayCalibResult
        """
        if len(self.captures) < 2:
            raise ValueError(f"Need ≥2 calibration points (have {len(self.captures)})")

        if initial_delays is None:
            from config import DEFAULT_ADELAY
            initial_delays = {aid: DEFAULT_ADELAY for aid in self.anchor_ids}

        N_anchors = len(self.anchor_ids)
        K = len(self.captures)

        # Build observation arrays
        # Each observation: (point_idx, anchor_idx, d_measured, d_expected)
        observations = []
        for k, cap in enumerate(self.captures):
            tag_pos = np.array(cap["point"])
            for j, aid in enumerate(self.anchor_ids):
                if aid in cap["dists"]:
                    d_meas = cap["dists"][aid]
                    anchor_pos = np.array(self.anchor_coords[aid])
                    d_expected = np.linalg.norm(tag_pos - anchor_pos)
                    observations.append((k, j, d_meas, d_expected))

        n_obs = len(observations)
        if n_obs < N_anchors:
            raise ValueError(f"Only {n_obs} observations — need ≥{N_anchors}")

        d_meas_arr = np.array([o[2] for o in observations])
        d_exp_arr = np.array([o[3] for o in observations])
        anchor_idx_arr = np.array([o[1] for o in observations])

        # Bias = measured - expected (before any correction)
        biases = d_meas_arr - d_exp_arr
        rmse_before = float(np.sqrt(np.mean(biases**2)))
        errors_before = np.abs(biases).tolist()

        if self.mode == "anchor_only":
            # ── Linear LS: solve for anchor delay corrections ────────
            # bias_i = δ_j × DELAY_TO_METRES  →  δ_j = bias_i / DELAY_TO_METRES
            # Solve directly for delay units: A @ δ = biases / DELAY_TO_METRES
            # Build design matrix A (n_obs × N_anchors): indicator for which anchor
            A = np.zeros((n_obs, N_anchors))
            for i, (_, j, _, _) in enumerate(observations):
                A[i, j] = 1.0

            # Convert biases to delay units
            biases_units = biases / DELAY_TO_METRES

            # Direct linear solve (unconstrained — then clamp)
            corrections_units, _, _, _ = np.linalg.lstsq(A, biases_units, rcond=None)

            # Clamp to bounds
            corrections_units = np.clip(corrections_units,
                                        -self.delay_bound, self.delay_bound)
            corrections_m = corrections_units * DELAY_TO_METRES
            tag_correction = 0.0

        else:
            # ── Joint mode: N_anchors + 1 unknowns with zero-mean ────
            # bias_i = (δ_j + δ_tag) × DELAY_TO_METRES
            n_vars = N_anchors + 1  # last var is tag delay
            A = np.zeros((n_obs + 1, n_vars))
            b_units = np.zeros(n_obs + 1)

            for i, (_, j, _, _) in enumerate(observations):
                A[i, j] = 1.0
                A[i, N_anchors] = 1.0
                b_units[i] = biases[i] / DELAY_TO_METRES

            # Zero-mean constraint
            A[n_obs, :] = 1.0
            b_units[n_obs] = 0.0

            all_units, _, _, _ = np.linalg.lstsq(A, b_units, rcond=None)
            all_units = np.clip(all_units, -self.delay_bound, self.delay_bound)

            corrections_units = all_units[:N_anchors]
            corrections_m = corrections_units * DELAY_TO_METRES
            tag_correction = float(all_units[N_anchors])

        # Build result
        anchor_corrections = {}
        new_delays = {}
        for j, aid in enumerate(self.anchor_ids):
            delta = int(round(corrections_units[j]))
            anchor_corrections[aid] = delta
            raw = initial_delays.get(aid, 16556) + delta
            new_delays[aid] = max(1, min(65534, raw))

        # Compute RMSE after correction
        corrected_biases = biases.copy()
        for i, (_, j, _, _) in enumerate(observations):
            corrected_biases[i] -= corrections_m[j]
            if self.mode == "joint":
                corrected_biases[i] -= result.x[N_anchors]

        rmse_after = float(np.sqrt(np.mean(corrected_biases**2)))
        errors_after = np.abs(corrected_biases).tolist()

        return DelayCalibResult(
            anchor_corrections=anchor_corrections,
            new_delays=new_delays,
            initial_delays=dict(initial_delays),
            tag_correction=tag_correction,
            rmse_before=rmse_before,
            rmse_after=rmse_after,
            per_point_errors_before=errors_before,
            per_point_errors_after=errors_after,
            n_observations=n_obs,
            n_points=K,
            success=True,
            method=self.mode,
        )


# ── Self-test ────────────────────────────────────────────────────────────────

def _self_test():
    """
    Synthetic test: inject known delay biases per anchor, verify recovery.
    """
    np.random.seed(42)

    from config import ANCHOR_COORDS, CALIB_POINTS, DEFAULT_ADELAY

    # Inject known biases (in metres)
    true_biases_m = {"A1": 0.05, "A2": -0.03, "A3": 0.02}
    true_biases_units = {k: v / DELAY_TO_METRES for k, v in true_biases_m.items()}

    print("=" * 60)
    print("Delay Calibrator — Synthetic Self-Test")
    print("=" * 60)
    print(f"\nInjected biases:")
    for aid, bias in true_biases_m.items():
        print(f"  {aid}: {bias*1000:+.1f} mm ({true_biases_units[aid]:+.1f} delay units)")

    noise_sigma = 0.005  # 5 mm measurement noise

    cal = DelayCalibrator(ANCHOR_COORDS, mode="anchor_only")

    for (x_t, y_t) in CALIB_POINTS:
        tag_pos = np.array([x_t, y_t])
        dists = {}
        for aid in ["A1", "A2", "A3"]:
            anchor_pos = np.array(ANCHOR_COORDS[aid])
            d_true = np.linalg.norm(tag_pos - anchor_pos)
            d_meas = d_true + true_biases_m[aid] + np.random.randn() * noise_sigma
            dists[aid] = max(0.01, d_meas)
        cal.add_capture((x_t, y_t), dists)

    initial = {aid: DEFAULT_ADELAY for aid in ["A1", "A2", "A3"]}
    result = cal.solve(initial)

    print(f"\nResults:")
    print(f"  RMSE before: {result.rmse_before*100:.2f} cm")
    print(f"  RMSE after:  {result.rmse_after*100:.2f} cm")
    print()

    print("┌────────┬────────────────┬────────────────┬───────────┬───────────────┐")
    print("│ Anchor │ True bias      │ Estimated      │ Old delay │ New delay     │")
    print("├────────┼────────────────┼────────────────┼───────────┼───────────────┤")
    max_err = 0
    for aid in ["A1", "A2", "A3"]:
        true_u = true_biases_units[aid]
        est_u = result.anchor_corrections[aid]
        err = abs(true_u - est_u)
        max_err = max(max_err, err)
        print(f"│ {aid}     │ {true_u:+8.1f} units │ {est_u:+8d} units │ {initial[aid]:>9d} │ {result.new_delays[aid]:>9d} (Δ{est_u:+d}) │")
    print("└────────┴────────────────┴────────────────┴───────────┴───────────────┘")

    ok = max_err < 10 and result.rmse_after < 0.01
    print(f"\n{'PASS' if ok else 'FAIL'}: max correction error={max_err:.1f} units, "
          f"RMSE after={result.rmse_after*100:.2f} cm")
    return ok


if __name__ == "__main__":
    ok = _self_test()
    sys.exit(0 if ok else 1)
