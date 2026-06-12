"""
calibration_optimizer.py — Pure scipy optimizer for UWB antenna delay calibration.

No network or GUI code. Import and call optimize_delays() directly.

Mathematics
-----------
DW1000 measured range has a fixed positive bias that scales with antenna delay:
    d_meas ≈ d_true + bias
    bias ≈ delay_units × 0.4691e-3 m  (DW1000 at 63.8976 GHz clock)

We optimise over correction deltas (δA, δB, δC) in delay-units:
    corrected_dᵢ = mean_dᵢ_anchor - δ_anchor × DELAY_TO_METRES

Then re-run the closed-form linear trilateration and minimise:
    J = Σ_points [ (x_est - x_true)² + (y_est - y_true)² ]

Optionally, anchor coordinate corrections (±10 cm) can be co-optimised.
"""

from __future__ import annotations

import numpy as np
from dataclasses import dataclass, field
from typing import Optional
from scipy.optimize import least_squares

# DW1000 clock-derived constant: 1 delay unit ≈ 0.4691 mm
DELAY_TO_METRES: float = 0.4691e-3

# Search bounds
DELAY_BOUND: int   = 500     # ±500 delay units
COORD_BOUND: float = 0.10    # ±0.10 m anchor position correction


@dataclass
class CalibPoint:
    """One calibration ground-truth point with accumulated range statistics."""
    x_true: float
    y_true: float
    n_samples: int
    mean_d0: float   # mean range to A1
    mean_d1: float   # mean range to A2
    mean_d2: float   # mean range to A3
    std_d0: float = 0.0
    std_d1: float = 0.0
    std_d2: float = 0.0
    mean_x: float = 0.0
    mean_y: float = 0.0
    std_x: float = 0.0
    std_y: float = 0.0


@dataclass
class OptimizationResult:
    new_delays: dict                         # {"A1": int, "A2": int, "A3": int}
    anchor_corrections: Optional[dict]       # {"A1": (dx, dy), ...} or None
    rmse_before: float
    rmse_after: float
    max_error_before: float
    max_error_after: float
    per_point_errors_before: list[float]
    per_point_errors_after: list[float]
    optimizer_message: str
    success: bool
    nfev: int = 0


def _build_trilat_inv(anchors: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """
    Pre-compute A^-1 and k-vector for closed-form 2D trilateration.
    anchors: shape (3, 2)  — [x, y] for each anchor
    """
    A = np.array([
        [anchors[1, 0] - anchors[0, 0], anchors[1, 1] - anchors[0, 1]],
        [anchors[2, 0] - anchors[0, 0], anchors[2, 1] - anchors[0, 1]],
    ], dtype=float)
    k = np.sum(anchors ** 2, axis=1)   # shape (3,)
    Ainv = np.linalg.inv(A)
    return Ainv, k


def _trilaterate(d: np.ndarray, anchors: np.ndarray,
                 Ainv: np.ndarray, k: np.ndarray) -> np.ndarray:
    """
    Closed-form linear trilateration.
    d: shape (3,)  — measured ranges
    Returns estimated [x, y].
    """
    b = np.array([
        d[0]**2 - d[1]**2 + k[1] - k[0],
        d[0]**2 - d[2]**2 + k[2] - k[0],
    ], dtype=float)
    return 0.5 * (Ainv @ b)


def _position_errors(calib_points: list[CalibPoint],
                     anchors: np.ndarray,
                     delays_delta: np.ndarray) -> tuple[list[float], list[float]]:
    """
    Compute per-point Euclidean position errors given delay correction deltas.
    delays_delta: shape (3,) — correction in delay units for [A1, A2, A3]
    Returns (x_residuals, y_residuals) for scipy.optimize.least_squares.
    """
    Ainv, k = _build_trilat_inv(anchors)
    corrections_m = delays_delta * DELAY_TO_METRES   # shape (3,) metres

    x_res, y_res = [], []
    for pt in calib_points:
        d_raw = np.array([pt.mean_d0, pt.mean_d1, pt.mean_d2])
        d_corr = d_raw - corrections_m
        d_corr = np.clip(d_corr, 0.01, None)   # physical floor
        pos = _trilaterate(d_corr, anchors, Ainv, k)
        x_res.append(pos[0] - pt.x_true)
        y_res.append(pos[1] - pt.y_true)
    return x_res, y_res


def _residuals_delays_only(params: np.ndarray,
                            calib_points: list[CalibPoint],
                            anchor_coords: np.ndarray) -> np.ndarray:
    """Residuals for delay-only optimisation (3 free variables)."""
    xr, yr = _position_errors(calib_points, anchor_coords, params[:3])
    return np.array(xr + yr)


def _residuals_with_positions(params: np.ndarray,
                               calib_points: list[CalibPoint],
                               base_anchors: np.ndarray) -> np.ndarray:
    """Residuals for joint delay + anchor-position optimisation (9 free variables)."""
    delta_delays = params[:3]
    delta_coords  = params[3:].reshape(3, 2)
    anchors = base_anchors + delta_coords
    xr, yr = _position_errors(calib_points, anchors, delta_delays)
    return np.array(xr + yr)


def optimize_delays(
    calib_points: list[CalibPoint],
    anchor_coords: np.ndarray,        # shape (3, 2)
    initial_delays: dict[str, int],   # {"A1": int, "A2": int, "A3": int}
    optimize_positions: bool = False,
) -> OptimizationResult:
    """
    Run Levenberg-Marquardt optimisation to find the best antenna delay
    corrections (and optionally anchor coordinate corrections) that minimise
    the sum of squared position errors over all calibration points.

    Parameters
    ----------
    calib_points      : list of CalibPoint (at least 3, ideally 9)
    anchor_coords     : numpy (3, 2) — [x, y] for A1, A2, A3 (metres)
    initial_delays    : dict with keys "A1", "A2", "A3" → current delay values
    optimize_positions: also co-optimise anchor coordinates (±10 cm bounds)

    Returns
    -------
    OptimizationResult with new delays and per-point diagnostics.
    """
    if len(calib_points) < 3:
        raise ValueError("Need at least 3 calibration points.")

    delay_arr = np.array([initial_delays["A1"],
                          initial_delays["A2"],
                          initial_delays["A3"]], dtype=float)

    # ── Compute RMSE before ──────────────────────────────────────
    xr0, yr0 = _position_errors(calib_points, anchor_coords,
                                 np.zeros(3, dtype=float))
    errors_before = [np.sqrt(x**2 + y**2) for x, y in zip(xr0, yr0)]
    rmse_before   = float(np.sqrt(np.mean(np.array(errors_before)**2)))
    max_err_before = float(np.max(errors_before))

    # ── Run optimisation ─────────────────────────────────────────
    if not optimize_positions:
        x0     = np.zeros(3, dtype=float)
        bounds = (np.full(3, -DELAY_BOUND), np.full(3,  DELAY_BOUND))
        result = least_squares(
            _residuals_delays_only,
            x0, bounds=bounds, method="trf",
            args=(calib_points, anchor_coords),
            ftol=1e-9, xtol=1e-9, gtol=1e-9, max_nfev=2000,
        )
        delta_delays    = result.x[:3]
        anchor_corr_out = None

    else:
        x0     = np.zeros(9, dtype=float)
        lb = np.array([-DELAY_BOUND]*3 + [-COORD_BOUND]*6)
        ub = np.array([ DELAY_BOUND]*3 + [ COORD_BOUND]*6)
        result = least_squares(
            _residuals_with_positions,
            x0, bounds=(lb, ub), method="trf",
            args=(calib_points, anchor_coords),
            ftol=1e-9, xtol=1e-9, gtol=1e-9, max_nfev=4000,
        )
        delta_delays   = result.x[:3]
        delta_coords   = result.x[3:].reshape(3, 2)
        anchor_corr_out = {
            "A1": tuple(delta_coords[0]),
            "A2": tuple(delta_coords[1]),
            "A3": tuple(delta_coords[2]),
        }

    # ── Compute RMSE after ───────────────────────────────────────
    anchors_final = anchor_coords.copy()
    if optimize_positions and anchor_corr_out:
        for i, key in enumerate(["A1", "A2", "A3"]):
            anchors_final[i] += np.array(anchor_corr_out[key])

    xr1, yr1 = _position_errors(calib_points, anchors_final, delta_delays)
    errors_after = [np.sqrt(x**2 + y**2) for x, y in zip(xr1, yr1)]
    rmse_after   = float(np.sqrt(np.mean(np.array(errors_after)**2)))
    max_err_after = float(np.max(errors_after))

    # ── Build new delay dict ─────────────────────────────────────
    new_delays = {}
    for i, key in enumerate(["A1", "A2", "A3"]):
        raw = initial_delays[key] + int(round(delta_delays[i]))
        new_delays[key] = max(1, min(65534, raw))   # clamp to valid range

    return OptimizationResult(
        new_delays             = new_delays,
        anchor_corrections     = anchor_corr_out,
        rmse_before            = rmse_before,
        rmse_after             = rmse_after,
        max_error_before       = max_err_before,
        max_error_after        = max_err_after,
        per_point_errors_before= errors_before,
        per_point_errors_after = errors_after,
        optimizer_message      = result.message,
        success                = bool(result.success),
        nfev                   = int(result.nfev),
    )


# ── CLI smoke-test ────────────────────────────────────────────
if __name__ == "__main__":
    """
    Quick self-test: inject known range biases, confirm optimizer recovers them.
    Inject +0.05 m bias on A1 only → A1 delay correction should be ≈ +107 units.
    """
    import json

    SQRT3 = 1.73205
    anchors = np.array([
        [ 0.0000,  2.0000],
        [-SQRT3,  -1.0000],
        [ SQRT3,  -1.0000],
    ])

    from config import CALIB_POINTS

    BIAS_A1_M = 0.05   # inject 50 mm bias on A1

    def true_ranges(x_t, y_t):
        pt = np.array([x_t, y_t])
        return np.linalg.norm(anchors - pt, axis=1)

    pts = []
    for (x_t, y_t) in CALIB_POINTS:
        d = true_ranges(x_t, y_t)
        d[0] += BIAS_A1_M   # artificial bias
        pts.append(CalibPoint(
            x_true=x_t, y_true=y_t, n_samples=200,
            mean_d0=float(d[0]), mean_d1=float(d[1]), mean_d2=float(d[2]),
        ))

    initial = {"A1": 16556, "A2": 16556, "A3": 16556}
    res = optimize_delays(pts, anchors, initial, optimize_positions=False)
    expected_delta = round(BIAS_A1_M / DELAY_TO_METRES)

    print("=== Optimizer self-test ===")
    print(f"Injected bias: {BIAS_A1_M*1000:.1f} mm on A1 "
          f"(≈ {expected_delta} delay units)")
    print(f"New delays: {res.new_delays}")
    print(f"Expected A1 delta: +{expected_delta}  Got: {res.new_delays['A1'] - 16556:+d}")
    print(f"RMSE before: {res.rmse_before*100:.2f} cm   after: {res.rmse_after*100:.2f} cm")
    print(f"Max error before: {res.max_error_before*100:.2f} cm   after: {res.max_error_after*100:.2f} cm")
    print(f"Success: {res.success}  nfev: {res.nfev}  msg: {res.optimizer_message}")
