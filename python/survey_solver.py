#!/usr/bin/env python3
"""
survey_solver.py — Tag-mediated anchor survey with Procrustes alignment.

§1 Fix: Instead of outputting surveyed coordinates in an arbitrary local
frame (A1=origin, A2=+X), we compute the rigid transform (rotation +
translation + optional scale) that maps the surveyed coordinates into
the known reference frame.

Usage (standalone self-test):
    python survey_solver.py

Called from calibration_server.py:
    from survey_solver import SurveySolver
"""

import sys
import json
import math
import numpy as np
from pathlib import Path
from datetime import datetime
from dataclasses import dataclass, field

import warnings
with warnings.catch_warnings():
    warnings.simplefilter("ignore", UserWarning)
    try:
        from scipy.optimize import least_squares
    except ImportError:
        print("Missing dependency: pip install scipy")
        sys.exit(1)


@dataclass
class SurveyResult:
    """Result of the survey + Procrustes alignment."""
    surveyed_coords: dict            # {"A1": (x, y), ...} — raw optimizer output
    transformed_coords: dict         # {"A1": (x, y), ...} — after Procrustes
    reference_coords: dict           # {"A1": (x, y), ...} — known reference
    per_anchor_residual: dict        # {"A1": 0.015, ...} — metres
    rms_error: float                 # overall RMS error (metres)
    rotation_deg: float              # rotation angle applied
    scale_factor: float              # scale correction applied
    translation: tuple               # (tx, ty) translation applied
    optimizer_info: dict             # raw optimizer stats
    success: bool


class SurveySolver:
    """
    Jointly solves for anchor positions from tag-mediated distance captures,
    then aligns to the known reference frame via Procrustes.
    """

    def __init__(self, reference_coords: dict, anchor_ids: list = None):
        """
        Parameters
        ----------
        reference_coords : dict
            Known anchor layout, e.g. {"A1": (0.0, 2.0), "A2": (-1.7321, -1.0), ...}
        anchor_ids : list, optional
            Ordered anchor IDs as integers (DW1000 addresses). Default: [1, 2, 3]
        """
        self.reference_coords = reference_coords
        self.anchor_ids = anchor_ids or [1, 2, 3]
        self.anchor_names = [f"A{aid}" for aid in self.anchor_ids]
        self.captures = []

    def add_capture(self, dists: dict):
        """
        Add one calibration capture.

        Parameters
        ----------
        dists : dict
            {anchor_id_int: averaged_distance_m, ...}
        """
        self.captures.append({"dists": dists})

    def clear(self):
        self.captures.clear()

    def solve(self, apply_scale: bool = True) -> SurveyResult:
        """
        Run the joint optimizer + Procrustes alignment.

        Parameters
        ----------
        apply_scale : bool
            If True, the Procrustes transform includes a uniform scale correction.
            If False, only rotation + translation are applied.

        Returns
        -------
        SurveyResult
        """
        if len(self.captures) < 4:
            raise ValueError(f"Need ≥4 captures, have {len(self.captures)}")

        # ── Step 1: Joint optimizer (anchor + tag positions) ──────────
        raw_positions, opt_info = self._joint_optimize()

        # Build surveyed dict
        surveyed = {}
        for i, name in enumerate(self.anchor_names):
            surveyed[name] = (float(raw_positions[i, 0]), float(raw_positions[i, 1]))

        # ── Step 2: Procrustes alignment to reference frame ──────────
        S = raw_positions.copy()  # (N, 2) surveyed
        R = np.array([self.reference_coords[name] for name in self.anchor_names])  # (N, 2)

        transformed, rot_deg, scale, translation = self._procrustes(S, R, apply_scale)

        # Build result dicts
        transformed_dict = {}
        residuals_dict = {}
        for i, name in enumerate(self.anchor_names):
            transformed_dict[name] = (float(transformed[i, 0]), float(transformed[i, 1]))
            residuals_dict[name] = float(np.linalg.norm(transformed[i] - R[i]))

        rms = float(np.sqrt(np.mean(np.array(list(residuals_dict.values()))**2)))

        return SurveyResult(
            surveyed_coords=surveyed,
            transformed_coords=transformed_dict,
            reference_coords={k: v for k, v in self.reference_coords.items()},
            per_anchor_residual=residuals_dict,
            rms_error=rms,
            rotation_deg=rot_deg,
            scale_factor=scale,
            translation=translation,
            optimizer_info=opt_info,
            success=opt_info.get("success", False),
        )

    # ── Joint optimizer (same core as multipoint_survey.py) ──────────

    def _joint_optimize(self) -> tuple:
        """
        Jointly solve for N anchor positions given K captures.
        Returns (positions (N,2), info dict).
        """
        N = len(self.anchor_ids)
        K = len(self.captures)
        aid_to_idx = {aid: i for i, aid in enumerate(self.anchor_ids)}

        # Build observation list
        obs = []
        for ci, cap in enumerate(self.captures):
            for aid, dist in cap["dists"].items():
                aid_int = int(aid)
                if aid_int in aid_to_idx and dist > 0:
                    obs.append((ci, aid_to_idx[aid_int], float(dist)))

        if len(obs) < N + K:
            raise ValueError(
                f"Not enough observations ({len(obs)}) for {N} anchors + {K} captures."
            )

        # Initial guess — equilateral polygon scaled by median distance
        median_dist = np.median([d for _, _, d in obs])
        x0_anchors = np.zeros((N, 2))
        for i in range(N):
            angle = 2 * math.pi * i / N + math.pi / 2
            x0_anchors[i, 0] = median_dist * math.cos(angle)
            x0_anchors[i, 1] = median_dist * math.sin(angle)

        x0_tags = np.tile(x0_anchors.mean(axis=0), (K, 1))

        # Pack/unpack with DOF removal (anchor0=origin, anchor1=+X)
        def pack(a, t):
            parts = []
            if N >= 2:
                parts.append(a[1, 0:1])
            for i in range(2, N):
                parts.append(a[i, :2])
            for ci in range(K):
                parts.append(t[ci, :2])
            return np.concatenate(parts) if parts else np.array([])

        def unpack(p):
            a = np.zeros((N, 2))
            idx = 0
            if N >= 2:
                a[1, 0] = p[idx]; idx += 1
            for i in range(2, N):
                a[i, :2] = p[idx:idx+2]; idx += 2
            t = np.zeros((K, 2))
            for ci in range(K):
                t[ci, :2] = p[idx:idx+2]; idx += 2
            return a, t

        p0 = pack(x0_anchors, x0_tags)

        def residuals(p):
            a, t = unpack(p)
            r = np.empty(len(obs))
            for k, (ci, ai, d_meas) in enumerate(obs):
                d_model = np.linalg.norm(t[ci] - a[ai])
                r[k] = d_model - d_meas
            return r

        result = least_squares(
            residuals, p0, method="trf",
            loss="soft_l1", f_scale=0.10,
            max_nfev=5000,
        )

        a_final, _ = unpack(result.x)

        # Enforce canonical orientation for consistency
        if N >= 2 and a_final[1, 0] < 0:
            a_final[:, 0] = -a_final[:, 0]
        if N >= 3 and a_final[2, 1] < 0:
            a_final[:, 1] = -a_final[:, 1]

        r_final = residuals(result.x)
        rms = float(np.sqrt(np.mean(r_final**2)))

        info = {
            "residual_rms": rms,
            "n_captures": K,
            "n_anchors": N,
            "cost": float(result.cost),
            "success": bool(result.success),
            "message": str(result.message),
        }

        return a_final[:, :2], info

    # ── Procrustes alignment ─────────────────────────────────────────

    @staticmethod
    def _procrustes(S: np.ndarray, R: np.ndarray, apply_scale: bool = True):
        """
        Compute the Procrustes alignment: find rotation, scale, translation
        that maps surveyed points S onto reference points R.

        Parameters
        ----------
        S : (N, 2) surveyed positions
        R : (N, 2) reference positions
        apply_scale : bool — include uniform scale correction

        Returns
        -------
        transformed : (N, 2) — S mapped to R's frame
        rotation_deg : float — rotation angle in degrees
        scale : float — scale factor (1.0 if not applied)
        translation : (float, float) — (tx, ty)
        """
        N = S.shape[0]

        # Centroids
        mu_S = S.mean(axis=0)
        mu_R = R.mean(axis=0)

        # Centre both point sets
        S_c = S - mu_S
        R_c = R - mu_R

        # Cross-covariance matrix
        H = S_c.T @ R_c  # (2, 2)

        # SVD
        U, Sigma, Vt = np.linalg.svd(H)
        V = Vt.T

        # Correct for reflection
        d = np.linalg.det(V @ U.T)
        D = np.diag([1.0, np.sign(d)])

        # Optimal rotation
        Rot = V @ D @ U.T

        # Optimal scale
        if apply_scale:
            var_S = np.sum(S_c ** 2)
            if var_S > 1e-12:
                scale = np.trace(np.diag(Sigma) @ D) / var_S
            else:
                scale = 1.0
        else:
            scale = 1.0

        # Translation
        t = mu_R - scale * (Rot @ mu_S)

        # Apply transform
        transformed = (scale * (Rot @ S.T)).T + t

        # Extract rotation angle
        rot_rad = math.atan2(Rot[1, 0], Rot[0, 0])
        rot_deg = math.degrees(rot_rad)

        return transformed, rot_deg, float(scale), (float(t[0]), float(t[1]))


def write_anchors_json(positions: dict, path: str = None) -> dict:
    """Write computed anchor positions to anchors.json."""
    from config import ANCHORS_JSON_FILE, ANCHOR_ID_TO_ADDR

    if path is None:
        path = ANCHORS_JSON_FILE
    path = Path(path)

    xs = [positions[k][0] for k in sorted(positions)]
    ys = [positions[k][1] for k in sorted(positions)]

    pad = 0.5
    bounds = [
        min(xs) - pad, max(xs) + pad,
        min(ys) - pad, max(ys) + pad,
        0.0, 3.0,
    ]

    anchors_list = []
    for name in sorted(positions):
        aid = ANCHOR_ID_TO_ADDR.get(name, int(name[1:]))
        anchors_list.append({
            "id": aid,
            "x": round(positions[name][0], 4),
            "y": round(positions[name][1], 4),
            "z": 0.0,
        })

    out = {
        "dim": 2,
        "bounds": [round(v, 4) for v in bounds],
        "anchors": anchors_list,
        "survey_timestamp": datetime.now().isoformat(timespec="seconds"),
        "survey_method": "multipoint_tag_mediated_procrustes",
    }

    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w") as f:
        json.dump(out, f, indent=2)
        f.write("\n")

    print(f"[SURVEY] Written anchors.json → {path}")
    return out


# ── Self-test ────────────────────────────────────────────────────────────────

def _self_test():
    """
    Synthetic test: place anchors, simulate captures, verify Procrustes recovery.
    """
    np.random.seed(42)

    from config import ANCHOR_COORDS, CALIB_POINTS

    ref = ANCHOR_COORDS
    true_anchors = np.array([ref["A1"], ref["A2"], ref["A3"]])

    # Simulate the tag at each calibration grid point
    noise_sigma = 0.05  # 5 cm noise on distances
    solver = SurveySolver(ref, anchor_ids=[1, 2, 3])

    for (x_t, y_t) in CALIB_POINTS:
        tag = np.array([x_t, y_t])
        dists = {}
        for ai, aid in enumerate([1, 2, 3]):
            d_true = np.linalg.norm(tag - true_anchors[ai])
            dists[aid] = max(0.01, d_true + np.random.randn() * noise_sigma)
        solver.add_capture(dists)

    result = solver.solve(apply_scale=True)

    print("=" * 60)
    print("Survey Solver — Synthetic Self-Test")
    print(f"  Noise σ: {noise_sigma*100:.0f} cm")
    print("=" * 60)

    print(f"\nOptimizer: success={result.optimizer_info['success']}  "
          f"residual_rms={result.optimizer_info['residual_rms']:.4f} m")

    print(f"\nProcrustes: rotation={result.rotation_deg:.2f}°  "
          f"scale={result.scale_factor:.4f}  "
          f"translation=({result.translation[0]:.3f}, {result.translation[1]:.3f})")

    print("\n┌─────────┬────────────────────────┬────────────────────────┬────────────────────────┬──────────┐")
    print("│ Anchor  │ Surveyed (raw)         │ Transformed            │ Reference              │ Error    │")
    print("├─────────┼────────────────────────┼────────────────────────┼────────────────────────┼──────────┤")
    for name in ["A1", "A2", "A3"]:
        sx, sy = result.surveyed_coords[name]
        tx, ty = result.transformed_coords[name]
        rx, ry = result.reference_coords[name]
        err = result.per_anchor_residual[name]
        print(f"│ {name}      │ ({sx:+7.3f}, {sy:+7.3f})    │ ({tx:+7.3f}, {ty:+7.3f})    │ ({rx:+7.3f}, {ry:+7.3f})    │ {err*100:5.1f} cm │")
    print("└─────────┴────────────────────────┴────────────────────────┴────────────────────────┴──────────┘")

    print(f"\nRMS error: {result.rms_error*100:.2f} cm")

    ok = result.rms_error < 0.15
    print(f"\n{'PASS' if ok else 'FAIL'}: RMS={result.rms_error*100:.1f} cm (threshold: 15 cm)")
    return ok


if __name__ == "__main__":
    ok = _self_test()
    sys.exit(0 if ok else 1)
