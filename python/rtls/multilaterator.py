"""
Overdetermined N-anchor position solver (Levenberg-Marquardt).

Port of MATLAB +rtls/Multilaterator.m — toolbox-free LM with robust gating.
"""

import numpy as np
from dataclasses import dataclass, field
from typing import Optional


@dataclass
class SolveInfo:
    """Information about the multilateration solve attempt."""
    ok:    bool = False
    used:  np.ndarray = field(default_factory=lambda: np.array([], dtype=bool))
    rms:   float = float('nan')
    cov:   np.ndarray = field(default_factory=lambda: np.full((2,2), float('nan')))
    iters: int = 0


class Multilaterator:
    """
    Solves for the tag position given N anchor coordinates and distances.
    Uses the Levenberg-Marquardt (LM) non-linear least-squares algorithm.
    """
    def __init__(self, dim=2):
        self.dim        = dim
        self.use_gating = True
        self.gate_k     = 3.0
        self.max_iters  = 30
        self.range_sigma = 0.10

    # ── public ────────────────────────────────────────────────
    def solve(self, A: np.ndarray, d: np.ndarray,
              x0: Optional[np.ndarray] = None) -> tuple:
        """
        Solves the position using Levenberg-Marquardt, with optional robust gating.
        
        Args:
            A: (N, dim) array of anchor coordinates.
            d: (N,) array of measured distances.
            x0: Initial guess for the tag position. If None, uses the centroid of A.
            
        Returns:
            pos: (dim,) estimated tag position.
            info: SolveInfo containing status, covariance, and used anchors.
        """
        d = np.asarray(d, dtype=float).ravel()
        N = A.shape[0]
        info = SolveInfo(used=np.ones(N, dtype=bool),
                         cov=np.full((self.dim, self.dim), float('nan')))
        # We need at least dim + 1 anchors to uniquely resolve position
        if N < self.dim + 1:
            return np.full(self.dim, float('nan')), info

        if x0 is None:
            # Centroid of the anchors is a safe initial guess
            x0 = np.mean(A, axis=0)
        x0 = np.asarray(x0, dtype=float).ravel()

        # First solve with all anchors
        pos, it1 = self._lm(A, d, x0.copy())
        used = np.ones(N, dtype=bool)

        # Robust residual gating: discard outliers (e.g. NLOS bounces) and re-solve
        if self.use_gating and N >= self.dim + 2:
            r   = self._residuals(pos, A, d)
            med = np.median(r)
            # Median Absolute Deviation (MAD) is robust to outliers
            mad = np.median(np.abs(r - med))
            # 1.4826 scales MAD to be asymptotically consistent with standard deviation
            scale = max(1.4826 * mad, 1e-3)
            
            # Keep anchors whose residuals are within gate_k standard deviations
            used = np.abs(r - med) <= self.gate_k * scale
            
            # If we threw out some anchors but still have enough to solve, re-run LM
            if np.sum(used) >= self.dim + 1 and np.sum(used) < N:
                pos, it2 = self._lm(A[used], d[used], pos.copy())
                it1 += it2
            else:
                # If we threw out too many, fall back to using all anchors
                used = np.ones(N, dtype=bool)

        # Final residual calculation for stats
        r   = self._residuals(pos, A[used], d[used])
        info.ok    = True
        info.used  = used
        info.rms   = float(np.sqrt(np.mean(r**2)))
        info.iters = it1

        # Covariance approximation: σ² · inv(J'J)
        # Used by the EKF to weight the trust in this measurement
        J = self._jacobian(pos, A[used])
        H = J.T @ J
        if np.linalg.cond(H) < 1e12:
            info.cov = (self.range_sigma**2) * np.linalg.inv(H)

        return pos, info

    # ── private ───────────────────────────────────────────────
    def _lm(self, A, d, x):
        """
        Core Levenberg-Marquardt optimizer loop.
        Minimizes sum( (||x - A_i|| - d_i)^2 ).
        """
        lam = 1e-3
        r = self._residuals(x, A, d)
        prev_cost = r @ r
        iters = 0
        for it in range(self.max_iters):
            iters = it + 1
            J = self._jacobian(x, A)
            
            # Normal equations matrix: H = J^T J
            H = J.T @ J
            # Gradient: g = J^T r
            g = J.T @ r
            
            # LM step: solve (H + lambda * diag(H)) * step = -g
            # The lambda term interpolates between Gauss-Newton and Gradient Descent
            step = -np.linalg.solve(H + lam * np.diag(np.diag(H) + 1e-9), g)
            xn = x + step
            
            rn = self._residuals(xn, A, d)
            cost = rn @ rn
            
            if cost < prev_cost:
                # Step was good, accept it and decrease lambda (move towards Gauss-Newton)
                x, r, prev_cost = xn, rn, cost
                lam = max(lam / 3, 1e-9)
                if np.linalg.norm(step) < 1e-6:
                    break
            else:
                # Step was bad, reject it and increase lambda (move towards Gradient Descent)
                lam = min(lam * 3, 1e9)
        return x, iters

    @staticmethod
    def _residuals(x, A, d):
        """
        Calculates the error between the geometric distance and the measured distance.
        r_i = ||x - A_i|| - d_i
        """
        return np.linalg.norm(x - A, axis=1) - d

    @staticmethod
    def _jacobian(x, A):
        """
        Calculates the Jacobian matrix (partial derivatives of the residuals w.r.t x).
        J_{i,j} = (x_j - A_{i,j}) / ||x - A_i||
        """
        diff = x - A
        rng  = np.linalg.norm(diff, axis=1, keepdims=True)
        rng  = np.maximum(rng, 1e-6) # Prevent divide-by-zero if tag is exactly on an anchor
        return diff / rng

