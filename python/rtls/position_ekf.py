"""
Constant-velocity Kalman filter over the multilaterated fix.

Port of MATLAB +rtls/PositionEKF.m.
State = [pos(dim); vel(dim)].
"""

import numpy as np
from typing import Optional, Tuple


class PositionEKF:
    """
    A discrete-time linear Kalman Filter with a constant-velocity (CV) model.
    It smooths the noisy (X, Y) positions coming from the multilateration solver,
    incorporating a mathematical model of physics (velocity and random acceleration).
    """
    def __init__(self, dim=2):
        self.dim = dim
        n = 2 * dim
        # State vector: [x, y, vx, vy] (if dim=2)
        self.x = np.zeros(n)
        # Covariance matrix P: Uncertainty of the current state estimate
        self.P = np.eye(n)
        
        # Tuning parameters
        # q_accel: Process noise power spectral density (PSD).
        # Controls how much random acceleration we expect.
        # High Q -> Trust measurements more, track turns faster, but more jitter.
        # Low Q -> Trust model more, very smooth path, but lags behind sharp turns.
        self.q_accel   = 1.0      
        
        # pos_sigma: Default measurement standard deviation (in meters).
        # Used if the multilaterator fails to provide a covariance matrix.
        self.pos_sigma = 0.10     
        self.initialized = False

    def initialize(self, pos0: np.ndarray):
        """
        Initializes the filter state with an initial position measurement.
        Assumes initial velocity is 0.
        """
        pos0 = np.asarray(pos0).ravel()
        self.x = np.concatenate([pos0, np.zeros(self.dim)])
        # Initialize covariance: Moderate uncertainty for position, high for velocity
        self.P = np.block([
            [np.eye(self.dim) * self.pos_sigma**2, np.zeros((self.dim, self.dim))],
            [np.zeros((self.dim, self.dim)),        np.eye(self.dim) * 1.0],
        ])
        self.initialized = True

    def predict(self, dt: float):
        """
        Time Update (Predict) step.
        Advances the state based on the constant-velocity model.
        """
        d = self.dim
        I = np.eye(d)
        Z = np.zeros((d, d))
        
        # State Transition Matrix F
        # New pos = pos + vel * dt
        # New vel = vel
        F = np.block([[I, dt*I], [Z, I]])
        
        # Process Noise Covariance Matrix Q (Continuous White Noise Acceleration Model)
        # Represents the uncertainty added to the system over time dt due to unknown accelerations
        q = self.q_accel
        Q = q * np.block([
            [(dt**3/3)*I, (dt**2/2)*I],
            [(dt**2/2)*I,  dt*I],
        ])
        
        # 1. Project the state ahead
        self.x = F @ self.x
        
        # 2. Project the error covariance ahead
        self.P = F @ self.P @ F.T + Q

    def update(self, z: np.ndarray, R: Optional[np.ndarray] = None):
        """
        Measurement Update (Correct) step.
        Fuses the predicted state with the new position measurement.
        """
        if R is None:
            # Fallback measurement covariance if LM didn't provide one
            R = np.eye(self.dim) * self.pos_sigma**2
            
        d = self.dim
        # Observation Matrix H: We only measure position, not velocity
        # z = H * x = [I | 0] * [pos; vel] = pos
        H = np.hstack([np.eye(d), np.zeros((d, d))])
        
        # Measurement Residual y (Innovation)
        y = z.ravel() - H @ self.x
        
        # Innovation Covariance S
        S = H @ self.P @ H.T + R
        
        # Optimal Kalman Gain K
        # Determines how much we trust the new measurement vs the prediction
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # Update the state estimate
        self.x = self.x + K @ y
        
        # Update the error covariance matrix
        n = len(self.x)
        self.P = (np.eye(n) - K @ H) @ self.P

    def step(self, dt: float, z: np.ndarray,
             R: Optional[np.ndarray] = None) -> Tuple[np.ndarray, np.ndarray]:
        """
        Convenience wrapper that runs a full Predict + Update cycle.
        
        Args:
            dt: Time elapsed since last step (seconds).
            z: Measured position from multilateration.
            R: Measurement covariance matrix from multilateration (optional).
            
        Returns:
            pos: Smoothed position.
            vel: Estimated velocity.
        """
        z = np.asarray(z).ravel()
        if not self.initialized:
            self.initialize(z)
        else:
            self.predict(dt)
            if np.all(np.isfinite(z)):
                self.update(z, R)
                
        pos = self.x[:self.dim].copy()
        vel = self.x[self.dim:].copy()
        return pos, vel
