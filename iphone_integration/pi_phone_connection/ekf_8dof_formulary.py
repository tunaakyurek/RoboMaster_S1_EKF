"""
8-DOF Extended Kalman Filter Implementation
===========================================
Following RoboMaster EKF Formulary specifications
Optimized for iPhone sensor data integration

This implementation uses a simplified 8-DOF state vector focusing on
the essential states for drone navigation while maintaining
mathematical rigor from the formulary.

State Vector (8-DOF):
[x, y, z, vz, roll, pitch, yaw, yaw_rate]

Author: RoboMaster EKF Integration System
Date: 2025
"""

import numpy as np
from dataclasses import dataclass
from typing import Optional, Tuple, Dict, Any
import logging
import time

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


@dataclass
class EKF8State:
    """
    8-DOF EKF state vector following formulary specifications
    Simplified from 12-DOF for computational efficiency
    """
    # Position states (3-DOF)
    x: float          # meters (North/Forward)
    y: float          # meters (East/Right)
    z: float          # meters (Down/Altitude)
    
    # Velocity state (1-DOF) 
    vz: float         # m/s (vertical velocity)
    
    # Orientation states (3-DOF)
    roll: float       # radians
    pitch: float      # radians
    yaw: float        # radians
    
    # Angular velocity state (1-DOF)
    yaw_rate: float   # rad/s
    
    def to_array(self) -> np.ndarray:
        """Convert to numpy array"""
        return np.array([
            self.x, self.y, self.z, self.vz,
            self.roll, self.pitch, self.yaw, self.yaw_rate
        ])
    
    @classmethod
    def from_array(cls, arr: np.ndarray) -> 'EKF8State':
        """Create from numpy array"""
        return cls(
            x=arr[0], y=arr[1], z=arr[2], vz=arr[3],
            roll=arr[4], pitch=arr[5], yaw=arr[6], yaw_rate=arr[7]
        )
    
    def __repr__(self) -> str:
        return (f"EKF8State(pos=[{self.x:.2f}, {self.y:.2f}, {self.z:.2f}], "
                f"vz={self.vz:.2f}, rpy=[{np.degrees(self.roll):.1f}°, "
                f"{np.degrees(self.pitch):.1f}°, {np.degrees(self.yaw):.1f}°], "
                f"yaw_rate={np.degrees(self.yaw_rate):.1f}°/s)")


class EKF8DOF:
    """
    8-DOF Extended Kalman Filter for drone state estimation
    Following RoboMaster EKF Formulary mathematical specifications
    """
    
    def __init__(self, config: Optional[Dict[str, Any]] = None):
        """
        Initialize 8-DOF EKF
        
        Args:
            config: Optional configuration dictionary
        """
        # State dimension
        self.n_states = 8
        
        # Initialize state vector [x, y, z, vz, roll, pitch, yaw, yaw_rate]
        self.x = np.zeros(self.n_states)
        
        # Initialize covariance matrix
        self.P = np.eye(self.n_states)
        self.P[0:3, 0:3] *= 1.0    # Position uncertainty
        self.P[3, 3] = 0.5         # Vertical velocity uncertainty
        self.P[4:7, 4:7] *= 0.1    # Orientation uncertainty
        self.P[7, 7] = 0.1         # Yaw rate uncertainty
        
        # Process noise covariance Q (as per formulary)
        self.Q = self._create_process_noise_matrix(config)
        
        # Measurement noise covariances R (as per formulary)
        self.R_accel = np.eye(3) * 0.5     # Accelerometer noise
        self.R_gyro = np.eye(3) * 0.1      # Gyroscope noise
        self.R_mag = np.eye(3) * 0.5       # Magnetometer noise
        self.R_gps = np.eye(3) * 1.0       # GPS noise
        self.R_baro = np.array([[0.1]])    # Barometer noise
        
        # Physical constants (as per formulary)
        self.gravity = 9.81  # m/s²
        
        # Drone-specific parameters
        self.drone_velocity = 0.0  # Current forward velocity (m/s)
        self.use_drone_velocity = config.get('use_drone_velocity', False) if config else False
        
        # Time tracking
        self.last_update_time = None
        
        # Statistics
        self.update_count = 0
        self.prediction_count = 0
        
        logger.info("8-DOF EKF initialized following formulary specifications")
    
    def _create_process_noise_matrix(self, config: Optional[Dict] = None) -> np.ndarray:
        """
        Create process noise covariance matrix Q
        Following formulary specifications
        """
        Q = np.zeros((self.n_states, self.n_states))
        
        # Default values from formulary
        q_pos = 0.01      # Position process noise
        q_vz = 0.1        # Vertical velocity process noise
        q_orient = 0.05   # Orientation process noise
        q_yaw_rate = 0.1  # Yaw rate process noise
        
        # Override with config if provided
        if config:
            q_pos = config.get('q_position', q_pos)
            q_vz = config.get('q_velocity', q_vz)
            q_orient = config.get('q_orientation', q_orient)
            q_yaw_rate = config.get('q_angular_velocity', q_yaw_rate)
        
        # Set diagonal elements
        Q[0:3, 0:3] = np.eye(3) * q_pos      # Position noise
        Q[3, 3] = q_vz                       # Vertical velocity noise
        Q[4:7, 4:7] = np.eye(3) * q_orient   # Orientation noise
        Q[7, 7] = q_yaw_rate                 # Yaw rate noise
        
        return Q
    
    def predict(self, dt: float):
        """
        Prediction step following formulary equations:
        x_k = F_k * x_{k-1}
        P_k = F_k * P_{k-1} * F_k^T + Q_k
        
        Args:
            dt: Time step in seconds
        """
        if dt <= 0:
            logger.warning(f"Invalid dt: {dt}. Skipping prediction.")
            return
        
        # State transition matrix F (8x8)
        F = self._compute_state_transition_matrix(dt)
        
        # Predict state: x = F * x
        self.x = F @ self.x
        
        # Normalize angles
        self.x[4:7] = self._normalize_angles(self.x[4:7])
        
        # Predict covariance: P = F * P * F^T + Q * dt
        self.P = F @ self.P @ F.T + self.Q * dt
        
        self.prediction_count += 1
        
        logger.debug(f"Prediction step completed (dt={dt:.3f}s)")
    
    def _compute_state_transition_matrix(self, dt: float) -> np.ndarray:
        """
        Compute state transition matrix F for 8-DOF system
        Following kinematic model from formulary
        
        State: [x, y, z, vz, roll, pitch, yaw, yaw_rate]
        """
        F = np.eye(self.n_states)
        
        # Position updates
        if self.use_drone_velocity and self.drone_velocity != 0:
            # Use drone velocity for horizontal position update
            yaw = self.x[6]
            F[0, 6] = -self.drone_velocity * np.sin(yaw) * dt  # dx/dyaw
            F[1, 6] = self.drone_velocity * np.cos(yaw) * dt   # dy/dyaw
        
        # Vertical position from vertical velocity
        F[2, 3] = dt  # z += vz * dt
        
        # Yaw from yaw rate
        F[6, 7] = dt  # yaw += yaw_rate * dt
        
        return F
    
    def update_imu(self, accel: np.ndarray, gyro: np.ndarray):
        """
        Update with IMU measurements (accelerometer and gyroscope)
        Following formulary measurement model
        
        Args:
            accel: Accelerometer measurements [ax, ay, az] in m/s²
            gyro: Gyroscope measurements [wx, wy, wz] in rad/s
        """
        # Measurement vector
        z = np.concatenate([accel, gyro])
        
        # Expected measurements
        h = self._compute_expected_imu()
        
        # Measurement Jacobian
        H = self._compute_imu_jacobian()
        
        # Measurement noise
        R = np.block([
            [self.R_accel, np.zeros((3, 3))],
            [np.zeros((3, 3)), self.R_gyro]
        ])
        
        # Kalman update
        self._kalman_update(z, h, H, R)
        
        logger.debug("IMU update completed")
    
    def _compute_expected_imu(self) -> np.ndarray:
        """
        Compute expected IMU measurements
        Following formulary: gravity in body frame
        """
        roll, pitch, yaw = self.x[4:7]
        yaw_rate = self.x[7]
        
        # Expected acceleration (gravity in body frame)
        # Following formulary equations
        expected_accel = np.array([
            -self.gravity * np.sin(pitch),
            self.gravity * np.sin(roll) * np.cos(pitch),
            self.gravity * np.cos(roll) * np.cos(pitch)
        ])
        
        # Expected gyroscope (only yaw rate for 8-DOF)
        expected_gyro = np.array([0.0, 0.0, yaw_rate])
        
        return np.concatenate([expected_accel, expected_gyro])
    
    def _compute_imu_jacobian(self) -> np.ndarray:
        """
        Compute Jacobian matrix for IMU measurements
        H = ∂h/∂x following formulary
        """
        roll, pitch = self.x[4:6]
        
        H = np.zeros((6, self.n_states))
        
        # Accelerometer Jacobian (w.r.t. roll and pitch)
        # ∂ax/∂pitch
        H[0, 5] = -self.gravity * np.cos(pitch)
        
        # ∂ay/∂roll
        H[1, 4] = self.gravity * np.cos(roll) * np.cos(pitch)
        # ∂ay/∂pitch
        H[1, 5] = -self.gravity * np.sin(roll) * np.sin(pitch)
        
        # ∂az/∂roll
        H[2, 4] = -self.gravity * np.sin(roll) * np.cos(pitch)
        # ∂az/∂pitch
        H[2, 5] = -self.gravity * np.cos(roll) * np.sin(pitch)
        
        # Gyroscope Jacobian (only yaw rate)
        H[5, 7] = 1.0  # ∂wz/∂yaw_rate
        
        return H
    
    def update_magnetometer(self, mag: np.ndarray):
        """
        Update with magnetometer measurements for heading correction
        
        Args:
            mag: Magnetometer measurements [mx, my, mz] in μT
        """
        # For 8-DOF, we primarily use magnetometer for yaw correction
        # Simplified measurement model
        
        # Expected heading from magnetometer
        mag_heading = np.arctan2(mag[1], mag[0])
        
        # Measurement
        z = np.array([mag_heading])
        
        # Expected measurement (current yaw)
        h = np.array([self.x[6]])
        
        # Jacobian
        H = np.zeros((1, self.n_states))
        H[0, 6] = 1.0  # ∂heading/∂yaw
        
        # Measurement noise
        R = np.array([[0.1]])  # Heading uncertainty
        
        # Kalman update
        self._kalman_update(z, h, H, R)
        
        logger.debug("Magnetometer update completed")
    
    def update_gps(self, lat: float, lon: float, alt: float, 
                   reference_lat: Optional[float] = None,
                   reference_lon: Optional[float] = None):
        """
        Update with GPS measurements
        
        Args:
            lat: Latitude in degrees
            lon: Longitude in degrees  
            alt: Altitude in meters
            reference_lat: Reference latitude for local coordinates
            reference_lon: Reference longitude for local coordinates
        """
        # Convert GPS to local coordinates
        if reference_lat is None or reference_lon is None:
            # Use current position as reference
            logger.warning("No GPS reference provided, skipping update")
            return
        
        # Simple lat/lon to meters conversion (approximate)
        lat_to_m = 111320.0
        lon_to_m = 111320.0 * np.cos(np.radians(reference_lat))
        
        x_gps = (lat - reference_lat) * lat_to_m
        y_gps = (lon - reference_lon) * lon_to_m
        z_gps = -alt  # NED convention (down is positive)
        
        # Measurement vector
        z = np.array([x_gps, y_gps, z_gps])
        
        # Expected measurement (current position)
        h = self.x[0:3]
        
        # Jacobian
        H = np.zeros((3, self.n_states))
        H[0:3, 0:3] = np.eye(3)
        
        # Kalman update
        self._kalman_update(z, h, H, self.R_gps)
        
        logger.debug("GPS update completed")
    
    def update_barometer(self, altitude: float):
        """
        Update with barometer altitude measurement
        
        Args:
            altitude: Altitude in meters
        """
        # Measurement (convert to NED: down is positive)
        z = np.array([-altitude])
        
        # Expected measurement (current z position)
        h = np.array([self.x[2]])
        
        # Jacobian
        H = np.zeros((1, self.n_states))
        H[0, 2] = 1.0  # ∂altitude/∂z
        
        # Kalman update
        self._kalman_update(z, h, H, self.R_baro)
        
        logger.debug("Barometer update completed")
    
    def _kalman_update(self, z: np.ndarray, h: np.ndarray, 
                       H: np.ndarray, R: np.ndarray):
        """
        Perform Kalman update step following formulary:
        y = z - h (innovation)
        S = H * P * H^T + R (innovation covariance)
        K = P * H^T * S^(-1) (Kalman gain)
        x = x + K * y (state update)
        P = (I - K * H) * P * (I - K * H)^T + K * R * K^T (covariance update)
        
        Args:
            z: Measurement vector
            h: Expected measurement vector
            H: Measurement Jacobian matrix
            R: Measurement noise covariance
        """
        # Innovation
        y = z - h
        
        # Handle angle wrapping for heading measurements
        if len(y) == 1 and H[0, 6] != 0:  # Heading measurement
            y[0] = self._normalize_angle(y[0])
        
        # Innovation covariance
        S = H @ self.P @ H.T + R
        
        # Kalman gain
        try:
            K = self.P @ H.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            logger.warning("Singular matrix in Kalman gain calculation")
            return
        
        # Update state
        self.x = self.x + K @ y
        
        # Normalize angles
        self.x[4:7] = self._normalize_angles(self.x[4:7])
        
        # Update covariance (Joseph form for numerical stability)
        I_KH = np.eye(self.n_states) - K @ H
        KRKt = K @ R @ K.T
        self.P = I_KH @ self.P @ I_KH.T + KRKt
        
        self.update_count += 1
    
    def set_drone_velocity(self, velocity: float):
        """
        Set drone forward velocity for position prediction
        
        Args:
            velocity: Forward velocity in m/s
        """
        self.drone_velocity = velocity
    
    def get_state(self) -> EKF8State:
        """Get current state as EKF8State object"""
        return EKF8State.from_array(self.x)
    
    def get_covariance(self) -> np.ndarray:
        """Get current covariance matrix"""
        return self.P.copy()
    
    def get_position_uncertainty(self) -> np.ndarray:
        """Get position uncertainty (standard deviation)"""
        return np.sqrt(np.diag(self.P[0:3, 0:3]))
    
    def get_orientation_uncertainty(self) -> np.ndarray:
        """Get orientation uncertainty (standard deviation)"""
        return np.sqrt(np.diag(self.P[4:7, 4:7]))
    
    def reset(self, initial_state: Optional[EKF8State] = None):
        """
        Reset filter to initial conditions
        
        Args:
            initial_state: Optional initial state
        """
        if initial_state:
            self.x = initial_state.to_array()
        else:
            self.x = np.zeros(self.n_states)
        
        # Reset covariance
        self.P = np.eye(self.n_states)
        self.P[0:3, 0:3] *= 1.0    # Position uncertainty
        self.P[3, 3] = 0.5         # Vertical velocity uncertainty
        self.P[4:7, 4:7] *= 0.1    # Orientation uncertainty
        self.P[7, 7] = 0.1         # Yaw rate uncertainty
        
        # Reset statistics
        self.update_count = 0
        self.prediction_count = 0
        self.last_update_time = None
        
        logger.info("EKF reset")
    
    def _normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-π, π]"""
        return (angle + np.pi) % (2 * np.pi) - np.pi
    
    def _normalize_angles(self, angles: np.ndarray) -> np.ndarray:
        """Normalize multiple angles to [-π, π]"""
        return (angles + np.pi) % (2 * np.pi) - np.pi
    
    def get_statistics(self) -> Dict[str, Any]:
        """Get filter statistics"""
        return {
            'update_count': self.update_count,
            'prediction_count': self.prediction_count,
            'covariance_trace': np.trace(self.P),
            'position': self.x[0:3].tolist(),
            'orientation_deg': np.degrees(self.x[4:7]).tolist(),
            'position_uncertainty': self.get_position_uncertainty().tolist(),
            'orientation_uncertainty_deg': np.degrees(self.get_orientation_uncertainty()).tolist()
        }


# Example usage and testing
if __name__ == "__main__":
    # Create EKF
    config = {
        'q_position': 0.01,
        'q_velocity': 0.1,
        'q_orientation': 0.05,
        'q_angular_velocity': 0.1,
        'use_drone_velocity': True
    }
    
    ekf = EKF8DOF(config)
    
    # Simulate some updates
    dt = 0.02  # 50 Hz
    
    for i in range(100):
        # Prediction
        ekf.predict(dt)
        
        # Simulated IMU data
        accel = np.array([0.1, 0.0, 9.81]) + np.random.randn(3) * 0.1
        gyro = np.array([0.0, 0.0, 0.1]) + np.random.randn(3) * 0.01
        
        # Update
        ekf.update_imu(accel, gyro)
        
        # Print state every 20 iterations
        if i % 20 == 0:
            state = ekf.get_state()
            print(f"Iteration {i}: {state}")
    
    # Final statistics
    stats = ekf.get_statistics()
    print(f"\nFinal statistics: {stats}")
