"""
RoboMaster 8-DOF Extended Kalman Filter Implementation
=====================================================
Following EXACT RoboMaster EKF Formulary specifications
As specified in RoboMaster_Formulary.pdf

State Vector (8-DOF):
[x, y, theta, vx, vy, bias_accel_x, bias_accel_y, bias_angular_velocity]

Where:
- x, y: Position in global frame (meters)
- theta: Yaw angle (radians)
- vx, vy: Velocity in global frame (m/s)
- bias_accel_x, bias_accel_y: Accelerometer biases (m/s²)
- bias_angular_velocity: Gyroscope bias (rad/s)

Author: RoboMaster EKF Integration System
Date: 2025
"""

import numpy as np
from dataclasses import dataclass
from typing import Optional, Dict, Any
import logging
import time

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


@dataclass
class RoboMasterState:
    """
    RoboMaster 8-DOF EKF state vector
    Following exact formulary specifications
    """
    # Position (2-DOF)
    x: float                    # meters (global X position)
    y: float                    # meters (global Y position)
    
    # Orientation (1-DOF)
    theta: float                # radians (yaw angle)
    
    # Velocity (2-DOF)
    vx: float                   # m/s (global X velocity)
    vy: float                   # m/s (global Y velocity)
    
    # Sensor biases (3-DOF)
    bias_accel_x: float         # m/s² (accelerometer X bias)
    bias_accel_y: float         # m/s² (accelerometer Y bias)
    bias_angular_velocity: float # rad/s (gyroscope Z bias)
    
    def to_array(self) -> np.ndarray:
        """Convert to numpy array following formulary order"""
        return np.array([
            self.x, self.y, self.theta,
            self.vx, self.vy,
            self.bias_accel_x, self.bias_accel_y, self.bias_angular_velocity
        ])
    
    @classmethod
    def from_array(cls, arr: np.ndarray) -> 'RoboMasterState':
        """Create from numpy array"""
        return cls(
            x=arr[0], y=arr[1], theta=arr[2],
            vx=arr[3], vy=arr[4],
            bias_accel_x=arr[5], bias_accel_y=arr[6], 
            bias_angular_velocity=arr[7]
        )
    
    def get_position(self) -> np.ndarray:
        """Get position vector [x, y]"""
        return np.array([self.x, self.y])
    
    def get_velocity(self) -> np.ndarray:
        """Get velocity vector [vx, vy]"""
        return np.array([self.vx, self.vy])
    
    def get_biases(self) -> np.ndarray:
        """Get bias vector [bias_accel_x, bias_accel_y, bias_angular_velocity]"""
        return np.array([self.bias_accel_x, self.bias_accel_y, self.bias_angular_velocity])
    
    def __repr__(self) -> str:
        return (f"RoboMasterState(pos=[{self.x:.2f}, {self.y:.2f}], "
                f"theta={np.degrees(self.theta):.1f}°, "
                f"vel=[{self.vx:.2f}, {self.vy:.2f}], "
                f"biases=[{self.bias_accel_x:.3f}, {self.bias_accel_y:.3f}, {self.bias_angular_velocity:.3f}])")


class RoboMasterEKF8DOF:
    """
    RoboMaster 8-DOF Extended Kalman Filter
    Implements exact formulary specifications from RoboMaster_Formulary.pdf
    
    State vector: [x, y, theta, vx, vy, bias_accel_x, bias_accel_y, bias_angular_velocity]
    """
    
    def __init__(self, config: Optional[Dict[str, Any]] = None):
        """
        Initialize RoboMaster 8-DOF EKF following formulary specifications
        
        Args:
            config: Configuration dictionary with noise parameters
        """
        self.n_states = 8
        config = config or {}
        
        # State vector initialization
        self.x = np.zeros(self.n_states)
        
        # Initial state covariance P (following formulary)
        self.P = np.eye(self.n_states)
        self.P[0:2, 0:2] *= config.get('init_pos_var', 1.0)      # Position uncertainty
        self.P[2, 2] = config.get('init_theta_var', 0.1)         # Orientation uncertainty  
        self.P[3:5, 3:5] *= config.get('init_vel_var', 0.5)     # Velocity uncertainty
        self.P[5:7, 5:7] *= config.get('init_accel_bias_var', 0.01)  # Accel bias uncertainty
        self.P[7, 7] = config.get('init_gyro_bias_var', 0.001)   # Gyro bias uncertainty
        
        # Process noise covariance Q (following formulary)
        self.Q = self._create_process_noise_matrix(config)
        
        # Measurement noise covariances R (following formulary)
        self.R_accel = np.eye(2) * config.get('r_accel', 0.1)   # Accelerometer noise (2D)
        self.R_gyro = np.array([[config.get('r_gyro', 0.01)]])  # Gyroscope noise (1D)
        self.R_gps_pos = np.eye(2) * config.get('r_gps_pos', 1.0)     # GPS position noise
        self.R_gps_vel = np.eye(2) * config.get('r_gps_vel', 0.5)     # GPS velocity noise
        
        # Physical constants
        self.gravity = config.get('gravity', 9.81)  # m/s²
        
        # Statistics
        self.update_count = 0
        self.prediction_count = 0
        
        logger.info("RoboMaster 8-DOF EKF initialized following exact formulary specifications")
    
    def _create_process_noise_matrix(self, config: Dict) -> np.ndarray:
        """
        Create process noise covariance matrix Q following formulary
        """
        Q = np.zeros((self.n_states, self.n_states))
        
        # Process noise parameters (from formulary or config)
        q_pos = config.get('q_position', 0.01)          # Position process noise
        q_theta = config.get('q_theta', 0.01)           # Orientation process noise
        q_vel = config.get('q_velocity', 0.1)           # Velocity process noise
        q_accel_bias = config.get('q_accel_bias', 1e-6) # Accelerometer bias drift
        q_gyro_bias = config.get('q_gyro_bias', 1e-8)   # Gyroscope bias drift
        
        # Set diagonal elements
        Q[0:2, 0:2] = np.eye(2) * q_pos         # Position noise
        Q[2, 2] = q_theta                       # Orientation noise
        Q[3:5, 3:5] = np.eye(2) * q_vel         # Velocity noise
        Q[5:7, 5:7] = np.eye(2) * q_accel_bias  # Accelerometer bias drift
        Q[7, 7] = q_gyro_bias                   # Gyroscope bias drift
        
        return Q
    
    def predict(self, dt: float, control_input: Optional[np.ndarray] = None):
        """
        Prediction step following RoboMaster formulary motion model
        
        Args:
            dt: Time step (seconds)
            control_input: Optional control inputs [ax_control, ay_control, omega_control]
        """
        if dt <= 0:
            logger.warning(f"Invalid dt: {dt}. Skipping prediction.")
            return
        
        # Current state
        x, y, theta = self.x[0:3]
        vx, vy = self.x[3:5]
        bias_ax, bias_ay, bias_omega = self.x[5:8]
        
        # Motion model following formulary
        # Position update: p_k+1 = p_k + v_k * dt
        self.x[0] = x + vx * dt     # x += vx * dt
        self.x[1] = y + vy * dt     # y += vy * dt
        
        # Orientation update: theta_k+1 = theta_k + omega * dt
        if control_input is not None and len(control_input) >= 3:
            omega_measured = control_input[2]
            omega_true = omega_measured - bias_omega
            self.x[2] = theta + omega_true * dt
        # If no control input, orientation remains constant
        
        # Velocity update with control input
        if control_input is not None and len(control_input) >= 2:
            ax_measured, ay_measured = control_input[0:2]
            # Remove bias and apply in global frame
            ax_true = ax_measured - bias_ax
            ay_true = ay_measured - bias_ay
            
            # Transform acceleration from body to global frame
            cos_theta = np.cos(theta)
            sin_theta = np.sin(theta)
            
            ax_global = ax_true * cos_theta - ay_true * sin_theta
            ay_global = ax_true * sin_theta + ay_true * cos_theta
            
            self.x[3] = vx + ax_global * dt  # vx += ax_global * dt
            self.x[4] = vy + ay_global * dt  # vy += ay_global * dt
        
        # Biases remain constant (random walk model)
        # bias_k+1 = bias_k (handled by process noise)
        
        # Normalize angle
        self.x[2] = self._normalize_angle(self.x[2])
        
        # Compute state transition Jacobian
        F = self._compute_state_jacobian(dt, control_input)
        
        # Predict covariance: P = F * P * F^T + Q
        self.P = F @ self.P @ F.T + self.Q * dt
        
        self.prediction_count += 1
        
        logger.debug(f"RoboMaster prediction completed (dt={dt:.3f}s)")
    
    def _compute_state_jacobian(self, dt: float, control_input: Optional[np.ndarray]) -> np.ndarray:
        """
        Compute state transition Jacobian F following formulary
        
        State: [x, y, theta, vx, vy, bias_ax, bias_ay, bias_omega]
        """
        F = np.eye(self.n_states)
        
        # Position derivatives
        F[0, 3] = dt  # ∂x/∂vx
        F[1, 4] = dt  # ∂y/∂vy
        
        # Orientation derivatives (if angular velocity provided)
        if control_input is not None and len(control_input) >= 3:
            F[2, 7] = -dt  # ∂theta/∂bias_omega (negative because omega_true = omega_meas - bias)
        
        # Velocity derivatives (if acceleration provided)
        if control_input is not None and len(control_input) >= 2:
            theta = self.x[2]
            cos_theta = np.cos(theta)
            sin_theta = np.sin(theta)
            
            ax_meas, ay_meas = control_input[0:2]
            bias_ax, bias_ay = self.x[5:7]
            
            ax_true = ax_meas - bias_ax
            ay_true = ay_meas - bias_ay
            
            # ∂vx/∂theta
            F[3, 2] = dt * (-ax_true * sin_theta - ay_true * cos_theta)
            # ∂vy/∂theta  
            F[4, 2] = dt * (ax_true * cos_theta - ay_true * sin_theta)
            
            # ∂vx/∂bias_ax, ∂vx/∂bias_ay
            F[3, 5] = -dt * cos_theta  # ∂vx/∂bias_ax
            F[3, 6] = dt * sin_theta   # ∂vx/∂bias_ay
            
            # ∂vy/∂bias_ax, ∂vy/∂bias_ay
            F[4, 5] = -dt * sin_theta  # ∂vy/∂bias_ax
            F[4, 6] = -dt * cos_theta  # ∂vy/∂bias_ay
        
        return F
    
    def update_imu(self, accel_body: np.ndarray, gyro_z: float):
        """
        Update with IMU measurements following formulary sensor model
        
        Args:
            accel_body: Body frame acceleration [ax, ay] (m/s²)
            gyro_z: Angular velocity measurement (rad/s)
        """
        # Measurement vector
        z = np.concatenate([accel_body, [gyro_z]])
        
        # Expected measurements
        h = self._compute_expected_imu()
        
        # Measurement Jacobian
        H = self._compute_imu_jacobian()
        
        # Combined measurement noise
        R = np.block([
            [self.R_accel, np.zeros((2, 1))],
            [np.zeros((1, 2)), self.R_gyro]
        ])
        
        # Kalman update
        self._kalman_update(z, h, H, R)
        
        logger.debug("RoboMaster IMU update completed")
    
    def _compute_expected_imu(self) -> np.ndarray:
        """
        Compute expected IMU measurements following formulary
        
        Expected accelerometer: measures gravity + linear acceleration in body frame
        Expected gyroscope: measures angular velocity + bias
        """
        theta = self.x[2]
        vx, vy = self.x[3:5]
        bias_ax, bias_ay, bias_omega = self.x[5:8]
        
        # Expected accelerometer measurements in body frame
        # For a ground vehicle, this is primarily gravity projection
        # Plus any centripetal acceleration from turning
        
        # Gravity in body frame (assuming level ground)
        gravity_body_x = 0.0  # No pitch assumed for ground vehicle
        gravity_body_y = 0.0  # No roll assumed for ground vehicle
        
        # Add bias
        expected_accel = np.array([
            gravity_body_x + bias_ax,
            gravity_body_y + bias_ay
        ])
        
        # Expected gyroscope measurement (angular velocity + bias)
        # For ground vehicle, this is yaw rate
        # We need to estimate current yaw rate from velocity and path curvature
        velocity_magnitude = np.sqrt(vx**2 + vy**2)
        if velocity_magnitude > 0.1:  # Avoid division by zero
            # Estimate yaw rate from velocity direction change
            # This is an approximation - could be improved with vehicle model
            estimated_yaw_rate = 0.0  # Simplified for now
        else:
            estimated_yaw_rate = 0.0
        
        expected_gyro = estimated_yaw_rate + bias_omega
        
        return np.concatenate([expected_accel, [expected_gyro]])
    
    def _compute_imu_jacobian(self) -> np.ndarray:
        """
        Compute Jacobian matrix for IMU measurements
        H = ∂h/∂x following formulary
        """
        H = np.zeros((3, self.n_states))
        
        # Accelerometer Jacobian
        # ∂accel_x/∂bias_ax = 1
        H[0, 5] = 1.0
        # ∂accel_y/∂bias_ay = 1  
        H[1, 6] = 1.0
        
        # Gyroscope Jacobian
        # ∂gyro_z/∂bias_omega = 1
        H[2, 7] = 1.0
        
        # Additional terms for velocity-dependent yaw rate would go here
        # For now, simplified model
        
        return H
    
    def update_gps_position(self, gps_pos: np.ndarray):
        """
        Update with GPS position measurements
        
        Args:
            gps_pos: GPS position [x, y] in global frame (meters)
        """
        # Measurement vector
        z = gps_pos
        
        # Expected measurement (current position)
        h = self.x[0:2]
        
        # Measurement Jacobian
        H = np.zeros((2, self.n_states))
        H[0:2, 0:2] = np.eye(2)  # ∂pos/∂pos = I
        
        # Kalman update
        self._kalman_update(z, h, H, self.R_gps_pos)
        
        logger.debug(f"GPS position update: {gps_pos}")
    
    def update_gps_velocity(self, gps_vel: np.ndarray):
        """
        Update with GPS velocity measurements
        
        Args:
            gps_vel: GPS velocity [vx, vy] in global frame (m/s)
        """
        # Measurement vector
        z = gps_vel
        
        # Expected measurement (current velocity)
        h = self.x[3:5]
        
        # Measurement Jacobian
        H = np.zeros((2, self.n_states))
        H[0:2, 3:5] = np.eye(2)  # ∂vel/∂vel = I
        
        # Kalman update
        self._kalman_update(z, h, H, self.R_gps_vel)
        
        logger.debug(f"GPS velocity update: {gps_vel}")
    
    def _kalman_update(self, z: np.ndarray, h: np.ndarray, H: np.ndarray, R: np.ndarray):
        """
        Standard Kalman update following formulary equations
        """
        # Innovation
        y = z - h
        
        # Handle angle wrapping for orientation measurements
        if H.shape[0] == 1 and H[0, 2] != 0:  # Orientation measurement
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
        
        # Normalize angle
        self.x[2] = self._normalize_angle(self.x[2])
        
        # Update covariance (Joseph form for numerical stability)
        I_KH = np.eye(self.n_states) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T
        
        self.update_count += 1
    
    def get_state(self) -> RoboMasterState:
        """Get current state as RoboMasterState object"""
        return RoboMasterState.from_array(self.x)
    
    def get_covariance(self) -> np.ndarray:
        """Get current covariance matrix"""
        return self.P.copy()
    
    def get_position_uncertainty(self) -> np.ndarray:
        """Get position uncertainty (standard deviation)"""
        return np.sqrt(np.diag(self.P[0:2, 0:2]))
    
    def get_velocity_uncertainty(self) -> np.ndarray:
        """Get velocity uncertainty (standard deviation)"""
        return np.sqrt(np.diag(self.P[3:5, 3:5]))
    
    def get_bias_uncertainty(self) -> np.ndarray:
        """Get bias uncertainty (standard deviation)"""
        return np.sqrt(np.diag(self.P[5:8, 5:8]))
    
    def reset(self, initial_state: Optional[RoboMasterState] = None):
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
        self.P[0:2, 0:2] *= 1.0    # Position uncertainty
        self.P[2, 2] = 0.1         # Orientation uncertainty
        self.P[3:5, 3:5] *= 0.5    # Velocity uncertainty
        self.P[5:7, 5:7] *= 0.01   # Accel bias uncertainty
        self.P[7, 7] = 0.001       # Gyro bias uncertainty
        
        # Reset statistics
        self.update_count = 0
        self.prediction_count = 0
        
        logger.info("RoboMaster EKF reset")
    
    def _normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-π, π]"""
        return (angle + np.pi) % (2 * np.pi) - np.pi
    
    def get_statistics(self) -> Dict[str, Any]:
        """Get filter statistics"""
        return {
            'update_count': self.update_count,
            'prediction_count': self.prediction_count,
            'covariance_trace': np.trace(self.P),
            'position': self.x[0:2].tolist(),
            'velocity': self.x[3:5].tolist(),
            'orientation_deg': np.degrees(self.x[2]),
            'biases': self.x[5:8].tolist(),
            'position_uncertainty': self.get_position_uncertainty().tolist(),
            'velocity_uncertainty': self.get_velocity_uncertainty().tolist(),
            'bias_uncertainty': self.get_bias_uncertainty().tolist()
        }


# Example usage and testing
if __name__ == "__main__":
    # Create RoboMaster EKF with configuration
    config = {
        'q_position': 0.01,
        'q_theta': 0.01,
        'q_velocity': 0.1,
        'q_accel_bias': 1e-6,
        'q_gyro_bias': 1e-8,
        'r_accel': 0.1,
        'r_gyro': 0.01,
        'r_gps_pos': 1.0,
        'r_gps_vel': 0.5
    }
    
    ekf = RoboMasterEKF8DOF(config)
    
    # Simulate robot motion
    dt = 0.02  # 50 Hz
    
    for i in range(200):
        # Simulated control input [ax, ay, omega]
        control = np.array([1.0, 0.1, 0.05])  # Accelerating forward with slight turn
        
        # Prediction
        ekf.predict(dt, control)
        
        # Simulated IMU measurements
        accel_body = control[0:2] + np.random.randn(2) * 0.05  # Noisy acceleration
        gyro_z = control[2] + np.random.randn() * 0.01         # Noisy angular velocity
        
        ekf.update_imu(accel_body, gyro_z)
        
        # Simulated GPS updates every 10 steps
        if i % 10 == 0:
            # Simulated GPS position
            true_pos = np.array([i * 0.02, 0.5 * np.sin(i * 0.01)])
            gps_pos = true_pos + np.random.randn(2) * 0.1
            ekf.update_gps_position(gps_pos)
            
            # Simulated GPS velocity
            true_vel = np.array([1.0, 0.5 * 0.01 * np.cos(i * 0.01)])
            gps_vel = true_vel + np.random.randn(2) * 0.05
            ekf.update_gps_velocity(gps_vel)
        
        # Print state every 50 iterations
        if i % 50 == 0:
            state = ekf.get_state()
            print(f"Iteration {i}: {state}")
    
    # Final statistics
    stats = ekf.get_statistics()
    print(f"\nFinal statistics: {stats}")
