"""
Extended Kalman Filter implementation for RoboMaster S1 state estimation.
Based solely on sensor measurements: IMU, GPS, Barometer, Magnetometer
"""

import numpy as np
from dataclasses import dataclass
from typing import Optional, Tuple
import time

@dataclass
class SensorData:
    """Container for all sensor measurements"""
    timestamp: float
    # IMU data
    accel: np.ndarray  # [ax, ay, az] in m/s²
    gyro: np.ndarray   # [wx, wy, wz] in rad/s
    # Magnetometer
    mag: np.ndarray    # [mx, my, mz] in μT
    # Barometer (if available)
    pressure: Optional[float] = None  # Pa
    altitude: Optional[float] = None  # m
    # GPS (if available)
    gps_lat: Optional[float] = None   # degrees
    gps_lon: Optional[float] = None   # degrees
    gps_alt: Optional[float] = None   # m
    # Chassis position from RoboMaster (relative)
    chassis_x: Optional[float] = None  # m
    chassis_y: Optional[float] = None  # m
    chassis_yaw: Optional[float] = None  # rad

@dataclass
class EKFState:
    """EKF state vector: [x, y, z, vx, vy, vz, roll, pitch, yaw, wx, wy, wz]"""
    position: np.ndarray     # [x, y, z] in meters
    velocity: np.ndarray     # [vx, vy, vz] in m/s
    orientation: np.ndarray  # [roll, pitch, yaw] in radians
    angular_velocity: np.ndarray  # [wx, wy, wz] in rad/s
    
    def to_vector(self) -> np.ndarray:
        """Convert state to 12D vector"""
        return np.concatenate([
            self.position, 
            self.velocity, 
            self.orientation, 
            self.angular_velocity
        ])
    
    @classmethod
    def from_vector(cls, vec: np.ndarray) -> 'EKFState':
        """Create state from 12D vector"""
        return cls(
            position=vec[0:3],
            velocity=vec[3:6],
            orientation=vec[6:9],
            angular_velocity=vec[9:12]
        )

class ExtendedKalmanFilter:
    """
    Extended Kalman Filter for drone state estimation
    State: [x, y, z, vx, vy, vz, roll, pitch, yaw, wx, wy, wz] (12D)
    """
    
    def __init__(self):
        # State dimension
        self.n_states = 12
        
        # Initialize state and covariance
        self.x = np.zeros(self.n_states)  # State vector
        self.P = np.eye(self.n_states) * 1.0  # Covariance matrix
        
        # Process noise covariance
        self.Q = self._create_process_noise_matrix()
        
        # Measurement noise covariances
        self.R_imu = np.eye(6) * 0.1  # IMU: [ax, ay, az, wx, wy, wz]
        self.R_mag = np.eye(3) * 0.5  # Magnetometer: [mx, my, mz]
        self.R_baro = np.array([[0.1]])  # Barometer: [altitude]
        self.R_gps = np.eye(3) * 1.0   # GPS: [x, y, z]
        self.R_chassis = np.eye(3) * 0.05  # Chassis: [x, y, yaw]
        
        # Reference values
        self.gravity = 9.81
        self.mag_declination = 0.0  # Magnetic declination (to be calibrated)
        
        # Time tracking
        self.last_time = None
        
        # Reference position (for GPS conversion)
        self.ref_lat = None
        self.ref_lon = None
        self.ref_alt = None
    
    def _create_process_noise_matrix(self) -> np.ndarray:
        """Create process noise covariance matrix Q"""
        Q = np.zeros((self.n_states, self.n_states))
        
        # Position noise (low, as it's integrated from velocity)
        Q[0:3, 0:3] = np.eye(3) * 0.01
        
        # Velocity noise (moderate)
        Q[3:6, 3:6] = np.eye(3) * 0.1
        
        # Orientation noise (low to moderate)
        Q[6:9, 6:9] = np.eye(3) * 0.05
        
        # Angular velocity noise (moderate)
        Q[9:12, 9:12] = np.eye(3) * 0.1
        
        return Q
    
    def predict(self, dt: float):
        """Prediction step of EKF"""
        if dt <= 0:
            return
            
        # State transition model (kinematic model)
        F = self._compute_state_transition_matrix(dt)
        
        # Predict state
        self.x = F @ self.x
        
        # Predict covariance
        self.P = F @ self.P @ F.T + self.Q * dt
        
        # Normalize angles
        self.x[6:9] = self._normalize_angles(self.x[6:9])
    
    def _compute_state_transition_matrix(self, dt: float) -> np.ndarray:
        """Compute state transition matrix F"""
        F = np.eye(self.n_states)
        
        # Position updates from velocity
        F[0:3, 3:6] = np.eye(3) * dt
        
        # Orientation updates from angular velocity
        F[6:9, 9:12] = np.eye(3) * dt
        
        return F
    
    def _normalize_angles(self, angles: np.ndarray) -> np.ndarray:
        """Normalize angles to [-π, π]"""
        return (angles + np.pi) % (2 * np.pi) - np.pi
    
    def update_imu(self, accel: np.ndarray, gyro: np.ndarray):
        """Update with IMU measurements"""
        # Expected IMU measurements based on current state
        h_imu = self._compute_expected_imu(self.x)
        
        # Jacobian matrix for IMU
        H_imu = self._compute_imu_jacobian(self.x)
        
        # Measurement vector
        z_imu = np.concatenate([accel, gyro])
        
        # Perform update
        self._kalman_update(z_imu, h_imu, H_imu, self.R_imu)
    
    def _compute_expected_imu(self, state: np.ndarray) -> np.ndarray:
        """Compute expected IMU measurements"""
        roll, pitch, yaw = state[6:9]
        wx, wy, wz = state[9:12]
        
        # Expected acceleration (gravity in body frame + motion)
        # Gravity vector in body frame
        g_body = np.array([
            -self.gravity * np.sin(pitch),
            self.gravity * np.sin(roll) * np.cos(pitch),
            self.gravity * np.cos(roll) * np.cos(pitch)
        ])
        
        # Expected gyroscope (direct measurement of angular velocity)
        expected_gyro = np.array([wx, wy, wz])
        
        return np.concatenate([g_body, expected_gyro])
    
    def _compute_imu_jacobian(self, state: np.ndarray) -> np.ndarray:
        """Compute Jacobian matrix for IMU measurements"""
        roll, pitch, yaw = state[6:9]
        
        H = np.zeros((6, self.n_states))
        
        # Accelerometer Jacobian (w.r.t. roll and pitch)
        H[0, 7] = -self.gravity * np.cos(pitch)  # da_x/d_pitch
        H[1, 6] = self.gravity * np.cos(roll) * np.cos(pitch)   # da_y/d_roll
        H[1, 7] = -self.gravity * np.sin(roll) * np.sin(pitch)  # da_y/d_pitch
        H[2, 6] = -self.gravity * np.sin(roll) * np.cos(pitch)  # da_z/d_roll
        H[2, 7] = -self.gravity * np.cos(roll) * np.sin(pitch)  # da_z/d_pitch
        
        # Gyroscope Jacobian (direct measurement)
        H[3:6, 9:12] = np.eye(3)
        
        return H
    
    def update_magnetometer(self, mag: np.ndarray):
        """Update with magnetometer measurements"""
        # Normalize magnetometer reading
        mag_norm = mag / np.linalg.norm(mag)
        
        # Expected magnetometer reading
        h_mag = self._compute_expected_magnetometer(self.x)
        
        # Jacobian for magnetometer
        H_mag = self._compute_magnetometer_jacobian(self.x)
        
        # Perform update
        self._kalman_update(mag_norm, h_mag, H_mag, self.R_mag)
    
    def _compute_expected_magnetometer(self, state: np.ndarray) -> np.ndarray:
        """Compute expected magnetometer measurements"""
        roll, pitch, yaw = state[6:9]
        
        # Magnetic field vector in NED frame (assuming north-aligned)
        mag_ned = np.array([np.cos(self.mag_declination), 
                           np.sin(self.mag_declination), 
                           0.0])
        
        # Rotation matrix from NED to body frame
        R = self._rotation_matrix_ned_to_body(roll, pitch, yaw)
        
        # Expected magnetometer reading in body frame
        mag_body = R @ mag_ned
        
        return mag_body / np.linalg.norm(mag_body)
    
    def _rotation_matrix_ned_to_body(self, roll: float, pitch: float, yaw: float) -> np.ndarray:
        """Compute rotation matrix from NED to body frame"""
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)
        
        R = np.array([
            [cp*cy, cp*sy, -sp],
            [sr*sp*cy - cr*sy, sr*sp*sy + cr*cy, sr*cp],
            [cr*sp*cy + sr*sy, cr*sp*sy - sr*cy, cr*cp]
        ])
        
        return R
    
    def _compute_magnetometer_jacobian(self, state: np.ndarray) -> np.ndarray:
        """Compute Jacobian for magnetometer measurements"""
        # This is a simplified version - full implementation would include
        # derivatives of the rotation matrix w.r.t. roll, pitch, yaw
        H = np.zeros((3, self.n_states))
        
        # For now, assume linear relationship with orientation
        H[0:3, 6:9] = np.eye(3) * 0.1
        
        return H
    
    def update_gps(self, lat: float, lon: float, alt: float):
        """Update with GPS measurements"""
        if self.ref_lat is None:
            # Set reference position on first GPS fix
            self.ref_lat = lat
            self.ref_lon = lon
            self.ref_alt = alt
            return
        
        # Convert GPS to local coordinates
        x, y, z = self._gps_to_local(lat, lon, alt)
        
        # GPS measurement vector
        z_gps = np.array([x, y, z])
        
        # Expected GPS measurement (just position)
        h_gps = self.x[0:3]
        
        # Jacobian (direct measurement of position)
        H_gps = np.zeros((3, self.n_states))
        H_gps[0:3, 0:3] = np.eye(3)
        
        # Perform update
        self._kalman_update(z_gps, h_gps, H_gps, self.R_gps)
    
    def _gps_to_local(self, lat: float, lon: float, alt: float) -> Tuple[float, float, float]:
        """Convert GPS coordinates to local coordinates"""
        # Simplified conversion (assumes small distances)
        R_earth = 6371000  # Earth radius in meters
        
        dlat = np.radians(lat - self.ref_lat)
        dlon = np.radians(lon - self.ref_lon)
        
        x = R_earth * dlon * np.cos(np.radians(self.ref_lat))
        y = R_earth * dlat
        z = alt - self.ref_alt
        
        return x, y, z
    
    def update_chassis(self, x: float, y: float, yaw: float):
        """Update with chassis encoder measurements"""
        z_chassis = np.array([x, y, yaw])
        
        # Expected chassis measurement
        h_chassis = np.array([self.x[0], self.x[1], self.x[8]])
        
        # Jacobian
        H_chassis = np.zeros((3, self.n_states))
        H_chassis[0, 0] = 1  # x position
        H_chassis[1, 1] = 1  # y position
        H_chassis[2, 8] = 1  # yaw angle
        
        # Perform update
        self._kalman_update(z_chassis, h_chassis, H_chassis, self.R_chassis)
    
    def _kalman_update(self, z: np.ndarray, h: np.ndarray, H: np.ndarray, R: np.ndarray):
        """Perform Kalman update step"""
        # Innovation
        y = z - h
        
        # Innovation covariance
        S = H @ self.P @ H.T + R
        
        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # Update state
        self.x = self.x + K @ y
        
        # Update covariance
        I_KH = np.eye(self.n_states) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T
        
        # Normalize angles
        self.x[6:9] = self._normalize_angles(self.x[6:9])
    
    def get_state(self) -> EKFState:
        """Get current state as EKFState object"""
        return EKFState.from_vector(self.x)
    
    def process_sensor_data(self, sensor_data: SensorData):
        """Process all available sensor data"""
        # Compute time step
        current_time = sensor_data.timestamp
        if self.last_time is not None:
            dt = current_time - self.last_time
            if dt > 0:
                self.predict(dt)
        self.last_time = current_time
        
        # Update with available sensors
        if sensor_data.accel is not None and sensor_data.gyro is not None:
            self.update_imu(sensor_data.accel, sensor_data.gyro)
        
        if sensor_data.mag is not None:
            self.update_magnetometer(sensor_data.mag)
        
        if all(x is not None for x in [sensor_data.gps_lat, sensor_data.gps_lon, sensor_data.gps_alt]):
            self.update_gps(sensor_data.gps_lat, sensor_data.gps_lon, sensor_data.gps_alt)
        
        if all(x is not None for x in [sensor_data.chassis_x, sensor_data.chassis_y, sensor_data.chassis_yaw]):
            self.update_chassis(sensor_data.chassis_x, sensor_data.chassis_y, sensor_data.chassis_yaw)