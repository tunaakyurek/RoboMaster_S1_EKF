# EKF Observability Fixes Summary
## Addressing Yaw Drift in RoboMaster 8-DOF EKF

**Date:** 2025-08-14  
**Issue:** Yaw (θ) was unobserved in the EKF, causing orientation drift and position errors  
**Root Cause:** Missing yaw measurements in the update step (H matrix had zeros in θ column)

---

## What Was Wrong

### 1. **IMU Double-Usage Problem**
- **Before:** IMU data was used both in prediction AND as measurement updates
- **Issue:** This violated the independence assumption and created fake observability
- **Result:** Biases "chased" the actual signal instead of being properly estimated

### 2. **Missing Yaw Observability**
- **Before:** Only GPS position/velocity updates were used (H matrix zeros in θ column)
- **Issue:** Kalman gain never corrected yaw, only x, y, vx, vy
- **Result:** Yaw drifted due to gyro bias, causing rotated accelerations to leak into velocity and position

### 3. **Incorrect Process Noise Structure**
- **Before:** Diagonal Q matrix assumed independent position/velocity
- **Issue:** For constant-acceleration kinematics, Q should have position-velocity coupling
- **Result:** Poor uncertainty propagation in the prediction step

---

## What Was Fixed

### 1. **Removed IMU Measurement Updates**
```python
def update_imu(self, accel_body: np.ndarray, gyro_z: float):
    """
    Deprecated: IMU should be used only in the prediction step, not as a measurement.
    Kept for backward compatibility but performs no update.
    """
    logger.debug("IMU measurement update is disabled to maintain independence from prediction inputs")
```

### 2. **Added Proper Discrete Process Noise Q**
```python
def _compute_discrete_process_noise(self, dt: float) -> np.ndarray:
    """
    Discretize continuous-time process noise (PSD) into discrete Qd for dt.
    - For each position-velocity axis: classic constant-acceleration model
    - For theta: driven by gyro noise PSD
    - For biases: random walk model
    """
    # X-axis [pos x (0), vel x (3)]
    Qd[0, 0] += (dt3 / 3.0) * q_a      # Position variance
    Qd[0, 3] += (dt2 / 2.0) * q_a      # Position-velocity covariance
    Qd[3, 0] += (dt2 / 2.0) * q_a      # Velocity-position covariance
    Qd[3, 3] += dt * q_a                # Velocity variance
```

### 3. **Added Yaw Measurement Updates**
```python
def update_yaw(self, yaw_measurement: float):
    """
    Update orientation (theta) using external yaw measurement.
    H = [0, 0, 1, 0, 0, 0, 0, 0]  # Only observes theta
    """
    z = np.array([yaw_measurement])
    h = np.array([self.x[2]])           # Current theta estimate
    H = np.zeros((1, self.n_states))
    H[0, 2] = 1.0                      # ∂h/∂theta = 1
    self._kalman_update(z, h, H, self.R_yaw)
```

### 4. **Added Non-Holonomic Constraint (NHC)**
```python
def update_non_holonomic_constraint(self, speed_threshold: float = 0.5):
    """
    Lateral body velocity should be ~0 for ground vehicles.
    Makes theta observable from vx, vy when moving forward.
    """
    # Lateral velocity: -sin(theta)*vx + cos(theta)*vy ≈ 0
    h = np.array([-np.sin(theta) * vx + np.cos(theta) * vy])
    
    # Jacobian: H = [0, 0, (-cos*vx - sin*vy), -sin, cos, 0, 0, 0]
    H[0, 2] = -np.cos(theta) * vx - np.sin(theta) * vy  # ∂h/∂theta
    H[0, 3] = -np.sin(theta)                            # ∂h/∂vx
    H[0, 4] = np.cos(theta)                             # ∂h/∂vy
```

### 5. **Added Zero-Velocity Updates (ZUPT)**
```python
def update_zero_velocity(self, speed_threshold: float = 0.1):
    """
    When speed is very low, measure vx=vy=0.
    Helps stabilize velocity and accelerometer biases.
    """
    if speed < speed_threshold:
        z = np.array([0.0, 0.0])        # [vx=0, vy=0]
        h = np.array([vx, vy])           # Current velocity estimate
        H = np.zeros((2, self.n_states))
        H[0, 3] = 1.0                   # ∂vx/∂vx
        H[1, 4] = 1.0                   # ∂vy/∂vy
```

### 6. **Added Zero-Angular-Rate Updates (ZARU)**
```python
def update_zero_angular_rate(self, angular_rate_threshold: float = 0.05):
    """
    When angular rate is very low, measure omega=0.
    Directly tightens gyroscope bias estimate.
    """
    if abs(omega_estimated) < angular_rate_threshold:
        z = np.array([0.0])             # omega = 0
        h = np.array([omega_estimated]) # Current omega estimate
        H = np.zeros((1, self.n_states))
        H[0, 7] = 1.0                   # ∂omega/∂bias_omega
```

### 7. **Added GPS Course-Based Yaw Updates**
```python
# GPS course-based yaw update when speed is sufficient (>0.7 m/s)
if speed > 0.7:  # Gate by speed to avoid jitter at low speeds
    # Convert GPS course to yaw (course is 0-360°, yaw is -π to π)
    yaw_course = course_rad - np.pi/2   # Adjust for coordinate system
    yaw_course = self._normalize_angle(yaw_course)
    
    # Update EKF with GPS-derived yaw
    self.ekf.update_yaw(yaw_course)
```

---

## Configuration Changes

### Process Noise (Increased for Better Observability)
```python
'q_gyro_bias': 1e-5,  # Increased from 1e-8 for better bias learning
```

### Measurement Noise (New Parameters)
```python
'r_yaw': 0.5,      # ~5° standard deviation for yaw updates
'r_nhc': 0.1,      # Non-holonomic constraint noise
'r_zupt': 0.01,    # Zero-velocity update noise
'r_zaru': 0.001,   # Zero-angular-rate update noise
```

### Initial Covariance (Increased for Better Observability)
```python
'init_gyro_bias_var': 0.01  # Increased from 0.001 for better observability
```

---

## How This Fixes the Problem

### 1. **Yaw Observability**
- **Before:** H matrix had zeros in θ column → no yaw corrections
- **After:** Yaw updates (magnetometer/GPS course) put 1 in θ column → yaw is observable

### 2. **Bias Learning**
- **Before:** Biases were poorly observable due to IMU double-usage
- **After:** Biases are learned through proper measurement updates (NHC, ZUPT, ZARU)

### 3. **Proper Uncertainty Propagation**
- **Before:** Diagonal Q assumed independent position/velocity
- **After:** Coupled position-velocity Q follows constant-acceleration model

### 4. **Robustness**
- **Before:** Only GPS position/velocity updates
- **After:** Multiple measurement sources (GPS, NHC, ZUPT, ZARU) provide redundancy

---

## Expected Results

### 1. **Eliminated Yaw Drift**
- Yaw will be corrected by magnetometer/GPS course updates
- Orientation uncertainty will remain bounded

### 2. **Better Bias Estimation**
- Accelerometer and gyroscope biases will converge faster
- Biases will be properly observable through constraint updates

### 3. **Improved Position Accuracy**
- Eliminated position drift caused by rotated accelerations
- Better velocity estimates through ZUPT updates

### 4. **Robust Performance**
- System works even when some sensors are unavailable
- Multiple measurement sources provide redundancy

---

## Usage

The fixes are automatically applied when you run the updated integration:

```python
# The EKF now automatically applies:
# - NHC updates when moving forward
# - ZUPT when stationary
# - ZARU when not rotating
# - GPS course yaw updates when moving

# You can also manually trigger updates if needed:
ekf.update_yaw(magnetometer_yaw)
ekf.update_non_holonomic_constraint()
ekf.update_zero_velocity()
ekf.update_zero_angular_rate()
```

---

## Testing

Run your existing RoboMaster integration as before. The system will now:
1. **Calibrate** sensors during stationary phase
2. **Apply constraints** automatically during operation
3. **Update yaw** from GPS course when available
4. **Maintain bounded uncertainty** in all state variables

Monitor the logs for:
- `"Yaw update with measurement=X.XXX rad"`
- `"NHC update applied: speed=X.XX m/s"`
- `"ZUPT applied: speed=X.XXX m/s"`
- `"ZARU applied: estimated_omega=X.XXXX rad/s"`

These indicate the new measurement updates are working correctly.
