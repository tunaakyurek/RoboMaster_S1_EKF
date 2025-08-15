# Yaw Observability Fixes for RoboMaster EKF

## Problem Summary

The original RoboMaster EKF suffered from **yaw (θ) unobservability**, causing:
- **Yaw drift**: θ drifted ~53° in ~25 seconds
- **Position drift**: Incorrect yaw caused misaligned acceleration integration
- **Poor bias estimation**: Gyro bias couldn't be properly learned without yaw measurements

## Root Cause Analysis

The EKF measurement model **lacked yaw observability**:

1. **State vector**: `[x, y, θ, vx, vy, bias_accel_x, bias_accel_y, bias_angular_velocity]`
2. **Prediction**: θ updated via gyro integration only: `θ_{k+1} = θ_k + (ω_z - bias_ω) * Δt`
3. **Updates**: GPS position/velocity updates only affected `[x, y]` and `[vx, vy]`
4. **Missing**: No measurement `h(x)` that directly observed θ
5. **Result**: θ column of measurement Jacobian H was always zero → no yaw correction

## Implemented Fixes

### 1. GPS Course-Derived Yaw Updates

**Method**: `update_gps_course_yaw(speed_threshold=0.7)`

```python
# Calculate course angle from GPS velocity
vx_gps, vy_gps = gps_velocity
yaw_course = np.arctan2(vy_gps, vx_gps)

# Update yaw with GPS course measurement
self.update_yaw(yaw_course)
```

**Why it works**: 
- Provides **absolute heading** when robot is moving
- Measurement model: `h_ψ(x) = θ` with Jacobian `H_ψ = [0 0 1 0 0 0 0 0]`
- The `1` in the θ column enables yaw correction in the Kalman update

**When to use**: Speed > 0.7 m/s for reliable course estimation

### 2. Magnetometer Yaw Updates

**Method**: `update_magnetometer_yaw(mag_vector)`

```python
# Calculate yaw from magnetometer (assuming level phone)
mx, my = mag_vector[0], mag_vector[1]
yaw_mag = np.arctan2(-my, mx)
self.update_yaw(yaw_mag)
```

**Why it works**:
- Provides **absolute heading** independent of motion
- Works even when stationary
- Direct measurement of θ with proper Jacobian

**Note**: Requires magnetometer calibration for hard/soft iron effects

### 3. Improved Non-Holonomic Constraint (NHC)

**Method**: `update_non_holonomic_constraint()`

```python
# Lateral velocity in body frame should be ~0 for ground vehicles
h_nhc(x) = -sin(θ) * vx + cos(θ) * vy ≈ 0

# Jacobian: H = [0, 0, (-cos(θ)*vx - sin(θ)*vy), -sin(θ), cos(θ), 0, 0, 0]
```

**Why it works**:
- Makes θ observable from `[vx, vy]` when moving forward
- The θ derivative `∂h/∂θ` provides observability
- Only applied when θ is meaningfully observable

### 4. Zero-Velocity and Zero-Angular-Rate Updates

**Methods**: `update_zero_velocity()`, `update_zero_angular_rate()`

```python
# ZUPT: When speed ≈ 0, measure vx = vy = 0
# ZARU: When |ω_z| ≈ 0, measure ω_z = 0
```

**Why they work**:
- **ZUPT**: Stabilizes velocity and helps accelerometer bias estimation
- **ZARU**: Directly tightens gyroscope bias estimate
- Standard pseudo-measurements for INS/GNSS systems

### 5. Improved Gyro Bias Process Noise

**Configuration change**:
```python
# Before: q_gyro_bias = 1e-8 (too confident)
# After:  q_gyro_bias = 1e-5 (allows bias learning)
```

**Why it works**:
- Allows the filter to **learn and correct** gyro bias
- With yaw measurements, the filter can now properly estimate bias
- Prevents overconfidence in bias estimates

## Integration Changes

### Main Integration Loop

```python
# After GPS updates
self.ekf.apply_constraint_updates()

# In EKF processing loop
self.ekf.apply_constraint_updates()
```

### GPS Update Handler

```python
def _handle_gps_update(self, gps_data):
    # Update position and velocity
    self.ekf.update_gps_position(gps_pos)
    self.ekf.update_gps_velocity(gps_vel)
    
    # Apply all constraint updates
    self.ekf.apply_constraint_updates()
```

## Expected Results

### Before Fixes
- Yaw drift: ~53° in 25 seconds
- Position drift due to misaligned acceleration
- Poor gyro bias estimation
- Unbounded yaw uncertainty

### After Fixes
- **Yaw drift eliminated** with GPS course updates
- **Stable position estimation** with correct orientation
- **Accurate bias estimation** with proper observability
- **Bounded yaw uncertainty** in covariance

## Testing

Run the test script to verify improvements:

```bash
python iphone_integration/test_improved_ekf.py
```

This will:
1. Simulate realistic robot motion
2. Apply all constraint updates
3. Compare true vs. estimated states
4. Generate performance plots

## Configuration Recommendations

### Process Noise (Q)
```python
config = {
    'q_accel': 0.5,           # White acceleration PSD
    'q_gyro': 0.01,           # White gyro noise PSD  
    'q_accel_bias': 1e-6,     # Accel bias random walk
    'q_gyro_bias': 1e-5,      # Gyro bias random walk (increased!)
}
```

### Measurement Noise (R)
```python
config = {
    'r_gps_pos': 1.0,         # GPS position noise
    'r_gps_vel': 0.5,         # GPS velocity noise
    'r_yaw': 0.5,             # Yaw measurement noise
    'r_nhc': 0.1,             # NHC constraint noise
    'r_zupt': 0.01,           # Zero-velocity noise
    'r_zaru': 0.001,          # Zero-angular-rate noise
}
```

## Best Practices

1. **Always apply constraint updates** after GPS updates
2. **Gate GPS course updates** by speed (>0.7 m/s)
3. **Use magnetometer** when available for stationary operation
4. **Monitor covariance** to ensure proper observability
5. **Tune noise parameters** based on sensor characteristics

## Conclusion

The yaw observability issue was solved by adding **absolute heading measurements** (GPS course, magnetometer) and **constraint-based pseudo-measurements** (NHC, ZUPT, ZARU). This brings the EKF back in line with proper observability requirements and eliminates the drift issues.

The fixes follow standard EKF practices and are commonly used in INS/GNSS systems for ground vehicles.
