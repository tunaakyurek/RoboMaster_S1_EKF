# RoboMaster S1 Lab Environment Fix Guide

## 🚨 Problem Analysis

**Original Error:**
```
Failed Error Line Number: 173
Error. Info: Traceback (most recent call last)? 
File "«CurFile›", line 17, in «CurModule> 
RuntimeError: invalid module, the module is numpy
```

**Root Cause:**
The original `autonomous_controller.py` uses `numpy` (line 17: `import numpy as np`), but the **RoboMaster S1 Lab environment** doesn't support external Python packages like numpy. The S1 runs in a restricted sandbox with only built-in Python libraries and RoboMaster-specific APIs.

## ✅ Solution: Lab-Compatible Version

### Files Created:
1. **`autonomous_controller_lab_fixed.py`** - Full autonomous controller without numpy
2. **`simple_autonomous_test.py`** - Minimal test version for easy copy-paste

## 🔧 Key Changes Made

### 1. Removed External Dependencies
```python
# ❌ Original (doesn't work in Lab)
import numpy as np
import logging
import threading
import queue

# ✅ Fixed (Lab compatible)
import math
import time
from enum import Enum
# No external imports - use built-in APIs only
```

### 2. Replaced numpy Functions
```python
# ❌ Original numpy usage
distance = np.sqrt(error_x**2 + error_y**2)
cos_yaw = np.cos(current_yaw)
output = np.clip(output, min_val, max_val)

# ✅ Pure Python replacement
distance = math.sqrt(error_x**2 + error_y**2)
cos_yaw = math.cos(current_yaw)
output = max(min_val, min(output, max_val))
```

### 3. Used RoboMaster Built-in APIs
```python
# ❌ External SDK syntax (for PC control)
import robot
ep_robot = robot.Robot()

# ✅ Lab environment APIs (built-in)
robot_ctrl.set_mode(rm_define.robot_mode_chassis_follow)
gyro_data = sensor_imu.get_gyroscope()
chassis_ctrl.set_trans_speed(speed_mm_per_sec)
```

### 4. Memory Optimization for S1
- **Removed threading** (high memory usage)
- **Simplified data structures** (no complex queues)
- **Reduced control frequency** (from 20Hz to 5-10Hz)
- **Manual update cycles** instead of continuous loops

## 🚀 How to Use

### Option 1: Copy Simple Test Version

1. **Open RoboMaster app** and connect to S1
2. **Go to Lab > Python**
3. **Copy entire content** of `simple_autonomous_test.py`
4. **Paste into Lab editor**
5. **Run** and observe the demo

### Option 2: Use Full Controller

1. **Copy content** of `autonomous_controller_lab_fixed.py`
2. **Paste into Lab editor**
3. **Uncomment the last line**: `run_autonomous_demo()`
4. **Run** the script

## 🎮 Available Demos

### Basic Movement Demo
```python
run_basic_movement_demo()
```
- Forward movement (3 seconds)
- Rotation left (2 seconds)
- Backward movement (2 seconds)
- Rotation right (2 seconds)

### Sensor Test
```python
run_sensor_test()
```
- Reads gyroscope data
- Reads accelerometer data
- Reads attitude data
- 10 readings with 1-second intervals

### Waypoint Navigation
```python
run_waypoint_demo()
```
- Navigate to (1,0) → (1,1) → (0,1) → (0,0)
- Basic odometry estimation
- PID-based navigation

### Emergency Stop
```python
emergency_stop()
```
- Immediate stop all movement
- Use if robot behaves unexpectedly

## ⚠️ Safety Features

1. **Conservative speeds**: Max 300-400 mm/s (vs normal 3000+ mm/s)
2. **Timeout protection**: All demos stop after 20-30 seconds
3. **Emergency stop**: Easy to trigger movement stop
4. **Error handling**: Graceful degradation on sensor failures

## 🔧 Configuration Options

### Speed Limits (adjust for your environment)
```python
config = {
    'max_velocity': 0.5,        # m/s (conservative)
    'max_yaw_rate': math.pi/4,  # rad/s (45 deg/s)
    'control_rate': 5,          # Hz (reduced for S1)
}
```

### PID Tuning
```python
config = {
    'position_kp': 0.8,   # Proportional gain
    'position_ki': 0.05,  # Integral gain  
    'position_kd': 0.2,   # Derivative gain
}
```

## 🎯 Performance Expectations

### RoboMaster S1 Limitations:
- **CPU**: ARM Cortex-A7 (~26 BogoMIPS per core)
- **Memory**: 272MB total RAM
- **Python**: Limited sandbox environment
- **Sensors**: IMU, camera (no direct position sensing)

### Expected Performance:
- **Control frequency**: 5-10 Hz (vs 20+ Hz with numpy)
- **Position accuracy**: ±0.2-0.5m (odometry-based)
- **Response time**: 100-200ms command latency
- **Maximum runtime**: 20-30 seconds per session

## 🐛 Troubleshooting

### Common Issues:

1. **"Invalid module" error**
   - ✅ Make sure no external imports (numpy, scipy, etc.)
   - ✅ Use only built-in Python + RoboMaster APIs

2. **Robot doesn't move**
   - ✅ Check robot mode: `robot_ctrl.set_mode(rm_define.robot_mode_chassis_follow)`
   - ✅ Verify speeds are within limits (300-1000 mm/s)

3. **Sensor data returns None/zeros**
   - ✅ Add try-catch around sensor calls
   - ✅ Use fallback values for missing sensors

4. **Code runs too slowly**
   - ✅ Reduce control frequency (5Hz instead of 20Hz)
   - ✅ Simplify calculations (avoid complex loops)

### Debug Tips:
```python
# Add debug prints
print(f"Current state: {controller.current_state}")
print(f"Command: vx={command.vx:.2f}, yaw_rate={command.yaw_rate:.2f}")

# Check sensor availability
try:
    gyro = sensor_imu.get_gyroscope()
    print(f"Gyro working: {gyro}")
except Exception as e:
    print(f"Gyro error: {e}")
```

## 📈 Next Steps

1. **Test simple movement** first
2. **Gradually increase complexity** (waypoints, etc.)
3. **Tune PID parameters** for your environment
4. **Add obstacle avoidance** using camera/distance sensors
5. **Integrate with iPhone sensor data** (if using external connection)

## 🔗 Related Files

- **Original file**: `autonomous_controller.py` (numpy-dependent)
- **Fixed version**: `autonomous_controller_lab_fixed.py` (Lab compatible)
- **Simple test**: `simple_autonomous_test.py` (minimal demo)
- **API reference**: See `Only_Experiment_Code/API_FIX_SUMMARY.md`

---

**Summary**: The original code failed because it tried to import numpy, which isn't available in the RoboMaster S1 Lab environment. The fixed versions use pure Python math and RoboMaster's built-in APIs for autonomous control.
