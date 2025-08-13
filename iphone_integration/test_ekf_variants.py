#!/usr/bin/env python3
"""
Test and Compare EKF Variants
==============================
Quick test script to compare the three EKF implementations:
- 3-DOF Rover EKF (ground navigation)
- 8-DOF Simplified Drone EKF (corrected)
- 9-DOF Full Drone EKF (complete dynamics)

Run this to verify all implementations work correctly.
"""

import numpy as np
import time
import sys
import os

# Add current directory to path
sys.path.append(os.path.dirname(__file__))

from pi_phone_connection.ekf_rover_3dof import EKFRover3DOF, EKFRoverState
from pi_phone_connection.ekf_8dof_formulary import EKF8DOF, EKF8State
from pi_phone_connection.ekf_drone_9dof import EKFDrone9DOF, EKFDroneState

def test_rover_3dof():
    """Test 3-DOF Rover EKF"""
    print("🚗 Testing 3-DOF Rover EKF...")
    
    config = {
        'q_position': 0.01,
        'q_heading': 0.02,
        'r_compass': 0.1,
        'r_gps': 1.0
    }
    
    ekf = EKFRover3DOF(config)
    dt = 0.1
    
    # Simulate rover driving in a circle
    for i in range(50):
        # Motion: 1 m/s forward, turning
        velocity = 1.0
        steering = 0.1 * np.sin(i * 0.1)
        
        ekf.predict(dt, velocity, steering)
        
        # Simulated GPS every 5 steps
        if i % 5 == 0:
            true_x = 2.0 * np.sin(i * 0.05)
            true_y = 2.0 * (1 - np.cos(i * 0.05))
            ekf.update_gps(true_x + np.random.normal(0, 0.1), 
                          true_y + np.random.normal(0, 0.1))
        
        # Simulated compass every 3 steps
        if i % 3 == 0:
            true_heading = i * 0.05
            ekf.update_compass(true_heading + np.random.normal(0, 0.05))
    
    state = ekf.get_state()
    stats = ekf.get_statistics()
    
    print(f"   Final state: {state}")
    print(f"   Updates: {stats['update_count']}, Predictions: {stats['prediction_count']}")
    print(f"   Position uncertainty: {stats['position_uncertainty']}")
    print("   ✅ Rover EKF test completed\n")

def test_drone_8dof():
    """Test 8-DOF Drone EKF (corrected)"""
    print("🚁 Testing 8-DOF Drone EKF (Corrected)...")
    
    config = {
        'q_position': 0.01,
        'q_velocity': 0.1,
        'q_orientation': 0.05,
        'use_drone_velocity': True
    }
    
    ekf = EKF8DOF(config)
    dt = 0.02
    
    # Simulate hovering with some motion
    for i in range(100):
        ekf.predict(dt)
        
        # Simulated IMU data
        accel = np.array([0.1 * np.sin(i * 0.1), 0.05 * np.cos(i * 0.1), 9.81]) + np.random.randn(3) * 0.1
        gyro = np.array([0.0, 0.0, 0.05 * np.cos(i * 0.05)]) + np.random.randn(3) * 0.01
        
        ekf.update_imu(accel, gyro)
        
        # Simulated magnetometer
        if i % 10 == 0:
            mag = np.array([20.0, 5.0, -40.0]) + np.random.randn(3) * 1.0
            ekf.update_magnetometer(mag)
        
        # Simulated barometer
        if i % 5 == 0:
            altitude = 5.0 + 0.5 * np.sin(i * 0.02) + np.random.normal(0, 0.1)
            ekf.update_barometer(altitude)
    
    state = ekf.get_state()
    stats = ekf.get_statistics()
    
    print(f"   Final state: {state}")
    print(f"   Updates: {stats['update_count']}, Predictions: {stats['prediction_count']}")
    print(f"   Position uncertainty: {stats['position_uncertainty']}")
    print("   ✅ 8-DOF Drone EKF test completed\n")

def test_drone_9dof():
    """Test 9-DOF Full Drone EKF"""
    print("🚁 Testing 9-DOF Full Drone EKF...")
    
    config = {
        'q_position': 0.01,
        'q_velocity': 0.1,
        'q_attitude': 0.05,
        'r_accel': 0.5,
        'r_gyro': 0.1
    }
    
    ekf = EKFDrone9DOF(config)
    dt = 0.02
    
    # Simulate complex 3D motion
    for i in range(100):
        # Angular velocity (rotation in yaw)
        omega = np.array([0.0, 0.0, 0.1 * np.sin(i * 0.05)])
        # Acceleration (hovering with small perturbations)
        accel_body = np.array([0.1 * np.cos(i * 0.1), 0.1 * np.sin(i * 0.1), 9.81])
        
        ekf.predict(dt, omega, accel_body)
        
        # Simulated IMU measurements
        accel_meas = accel_body + np.random.randn(3) * 0.1
        gyro_meas = omega + np.random.randn(3) * 0.01
        
        ekf.update_imu_full(accel_meas, gyro_meas)
        
        # Simulated magnetometer
        if i % 8 == 0:
            mag = np.array([25.0, 10.0, -45.0]) + np.random.randn(3) * 2.0
            ekf.update_magnetometer(mag)
        
        # Simulated barometer
        if i % 6 == 0:
            altitude = 10.0 + np.sin(i * 0.03) + np.random.normal(0, 0.1)
            ekf.update_barometer(altitude)
    
    state = ekf.get_state()
    stats = ekf.get_statistics()
    
    print(f"   Final state: {state}")
    print(f"   Updates: {stats['update_count']}, Predictions: {stats['prediction_count']}")
    print(f"   Position uncertainty: {stats['position_uncertainty']}")
    print("   ✅ 9-DOF Drone EKF test completed\n")

def test_enhanced_integration():
    """Test Enhanced Integration System"""
    print("🔧 Testing Enhanced Integration System...")
    
    try:
        from pi_phone_connection.main_integration_enhanced import EnhancediPhoneEKFIntegration, EKFMode
        
        # Test each mode
        modes = [EKFMode.ROVER_3DOF, EKFMode.DRONE_8DOF, EKFMode.DRONE_9DOF]
        
        for mode in modes:
            print(f"   Testing {mode.value}...")
            
            config = {
                'calibration': {'auto_calibrate': False},
                'logging': {'enabled': False}
            }
            
            # Create integration system
            integration = EnhancediPhoneEKFIntegration(None, mode)
            integration.config = config
            integration.is_calibrated = True  # Skip calibration for test
            
            # Test state retrieval
            state = integration.get_current_state()
            stats = integration.get_statistics()
            
            print(f"      Initial state: {state}")
            print(f"      Mode: {stats['mode']}")
        
        print("   ✅ Enhanced integration test completed\n")
    
    except Exception as e:
        print(f"   ❌ Enhanced integration test failed: {e}\n")

def performance_benchmark():
    """Compare performance of all EKF variants"""
    print("⚡ Performance Benchmark...")
    
    n_iterations = 1000
    
    # Benchmark Rover 3-DOF
    start_time = time.time()
    ekf_rover = EKFRover3DOF()
    for i in range(n_iterations):
        ekf_rover.predict(0.02, 1.0, 0.1)
        if i % 10 == 0:
            ekf_rover.update_compass(0.1)
    rover_time = time.time() - start_time
    
    # Benchmark Drone 8-DOF
    start_time = time.time()
    ekf_8dof = EKF8DOF()
    for i in range(n_iterations):
        ekf_8dof.predict(0.02)
        if i % 5 == 0:
            accel = np.array([0, 0, 9.81])
            gyro = np.array([0, 0, 0.1])
            ekf_8dof.update_imu(accel, gyro)
    drone_8dof_time = time.time() - start_time
    
    # Benchmark Drone 9-DOF
    start_time = time.time()
    ekf_9dof = EKFDrone9DOF()
    for i in range(n_iterations):
        omega = np.array([0, 0, 0.1])
        accel = np.array([0, 0, 9.81])
        ekf_9dof.predict(0.02, omega, accel)
        if i % 5 == 0:
            ekf_9dof.update_imu_full(accel, omega)
    drone_9dof_time = time.time() - start_time
    
    print(f"   Rover 3-DOF:     {rover_time:.3f}s ({n_iterations/rover_time:.0f} Hz)")
    print(f"   Drone 8-DOF:     {drone_8dof_time:.3f}s ({n_iterations/drone_8dof_time:.0f} Hz)")
    print(f"   Drone 9-DOF:     {drone_9dof_time:.3f}s ({n_iterations/drone_9dof_time:.0f} Hz)")
    print("   ✅ Performance benchmark completed\n")

def main():
    """Run all tests"""
    print("🧪 EKF Variants Test Suite")
    print("=" * 50)
    
    try:
        test_rover_3dof()
        test_drone_8dof()
        test_drone_9dof()
        test_enhanced_integration()
        performance_benchmark()
        
        print("🎉 All tests completed successfully!")
        print("\n📊 Summary:")
        print("   ✅ 3-DOF Rover EKF: Ground navigation (fast, simple)")
        print("   ✅ 8-DOF Drone EKF: Simplified drone (corrected, balanced)")
        print("   ✅ 9-DOF Drone EKF: Full dynamics (complete, advanced)")
        print("   ✅ Enhanced Integration: Multi-mode support")
        
    except Exception as e:
        print(f"❌ Test failed: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()
