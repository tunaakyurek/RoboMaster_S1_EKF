#!/usr/bin/env python3
"""
RoboMaster EKF Comprehensive Test Suite
========================================
Consolidated testing for coordinate transformation fix and drift resistance
"""

import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime
import time

# Import our modules
try:
    from pi_phone_connection.ekf_robomaster_8dof import RoboMasterEKF8DOF, RoboMasterState
except ImportError:
    from ekf_robomaster_8dof import RoboMasterEKF8DOF, RoboMasterState

def test_coordinate_transformation_fix():
    """Test that the coordinate transformation fix resolves Y-direction spikes"""
    
    print("🧪 Testing Coordinate Transformation Fix")
    print("=" * 45)
    
    # Configuration with conservative drift resistance
    config = {
        'q_accel': 0.2,        # Conservative reduction
        'q_gyro': 0.008,       # Conservative reduction
        'q_accel_bias': 5e-7,  # Conservative reduction
        'q_gyro_bias': 5e-6,   # Conservative reduction
        'r_zupt': 0.01,
        'r_zaru': 0.001
    }
    
    # Initialize EKF with fixed coordinate transformation
    ekf = RoboMasterEKF8DOF(config)
    
    # Simulate stationary operation with small sensor noise
    dt = 0.02  # 50 Hz
    duration = 30.0  # 30 seconds to catch any late spikes
    n_steps = int(duration / dt)
    
    # Storage for analysis
    times = []
    positions = []
    velocities = []
    orientations = []
    biases = []
    
    print(f"\n📊 Running {duration}s simulation with {n_steps} steps")
    print(f"⏱️  Time step: {dt:.3f}s")
    print(f"🎯 Testing coordinate transformation fix")
    
    # Phase 1: Calibration (first 5 seconds)
    print("\n🔧 Phase 1: Calibration (0-5s)")
    for i in range(int(5.0 / dt)):
        t = i * dt
        
        # Simulate small sensor noise during calibration
        accel_noise = np.random.randn(2) * 0.01  # Small accelerometer noise
        gyro_noise = np.random.randn(1) * 0.001  # Small gyroscope noise
        
        # Control input with noise
        control = np.array([accel_noise[0], accel_noise[1], gyro_noise[0]])
        
        # Prediction with fixed coordinate transformation
        ekf.predict(dt, control)
        
        # Apply constraints
        ekf.update_zero_velocity()
        ekf.update_zero_angular_rate()
        ekf.update_stationary_mode()
        
        # Store data
        times.append(t)
        state = ekf.get_state()
        positions.append([state.x, state.y])
        velocities.append([state.vx, state.vy])
        orientations.append(state.theta)
        biases.append([state.bias_accel_x, state.bias_accel_y, state.bias_angular_velocity])
        
        if i % 50 == 0:  # Print every second
            print(f"  {t:.1f}s: pos=[{state.x:.6f}, {state.y:.6f}], "
                  f"vel=[{state.vx:.6f}, {state.vy:.6f}], "
                  f"theta={np.degrees(state.theta):.3f}°")
    
    # Phase 2: Stationary operation (remaining time)
    print("\n📱 Phase 2: Stationary Operation (5-30s)")
    for i in range(int(5.0 / dt), n_steps):
        t = i * dt
        
        # Simulate very small sensor noise (stationary device)
        accel_noise = np.random.randn(2) * 0.005  # Very small accelerometer noise
        gyro_noise = np.random.randn(1) * 0.0005  # Very small gyroscope noise
        
        # Control input with minimal noise
        control = np.array([accel_noise[0], accel_noise[1], gyro_noise[0]])
        
        # Prediction with fixed coordinate transformation
        ekf.predict(dt, control)
        
        # Apply constraints
        ekf.update_zero_velocity()
        ekf.update_zero_angular_rate()
        ekf.update_stationary_mode()
        
        # Store data
        times.append(t)
        state = ekf.get_state()
        positions.append([state.x, state.y])
        velocities.append([state.vx, state.vy])
        orientations.append(state.theta)
        biases.append([state.bias_accel_x, state.bias_accel_y, state.bias_angular_velocity])
        
        if i % 50 == 0:  # Print every second
            print(f"  {t:.1f}s: pos=[{state.x:.6f}, {state.y:.6f}], "
                  f"vel=[{state.vx:.6f}, {state.vy:.6f}], "
                  f"theta={np.degrees(state.theta):.3f}°")
    
    # Analysis
    print("\n📈 Coordinate Fix Analysis Results")
    print("=" * 40)
    
    positions = np.array(positions)
    velocities = np.array(velocities)
    orientations = np.array(orientations)
    biases = np.array(biases)
    
    # Position drift analysis
    pos_drift = np.linalg.norm(positions[-1] - positions[0])
    max_pos_drift = np.max(np.linalg.norm(positions, axis=1))
    
    # Individual axis analysis (this is key for detecting Y-direction spikes)
    x_range = np.max(positions[:, 0]) - np.min(positions[:, 0])
    y_range = np.max(positions[:, 1]) - np.min(positions[:, 1])
    
    # Velocity drift
    vel_drift = np.linalg.norm(velocities[-1] - velocities[0])
    max_vel = np.max(np.linalg.norm(velocities, axis=1))
    
    # Orientation drift
    orient_drift = abs(orientations[-1] - orientations[0])
    max_orient_drift = np.max(np.abs(orientations))
    
    print(f"📍 Position drift: {pos_drift:.6f} m (max: {max_pos_drift:.6f} m)")
    print(f"   X-axis range: {x_range:.6f} m")
    print(f"   Y-axis range: {y_range:.6f} m")
    print(f"🚀 Velocity drift: {vel_drift:.6f} m/s (max: {max_vel:.6f} m/s)")
    print(f"🔄 Orientation drift: {np.degrees(orient_drift):.4f}° (max: {np.degrees(max_orient_drift):.4f}°)")
    
    # Bias stability
    bias_stability = np.std(biases, axis=0)
    print(f"⚖️  Bias stability (std): accel_x={bias_stability[0]:.6f}, "
          f"accel_y={bias_stability[1]:.6f}, gyro={bias_stability[2]:.6f}")
    
    # Check for Y-direction spike (the main issue we're fixing)
    if y_range > 0.1:  # More than 10cm Y-range indicates a problem
        print(f"❌ Y-DIRECTION SPIKE DETECTED: {y_range:.6f} m range")
        print(f"   This suggests the coordinate transformation fix didn't work")
    elif y_range > 0.05:  # 5-10cm Y-range is concerning
        print(f"⚠️  MODERATE Y-DIRECTION DRIFT: {y_range:.6f} m range")
        print(f"   The fix may need further tuning")
    else:
        print(f"✅ Y-DIRECTION STABLE: {y_range:.6f} m range")
        print(f"   Coordinate transformation fix appears successful")
    
    # Overall drift assessment
    if pos_drift < 0.02 and vel_drift < 0.02 and orient_drift < 0.02 and y_range < 0.05:
        print("✅ EXCELLENT: Very low drift, no Y-direction spikes")
    elif pos_drift < 0.05 and vel_drift < 0.05 and orient_drift < 0.05 and y_range < 0.1:
        print("✅ GOOD: Low drift levels, minor Y-direction issues")
    elif pos_drift < 0.1 and vel_drift < 0.1 and orient_drift < 0.1 and y_range < 0.2:
        print("⚠️  MODERATE: Some drift detected, Y-direction needs attention")
    else:
        print("❌ POOR: Significant drift and Y-direction problems")
    
    # Compare with your problematic data
    print(f"\n📊 Comparison with Your Problematic Data:")
    print(f"   Original Y-range: 2.91m (MAJOR SPIKE)")
    print(f"   Fixed Y-range: {y_range:.6f}m")
    print(f"   Improvement: {((2.91 - y_range) / 2.91 * 100):.1f}%")
    
    return {
        'pos_drift': pos_drift,
        'vel_drift': vel_drift,
        'orient_drift': orient_drift,
        'x_range': x_range,
        'y_range': y_range,
        'max_pos_drift': max_pos_drift,
        'max_vel': max_vel,
        'max_orient_drift': max_orient_drift,
        'bias_stability': bias_stability
    }

def test_conservative_drift_resistance():
    """Test the conservative drift resistance approach"""
    
    print("\n🧪 Testing Conservative Drift Resistance")
    print("=" * 45)
    
    # Test different configurations
    configs = {
        'Original': {
            'q_accel': 0.5,
            'q_gyro': 0.01,
            'q_accel_bias': 1e-6,
            'q_gyro_bias': 1e-5
        },
        'Conservative': {
            'q_accel': 0.2,
            'q_gyro': 0.008,
            'q_accel_bias': 5e-7,
            'q_gyro_bias': 5e-6
        }
    }
    
    results = {}
    
    for config_name, config in configs.items():
        print(f"\n📊 Testing {config_name} Configuration:")
        print(f"   q_accel: {config['q_accel']}")
        print(f"   q_gyro: {config['q_gyro']}")
        print(f"   q_accel_bias: {config['q_accel_bias']}")
        print(f"   q_gyro_bias: {config['q_gyro_bias']}")
        
        # Initialize EKF
        ekf = RoboMasterEKF8DOF(config)
        
        # Quick test (10 seconds)
        dt = 0.02
        duration = 10.0
        n_steps = int(duration / dt)
        
        positions = []
        velocities = []
        
        for i in range(n_steps):
            # Simulate small sensor noise
            accel_noise = np.random.randn(2) * 0.005
            gyro_noise = np.random.randn(1) * 0.0005
            
            control = np.array([accel_noise[0], accel_noise[1], gyro_noise[0]])
            
            ekf.predict(dt, control)
            ekf.update_zero_velocity()
            ekf.update_zero_angular_rate()
            
            state = ekf.get_state()
            positions.append([state.x, state.y])
            velocities.append([state.vx, state.vy])
        
        # Calculate drift
        positions = np.array(positions)
        velocities = np.array(velocities)
        
        pos_drift = np.linalg.norm(positions[-1] - positions[0])
        vel_drift = np.linalg.norm(velocities[-1] - velocities[0])
        
        results[config_name] = {
            'pos_drift': pos_drift,
            'vel_drift': vel_drift
        }
        
        print(f"   Position drift: {pos_drift:.6f} m")
        print(f"   Velocity drift: {vel_drift:.6f} m/s")
    
    # Compare results
    print(f"\n📊 Configuration Comparison:")
    pos_improvement = ((results['Original']['pos_drift'] - results['Conservative']['pos_drift']) / 
                      results['Original']['pos_drift'] * 100)
    vel_improvement = ((results['Original']['vel_drift'] - results['Conservative']['vel_drift']) / 
                      results['Original']['vel_drift'] * 100)
    
    print(f"   Position drift improvement: {pos_improvement:.1f}%")
    print(f"   Velocity drift improvement: {vel_improvement:.1f}%")
    
    return results

def main():
    """Run comprehensive EKF tests"""
    
    print("🧪 RoboMaster EKF Comprehensive Test Suite")
    print("=" * 50)
    
    try:
        # Test 1: Coordinate transformation fix
        print("\n" + "="*60)
        results1 = test_coordinate_transformation_fix()
        
        # Test 2: Conservative drift resistance
        print("\n" + "="*60)
        results2 = test_conservative_drift_resistance()
        
        # Final assessment
        print("\n" + "="*60)
        print("🎯 FINAL ASSESSMENT")
        print("="*60)
        
        if results1['y_range'] < 0.05:
            print("✅ COORDINATE TRANSFORMATION: SUCCESS - No Y-direction spikes")
        else:
            print("❌ COORDINATE TRANSFORMATION: FAILED - Y-direction issues persist")
        
        if results1['pos_drift'] < 0.02:
            print("✅ DRIFT RESISTANCE: EXCELLENT - Very low drift detected")
        elif results1['pos_drift'] < 0.05:
            print("✅ DRIFT RESISTANCE: GOOD - Low drift levels")
        else:
            print("❌ DRIFT RESISTANCE: POOR - Significant drift detected")
        
        print(f"\n📊 Final Results Summary:")
        print(f"   Y-direction range: {results1['y_range']:.6f} m")
        print(f"   Position drift: {results1['pos_drift']:.6f} m")
        print(f"   Velocity drift: {results1['vel_drift']:.6f} m/s")
        print(f"   Orientation drift: {np.degrees(results1['orient_drift']):.4f}°")
        
        print(f"\n🎉 Test suite completed successfully!")
        
    except Exception as e:
        print(f"❌ Test suite failed: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()
