#!/usr/bin/env python3
"""
Test Conservative Drift Fix - RoboMaster EKF
============================================
This script tests a more conservative approach to drift resistance
that doesn't over-constrain the system.
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

def test_conservative_drift_fix():
    """Test the conservative EKF drift fix approach"""
    
    print("🧪 Testing Conservative RoboMaster EKF Drift Fix")
    print("=" * 55)
    
    # Conservative configuration - moderate improvements without over-constraining
    conservative_config = {
        'q_accel': 0.2,        # Conservative reduction from 0.5
        'q_gyro': 0.008,       # Conservative reduction from 0.01
        'q_accel_bias': 5e-7,  # Conservative reduction from 1e-6
        'q_gyro_bias': 5e-6,   # Conservative reduction from 1e-5
        'r_zupt': 0.01,
        'r_zaru': 0.001
    }
    
    # Original configuration for comparison
    original_config = {
        'q_accel': 0.5,        # Original high value
        'q_gyro': 0.01,        # Original high value
        'q_accel_bias': 1e-6,  # Original high value
        'q_gyro_bias': 1e-5,   # Original high value
        'r_zupt': 0.01,
        'r_zaru': 0.001
    }
    
    print("📊 Testing Conservative Configuration:")
    print(f"   q_accel: {original_config['q_accel']} → {conservative_config['q_accel']}")
    print(f"   q_gyro: {original_config['q_gyro']} → {conservative_config['q_gyro']}")
    print(f"   q_accel_bias: {original_config['q_accel_bias']} → {conservative_config['q_accel_bias']}")
    print(f"   q_gyro_bias: {original_config['q_gyro_bias']} → {conservative_config['q_gyro_bias']}")
    
    # Initialize EKF with conservative config
    ekf = RoboMasterEKF8DOF(conservative_config)
    
    # Simulate stationary operation
    dt = 0.02  # 50 Hz
    duration = 20.0  # 20 seconds
    n_steps = int(duration / dt)
    
    # Storage for analysis
    times = []
    positions = []
    velocities = []
    orientations = []
    biases = []
    cov_traces = []
    
    print(f"\n📊 Running {duration}s simulation with {n_steps} steps")
    print(f"⏱️  Time step: {dt:.3f}s")
    
    # Phase 1: Calibration (first 5 seconds)
    print("\n🔧 Phase 1: Calibration (0-5s)")
    for i in range(int(5.0 / dt)):
        t = i * dt
        
        # Simulate small sensor noise during calibration
        accel_noise = np.random.randn(2) * 0.01  # Small accelerometer noise
        gyro_noise = np.random.randn(1) * 0.001  # Small gyroscope noise
        
        # Control input with noise
        control = np.array([accel_noise[0], accel_noise[1], gyro_noise[0]])
        
        # Prediction
        ekf.predict(dt, control)
        
        # Apply conservative constraints
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
        cov_traces.append(np.trace(ekf.get_covariance()))
        
        if i % 50 == 0:  # Print every second
            print(f"  {t:.1f}s: pos=[{state.x:.4f}, {state.y:.4f}], "
                  f"vel=[{state.vx:.4f}, {state.vy:.4f}], "
                  f"theta={np.degrees(state.theta):.2f}°")
    
    # Phase 2: Stationary operation (remaining time)
    print("\n📱 Phase 2: Stationary Operation (5-20s)")
    for i in range(int(5.0 / dt), n_steps):
        t = i * dt
        
        # Simulate very small sensor noise (stationary device)
        accel_noise = np.random.randn(2) * 0.005  # Very small accelerometer noise
        gyro_noise = np.random.randn(1) * 0.0005  # Very small gyroscope noise
        
        # Control input with minimal noise
        control = np.array([accel_noise[0], accel_noise[1], gyro_noise[0]])
        
        # Prediction
        ekf.predict(dt, control)
        
        # Apply conservative constraints
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
        cov_traces.append(np.trace(ekf.get_covariance()))
        
        if i % 50 == 0:  # Print every second
            print(f"  {t:.1f}s: pos=[{state.x:.4f}, {state.y:.4f}], "
                  f"vel=[{state.vx:.4f}, {state.vy:.4f}], "
                  f"theta={np.degrees(state.theta):.2f}°")
    
    # Analysis
    print("\n📈 Conservative Drift Analysis Results")
    print("=" * 40)
    
    positions = np.array(positions)
    velocities = np.array(velocities)
    orientations = np.array(orientations)
    biases = np.array(biases)
    
    # Position drift
    pos_drift = np.linalg.norm(positions[-1] - positions[0])
    max_pos_drift = np.max(np.linalg.norm(positions, axis=1))
    
    # Velocity drift
    vel_drift = np.linalg.norm(velocities[-1] - velocities[0])
    max_vel = np.max(np.linalg.norm(velocities, axis=1))
    
    # Orientation drift
    orient_drift = abs(orientations[-1] - orientations[0])
    max_orient_drift = np.max(np.abs(orientations))
    
    print(f"📍 Position drift: {pos_drift:.6f} m (max: {max_pos_drift:.6f} m)")
    print(f"🚀 Velocity drift: {vel_drift:.6f} m/s (max: {max_vel:.6f} m/s)")
    print(f"🔄 Orientation drift: {np.degrees(orient_drift):.4f}° (max: {np.degrees(max_orient_drift):.4f}°)")
    
    # Bias stability
    bias_stability = np.std(biases, axis=0)
    print(f"⚖️  Bias stability (std): accel_x={bias_stability[0]:.6f}, "
          f"accel_y={bias_stability[1]:.6f}, gyro={bias_stability[2]:.6f}")
    
    # Drift assessment
    if pos_drift < 0.02 and vel_drift < 0.02 and orient_drift < 0.02:
        print("✅ EXCELLENT: Very low drift detected")
    elif pos_drift < 0.05 and vel_drift < 0.05 and orient_drift < 0.05:
        print("✅ GOOD: Low drift levels")
    elif pos_drift < 0.1 and vel_drift < 0.1 and orient_drift < 0.1:
        print("⚠️  MODERATE: Some drift detected")
    else:
        print("❌ POOR: Significant drift detected")
    
    # Compare with your original data
    print(f"\n📊 Comparison with Your Original Data:")
    print(f"   Original position drift: 0.067m X, 0.149m Y over 19.2s")
    print(f"   Conservative fix result: {pos_drift:.6f}m over {duration:.1f}s")
    print(f"   Original velocity drift: ~0.05 m/s")
    print(f"   Conservative fix result: {vel_drift:.6f} m/s")
    print(f"   Original orientation drift: 11.5°")
    print(f"   Conservative fix result: {np.degrees(orient_drift):.4f}°")
    
    # Create visualization
    create_conservative_analysis_plot(times, positions, velocities, orientations, biases, cov_traces)
    
    return {
        'pos_drift': pos_drift,
        'vel_drift': vel_drift,
        'orient_drift': orient_drift,
        'max_pos_drift': max_pos_drift,
        'max_vel': max_vel,
        'max_orient_drift': max_orient_drift,
        'bias_stability': bias_stability
    }

def create_conservative_analysis_plot(times, positions, velocities, orientations, biases, cov_traces):
    """Create comprehensive analysis plot for conservative approach"""
    
    fig, axes = plt.subplots(3, 3, figsize=(15, 12))
    fig.suptitle('RoboMaster EKF Conservative Drift Fix Analysis', fontsize=16)
    
    # Convert to numpy arrays
    times = np.array(times)
    positions = np.array(positions)
    velocities = np.array(velocities)
    orientations = np.array(orientations)
    biases = np.array(biases)
    cov_traces = np.array(cov_traces)
    
    # 1. 2D Trajectory
    axes[0, 0].plot(positions[:, 0], positions[:, 1], 'b-', linewidth=2)
    axes[0, 0].plot(positions[0, 0], positions[0, 1], 'go', markersize=8, label='Start')
    axes[0, 0].plot(positions[-1, 0], positions[-1, 1], 'ro', markersize=8, label='End')
    axes[0, 0].set_xlabel('X Position (m)')
    axes[0, 0].set_ylabel('Y Position (m)')
    axes[0, 0].set_title('2D Trajectory (Conservative)')
    axes[0, 0].grid(True)
    axes[0, 0].legend()
    axes[0, 0].axis('equal')
    
    # 2. Position vs Time
    axes[0, 1].plot(times, positions[:, 0], 'r-', label='X', linewidth=2)
    axes[0, 1].plot(times, positions[:, 1], 'g-', label='Y', linewidth=2)
    axes[0, 1].set_xlabel('Time (s)')
    axes[0, 1].set_ylabel('Position (m)')
    axes[0, 1].set_title('Position vs Time (Conservative)')
    axes[0, 1].grid(True)
    axes[0, 1].legend()
    
    # 3. Orientation vs Time
    axes[0, 2].plot(times, np.degrees(orientations), 'm-', linewidth=2)
    axes[0, 2].set_xlabel('Time (s)')
    axes[0, 2].set_ylabel('Orientation (degrees)')
    axes[0, 2].set_title('Orientation vs Time (Conservative)')
    axes[0, 2].grid(True)
    
    # 4. Velocity vs Time
    axes[1, 0].plot(times, velocities[:, 0], 'r-', label='vx', linewidth=2)
    axes[1, 0].plot(times, velocities[:, 1], 'g-', label='vy', linewidth=2)
    axes[1, 0].plot(times, np.linalg.norm(velocities, axis=1), 'b--', label='Speed', linewidth=2)
    axes[1, 0].set_xlabel('Time (s)')
    axes[1, 0].set_ylabel('Velocity (m/s)')
    axes[1, 0].set_title('Velocity vs Time (Conservative)')
    axes[1, 0].grid(True)
    axes[1, 0].legend()
    
    # 5. Accelerometer Bias
    axes[1, 1].plot(times, biases[:, 0], 'r-', label='Bias X', linewidth=2)
    axes[1, 1].plot(times, biases[:, 1], 'g-', label='Bias Y', linewidth=2)
    axes[1, 1].set_xlabel('Time (s)')
    axes[1, 1].set_ylabel('Accel Bias (m/s²)')
    axes[1, 1].set_title('Accelerometer Bias (Conservative)')
    axes[1, 1].grid(True)
    axes[1, 1].legend()
    
    # 6. Gyroscope Bias
    axes[1, 2].plot(times, biases[:, 2], 'c-', linewidth=2)
    axes[1, 2].set_xlabel('Time (s)')
    axes[1, 2].set_ylabel('Gyro Bias (rad/s)')
    axes[1, 2].set_title('Gyroscope Bias (Conservative)')
    axes[1, 2].grid(True)
    
    # 7. Raw Position Magnitude
    pos_magnitude = np.linalg.norm(positions, axis=1)
    axes[2, 0].plot(times, pos_magnitude, 'b-', linewidth=2)
    axes[2, 0].set_xlabel('Time (s)')
    axes[2, 0].set_ylabel('Position Magnitude (m)')
    axes[2, 0].set_title('Position Magnitude vs Time (Conservative)')
    axes[2, 0].grid(True)
    
    # 8. Raw Velocity Magnitude
    vel_magnitude = np.linalg.norm(velocities, axis=1)
    axes[2, 1].plot(times, vel_magnitude, 'g-', linewidth=2)
    axes[2, 1].set_xlabel('Time (s)')
    axes[2, 1].set_ylabel('Velocity Magnitude (m/s)')
    axes[2, 1].set_title('Velocity Magnitude vs Time (Conservative)')
    axes[2, 1].grid(True)
    
    # 9. Covariance Trace
    axes[2, 2].semilogy(times, cov_traces, 'k-', linewidth=2)
    axes[2, 2].set_xlabel('Time (s)')
    axes[2, 2].set_ylabel('Covariance Trace (log scale)')
    axes[2, 2].set_title('EKF Uncertainty (Conservative)')
    axes[2, 2].grid(True)
    
    # Add calibration phase indicator
    for ax in axes.flat:
        ax.axvline(x=5.0, color='orange', linestyle='--', alpha=0.7, label='Calibration End')
    
    plt.tight_layout()
    
    # Save plot
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"conservative_drift_analysis_{timestamp}.png"
    plt.savefig(filename, dpi=300, bbox_inches='tight')
    print(f"📊 Plot saved as: {filename}")
    
    plt.show()

if __name__ == "__main__":
    try:
        results = test_conservative_drift_fix()
        print(f"\n🎯 Conservative test completed successfully!")
        print(f"📊 Final drift metrics: {results}")
    except Exception as e:
        print(f"❌ Conservative test failed: {e}")
        import traceback
        traceback.print_exc()
