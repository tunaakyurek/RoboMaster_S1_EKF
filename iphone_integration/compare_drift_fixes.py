#!/usr/bin/env python3
"""
Compare Drift Fixes - Before vs After
=====================================
This script demonstrates the dramatic improvement in drift resistance
by comparing the old and new EKF implementations.
"""

import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime

# Import our modules
try:
    from pi_phone_connection.ekf_robomaster_8dof import RoboMasterEKF8DOF, RoboMasterState
except ImportError:
    from ekf_robomaster_8dof import RoboMasterEKF8DOF, RoboMasterState

def simulate_old_ekf():
    """Simulate the old drift-prone EKF implementation"""
    # Old configuration (drift-prone)
    old_config = {
        'q_accel': 0.5,        # Original high value
        'q_gyro': 0.01,        # Original high value
        'q_accel_bias': 1e-6,  # Original high value
        'q_gyro_bias': 1e-5,   # Original high value
        'r_zupt': 0.01,
        'r_zaru': 0.001
    }
    
    ekf = RoboMasterEKF8DOF(old_config)
    
    # Simulate stationary operation
    dt = 0.02
    duration = 20.0
    n_steps = int(duration / dt)
    
    times = []
    positions = []
    velocities = []
    orientations = []
    
    print("🔴 Simulating OLD EKF (drift-prone)...")
    
    for i in range(n_steps):
        t = i * dt
        
        # Simulate sensor noise
        accel_noise = np.random.randn(2) * 0.01
        gyro_noise = np.random.randn(1) * 0.001
        
        control = np.array([accel_noise[0], accel_noise[1], gyro_noise[0]])
        
        # Prediction
        ekf.predict(dt, control)
        
        # Apply basic constraints (old implementation)
        ekf.update_zero_velocity()
        ekf.update_zero_angular_rate()
        
        # Store data
        times.append(t)
        state = ekf.get_state()
        positions.append([state.x, state.y])
        velocities.append([state.vx, state.vy])
        orientations.append(state.theta)
    
    return np.array(times), np.array(positions), np.array(velocities), np.array(orientations)

def simulate_new_ekf():
    """Simulate the new drift-resistant EKF implementation"""
    # New configuration (drift-resistant)
    new_config = {
        'q_accel': 0.1,        # Reduced from 0.5
        'q_gyro': 0.005,       # Reduced from 0.01
        'q_accel_bias': 1e-7,  # Reduced from 1e-6
        'q_gyro_bias': 1e-6,   # Reduced from 1e-5
        'r_zupt': 0.01,
        'r_zaru': 0.001
    }
    
    ekf = RoboMasterEKF8DOF(new_config)
    
    # Simulate stationary operation
    dt = 0.02
    duration = 20.0
    n_steps = int(duration / dt)
    
    times = []
    positions = []
    velocities = []
    orientations = []
    
    print("🟢 Simulating NEW EKF (drift-resistant)...")
    
    for i in range(n_steps):
        t = i * dt
        
        # Simulate sensor noise
        accel_noise = np.random.randn(2) * 0.01
        gyro_noise = np.random.randn(1) * 0.001
        
        control = np.array([accel_noise[0], accel_noise[1], gyro_noise[0]])
        
        # Prediction
        ekf.predict(dt, control)
        
        # Apply enhanced constraints (new implementation)
        ekf.update_zero_velocity()
        ekf.update_zero_angular_rate()
        ekf.update_stationary_mode()
        
        # Store data
        times.append(t)
        state = ekf.get_state()
        positions.append([state.x, state.y])
        velocities.append([state.vx, state.vy])
        orientations.append(state.theta)
    
    return np.array(times), np.array(positions), np.array(velocities), np.array(orientations)

def create_comparison_plot():
    """Create comparison plot showing before vs after"""
    
    print("📊 Running drift comparison...")
    
    # Simulate both implementations
    old_times, old_positions, old_velocities, old_orientations = simulate_old_ekf()
    new_times, new_positions, new_velocities, new_orientations = simulate_new_ekf()
    
    # Calculate drift metrics
    old_pos_drift = np.linalg.norm(old_positions[-1] - old_positions[0])
    old_vel_drift = np.linalg.norm(old_velocities[-1] - old_velocities[0])
    old_orient_drift = abs(old_orientations[-1] - old_orientations[0])
    
    new_pos_drift = np.linalg.norm(new_positions[-1] - new_positions[0])
    new_vel_drift = np.linalg.norm(new_velocities[-1] - new_velocities[0])
    new_orient_drift = abs(new_orientations[-1] - new_orientations[0])
    
    # Print comparison
    print("\n📈 DRIFT COMPARISON RESULTS")
    print("=" * 40)
    print(f"📍 Position Drift:")
    print(f"   🔴 OLD: {old_pos_drift:.6f} m")
    print(f"   🟢 NEW: {new_pos_drift:.6f} m")
    print(f"   📉 Improvement: {((old_pos_drift - new_pos_drift) / old_pos_drift * 100):.1f}%")
    
    print(f"\n🚀 Velocity Drift:")
    print(f"   🔴 OLD: {old_vel_drift:.6f} m/s")
    print(f"   🟢 NEW: {new_vel_drift:.6f} m/s")
    print(f"   📉 Improvement: {((old_vel_drift - new_vel_drift) / old_vel_drift * 100):.1f}%")
    
    print(f"\n🔄 Orientation Drift:")
    print(f"   🔴 OLD: {np.degrees(old_orient_drift):.4f}°")
    print(f"   🟢 NEW: {np.degrees(new_orient_drift):.4f}°")
    print(f"   📉 Improvement: {((old_orient_drift - new_orient_drift) / old_orient_drift * 100):.1f}%")
    
    # Create comparison plot
    fig, axes = plt.subplots(2, 3, figsize=(18, 10))
    fig.suptitle('RoboMaster EKF Drift Comparison: Before vs After Fixes', fontsize=16)
    
    # Position comparison
    old_pos_mag = np.linalg.norm(old_positions, axis=1)
    new_pos_mag = np.linalg.norm(new_positions, axis=1)
    
    axes[0, 0].plot(old_times, old_pos_mag, 'r-', linewidth=2, label='OLD (Drift-Prone)')
    axes[0, 0].plot(new_times, new_pos_mag, 'g-', linewidth=2, label='NEW (Drift-Resistant)')
    axes[0, 0].set_xlabel('Time (s)')
    axes[0, 0].set_ylabel('Position Magnitude (m)')
    axes[0, 0].set_title('Position Drift Comparison')
    axes[0, 0].grid(True)
    axes[0, 0].legend()
    axes[0, 0].axvline(x=5.0, color='orange', linestyle='--', alpha=0.7, label='Calibration End')
    
    # Velocity comparison
    old_vel_mag = np.linalg.norm(old_velocities, axis=1)
    new_vel_mag = np.linalg.norm(new_velocities, axis=1)
    
    axes[0, 1].plot(old_times, old_vel_mag, 'r-', linewidth=2, label='OLD (Drift-Prone)')
    axes[0, 1].plot(new_times, new_vel_mag, 'g-', linewidth=2, label='NEW (Drift-Resistant)')
    axes[0, 1].set_xlabel('Time (s)')
    axes[0, 1].set_ylabel('Velocity Magnitude (m/s)')
    axes[0, 1].set_title('Velocity Drift Comparison')
    axes[0, 1].grid(True)
    axes[0, 1].legend()
    axes[0, 1].axvline(x=5.0, color='orange', linestyle='--', alpha=0.7, label='Calibration End')
    
    # Orientation comparison
    axes[0, 2].plot(old_times, np.degrees(old_orientations), 'r-', linewidth=2, label='OLD (Drift-Prone)')
    axes[0, 2].plot(new_times, np.degrees(new_orientations), 'g-', linewidth=2, label='NEW (Drift-Resistant)')
    axes[0, 2].set_xlabel('Time (s)')
    axes[0, 2].set_ylabel('Orientation (degrees)')
    axes[0, 2].set_title('Orientation Drift Comparison')
    axes[0, 2].grid(True)
    axes[0, 2].legend()
    axes[0, 2].axvline(x=5.0, color='orange', linestyle='--', alpha=0.7, label='Calibration End')
    
    # 2D Trajectory comparison
    axes[1, 0].plot(old_positions[:, 0], old_positions[:, 1], 'r-', linewidth=2, label='OLD Trajectory')
    axes[1, 0].plot(new_positions[:, 0], new_positions[:, 1], 'g-', linewidth=2, label='NEW Trajectory')
    axes[1, 0].plot(old_positions[0, 0], old_positions[0, 1], 'ko', markersize=8, label='Start')
    axes[1, 0].plot(old_positions[-1, 0], old_positions[-1, 1], 'rs', markersize=8, label='OLD End')
    axes[1, 0].plot(new_positions[-1, 0], new_positions[-1, 1], 'gs', markersize=8, label='NEW End')
    axes[1, 0].set_xlabel('X Position (m)')
    axes[1, 0].set_ylabel('Y Position (m)')
    axes[1, 0].set_title('2D Trajectory Comparison')
    axes[1, 0].grid(True)
    axes[1, 0].legend()
    axes[1, 0].axis('equal')
    
    # Drift metrics bar chart
    drift_metrics = ['Position', 'Velocity', 'Orientation']
    old_drifts = [old_pos_drift * 1000, old_vel_drift * 1000, np.degrees(old_orient_drift)]
    new_drifts = [new_pos_drift * 1000, new_vel_drift * 1000, np.degrees(new_orient_drift)]
    
    x = np.arange(len(drift_metrics))
    width = 0.35
    
    axes[1, 1].bar(x - width/2, old_drifts, width, label='OLD (Drift-Prone)', color='red', alpha=0.7)
    axes[1, 1].bar(x + width/2, new_drifts, width, label='NEW (Drift-Resistant)', color='green', alpha=0.7)
    axes[1, 1].set_xlabel('Drift Type')
    axes[1, 1].set_ylabel('Drift Magnitude')
    axes[1, 1].set_title('Drift Metrics Comparison')
    axes[1, 1].set_xticks(x)
    axes[1, 1].set_xticklabels(drift_metrics)
    axes[1, 1].legend()
    axes[1, 1].grid(True, axis='y')
    
    # Improvement percentage
    improvements = [
        ((old_pos_drift - new_pos_drift) / old_pos_drift * 100),
        ((old_vel_drift - new_vel_drift) / old_vel_drift * 100),
        ((old_orient_drift - new_orient_drift) / old_orient_drift * 100)
    ]
    
    axes[1, 2].bar(drift_metrics, improvements, color=['blue', 'green', 'orange'], alpha=0.7)
    axes[1, 2].set_xlabel('Drift Type')
    axes[1, 2].set_ylabel('Improvement (%)')
    axes[1, 2].set_title('Drift Reduction Improvement')
    axes[1, 2].grid(True, axis='y')
    
    # Add value labels on bars
    for i, v in enumerate(improvements):
        axes[1, 2].text(i, v + 1, f'{v:.1f}%', ha='center', va='bottom', fontweight='bold')
    
    plt.tight_layout()
    
    # Save plot
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"drift_comparison_{timestamp}.png"
    plt.savefig(filename, dpi=300, bbox_inches='tight')
    print(f"📊 Comparison plot saved as: {filename}")
    
    plt.show()
    
    return {
        'old_drifts': [old_pos_drift, old_vel_drift, old_orient_drift],
        'new_drifts': [new_pos_drift, new_vel_drift, new_orient_drift],
        'improvements': improvements
    }

if __name__ == "__main__":
    try:
        results = create_comparison_plot()
        print(f"\n🎯 Comparison completed successfully!")
        print(f"📊 Overall improvement: {np.mean(results['improvements']):.1f}% average drift reduction")
    except Exception as e:
        print(f"❌ Comparison failed: {e}")
        import traceback
        traceback.print_exc()
