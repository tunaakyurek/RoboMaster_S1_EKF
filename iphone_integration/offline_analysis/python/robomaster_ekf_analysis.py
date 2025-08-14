#!/usr/bin/env python3
"""
RoboMaster EKF Offline Analysis
===============================
Analyze RoboMaster formulary EKF log files with state vector:
[x, y, theta, vx, vy, bias_accel_x, bias_accel_y, bias_angular_velocity]

Usage:
    python robomaster_ekf_analysis.py --file path/to/robomaster_ekf_log.csv
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import argparse
import os
from datetime import datetime
import sys

def load_robomaster_data(csv_file):
    """Load RoboMaster EKF CSV log file"""
    print(f"📁 Loading RoboMaster EKF data from: {csv_file}")
    
    # Expected columns for RoboMaster formulary
    expected_cols = [
        'timestamp', 'x', 'y', 'theta', 'vx', 'vy', 
        'bias_accel_x', 'bias_accel_y', 'bias_angular_velocity',
        'accel_x', 'accel_y', 'accel_z', 
        'gyro_x', 'gyro_y', 'gyro_z', 'cov_trace'
    ]
    
    try:
        df = pd.read_csv(csv_file)
        print(f"✅ Loaded {len(df)} samples")
        print(f"📊 Columns: {list(df.columns)}")
        
        # Validate columns
        missing_cols = [col for col in expected_cols if col not in df.columns]
        if missing_cols:
            print(f"⚠️ Missing columns: {missing_cols}")
        
        # Convert timestamp to relative time
        df['time_rel'] = df['timestamp'] - df['timestamp'].iloc[0]
        
        # Convert theta from radians to degrees for plotting
        df['theta_deg'] = np.degrees(df['theta'])
        
        print(f"⏱️ Data duration: {df['time_rel'].iloc[-1]:.1f} seconds")
        print(f"📈 Sampling rate: {len(df) / df['time_rel'].iloc[-1]:.1f} Hz")
        
        return df
        
    except Exception as e:
        print(f"❌ Error loading file: {e}")
        return None

def analyze_robomaster_trajectory(df):
    """Analyze RoboMaster trajectory data"""
    print("\n🎯 RoboMaster Trajectory Analysis")
    print("=" * 50)
    
    # Position statistics
    pos_stats = {
        'x_range': df['x'].max() - df['x'].min(),
        'y_range': df['y'].max() - df['y'].min(),
        'total_distance': np.sum(np.sqrt(np.diff(df['x'])**2 + np.diff(df['y'])**2)),
        'max_distance_from_origin': np.sqrt(df['x']**2 + df['y']**2).max()
    }
    
    # Velocity statistics
    vel_stats = {
        'max_vx': df['vx'].max(),
        'max_vy': df['vy'].max(),
        'max_speed': np.sqrt(df['vx']**2 + df['vy']**2).max(),
        'avg_speed': np.sqrt(df['vx']**2 + df['vy']**2).mean()
    }
    
    # Orientation statistics
    theta_stats = {
        'theta_range_deg': df['theta_deg'].max() - df['theta_deg'].min(),
        'max_angular_change': np.abs(np.diff(df['theta_deg'])).max()
    }
    
    # Bias statistics
    bias_stats = {
        'final_bias_accel_x': df['bias_accel_x'].iloc[-1],
        'final_bias_accel_y': df['bias_accel_y'].iloc[-1],
        'final_bias_angular': df['bias_angular_velocity'].iloc[-1],
        'bias_convergence_x': np.abs(df['bias_accel_x'].iloc[-1] - df['bias_accel_x'].iloc[len(df)//2]),
        'bias_convergence_y': np.abs(df['bias_accel_y'].iloc[-1] - df['bias_accel_y'].iloc[len(df)//2])
    }
    
    print("📍 Position Analysis:")
    print(f"   X range: {pos_stats['x_range']:.3f} m")
    print(f"   Y range: {pos_stats['y_range']:.3f} m")
    print(f"   Total distance: {pos_stats['total_distance']:.3f} m")
    print(f"   Max distance from origin: {pos_stats['max_distance_from_origin']:.3f} m")
    
    print("\n🚀 Velocity Analysis:")
    print(f"   Max vx: {vel_stats['max_vx']:.3f} m/s")
    print(f"   Max vy: {vel_stats['max_vy']:.3f} m/s")
    print(f"   Max speed: {vel_stats['max_speed']:.3f} m/s")
    print(f"   Average speed: {vel_stats['avg_speed']:.3f} m/s")
    
    print("\n🔄 Orientation Analysis:")
    print(f"   Theta range: {theta_stats['theta_range_deg']:.1f}°")
    print(f"   Max angular change: {theta_stats['max_angular_change']:.1f}°")
    
    print("\n⚖️ Sensor Bias Analysis:")
    print(f"   Final accel bias X: {bias_stats['final_bias_accel_x']:.4f} m/s²")
    print(f"   Final accel bias Y: {bias_stats['final_bias_accel_y']:.4f} m/s²")
    print(f"   Final angular bias: {bias_stats['final_bias_angular']:.4f} rad/s")
    print(f"   Bias convergence X: {bias_stats['bias_convergence_x']:.4f} m/s²")
    print(f"   Bias convergence Y: {bias_stats['bias_convergence_y']:.4f} m/s²")
    
    return pos_stats, vel_stats, theta_stats, bias_stats

def plot_robomaster_analysis(df, output_dir):
    """Create comprehensive RoboMaster analysis plots"""
    print(f"\n📊 Creating RoboMaster analysis plots...")
    
    # Create figure with subplots
    fig = plt.figure(figsize=(16, 12))
    
    # 1. 2D Trajectory
    ax1 = plt.subplot(3, 3, 1)
    plt.plot(df['x'], df['y'], 'b-', linewidth=2, alpha=0.7)
    plt.plot(df['x'].iloc[0], df['y'].iloc[0], 'go', markersize=10, label='Start')
    plt.plot(df['x'].iloc[-1], df['y'].iloc[-1], 'ro', markersize=10, label='End')
    plt.xlabel('X Position (m)')
    plt.ylabel('Y Position (m)')
    plt.title('RoboMaster 2D Trajectory')
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.axis('equal')
    
    # 2. Position vs Time
    ax2 = plt.subplot(3, 3, 2)
    plt.plot(df['time_rel'], df['x'], 'r-', label='X', linewidth=2)
    plt.plot(df['time_rel'], df['y'], 'g-', label='Y', linewidth=2)
    plt.xlabel('Time (s)')
    plt.ylabel('Position (m)')
    plt.title('Position vs Time')
    plt.grid(True, alpha=0.3)
    plt.legend()
    
    # 3. Orientation vs Time
    ax3 = plt.subplot(3, 3, 3)
    plt.plot(df['time_rel'], df['theta_deg'], 'm-', linewidth=2)
    plt.xlabel('Time (s)')
    plt.ylabel('Orientation (degrees)')
    plt.title('Orientation vs Time')
    plt.grid(True, alpha=0.3)
    
    # 4. Velocity vs Time
    ax4 = plt.subplot(3, 3, 4)
    plt.plot(df['time_rel'], df['vx'], 'r-', label='vx', linewidth=2)
    plt.plot(df['time_rel'], df['vy'], 'g-', label='vy', linewidth=2)
    speed = np.sqrt(df['vx']**2 + df['vy']**2)
    plt.plot(df['time_rel'], speed, 'b--', label='speed', linewidth=2)
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity (m/s)')
    plt.title('Velocity vs Time')
    plt.grid(True, alpha=0.3)
    plt.legend()
    
    # 5. Accelerometer Biases
    ax5 = plt.subplot(3, 3, 5)
    plt.plot(df['time_rel'], df['bias_accel_x'], 'r-', label='Bias X', linewidth=2)
    plt.plot(df['time_rel'], df['bias_accel_y'], 'g-', label='Bias Y', linewidth=2)
    plt.xlabel('Time (s)')
    plt.ylabel('Accel Bias (m/s²)')
    plt.title('Accelerometer Bias Estimation')
    plt.grid(True, alpha=0.3)
    plt.legend()
    
    # 6. Angular Velocity Bias
    ax6 = plt.subplot(3, 3, 6)
    plt.plot(df['time_rel'], df['bias_angular_velocity'], 'c-', linewidth=2)
    plt.xlabel('Time (s)')
    plt.ylabel('Angular Bias (rad/s)')
    plt.title('Gyroscope Bias Estimation')
    plt.grid(True, alpha=0.3)
    
    # 7. Sensor Data - Accelerometer
    ax7 = plt.subplot(3, 3, 7)
    plt.plot(df['time_rel'], df['accel_x'], 'r-', alpha=0.7, label='ax')
    plt.plot(df['time_rel'], df['accel_y'], 'g-', alpha=0.7, label='ay')
    plt.plot(df['time_rel'], df['accel_z'], 'b-', alpha=0.7, label='az')
    plt.xlabel('Time (s)')
    plt.ylabel('Acceleration (m/s²)')
    plt.title('Raw Accelerometer Data')
    plt.grid(True, alpha=0.3)
    plt.legend()
    
    # 8. Sensor Data - Gyroscope
    ax8 = plt.subplot(3, 3, 8)
    plt.plot(df['time_rel'], df['gyro_x'], 'r-', alpha=0.7, label='ωx')
    plt.plot(df['time_rel'], df['gyro_y'], 'g-', alpha=0.7, label='ωy')
    plt.plot(df['time_rel'], df['gyro_z'], 'b-', alpha=0.7, label='ωz')
    plt.xlabel('Time (s)')
    plt.ylabel('Angular Velocity (rad/s)')
    plt.title('Raw Gyroscope Data')
    plt.grid(True, alpha=0.3)
    plt.legend()
    
    # 9. Covariance Trace
    ax9 = plt.subplot(3, 3, 9)
    plt.semilogy(df['time_rel'], df['cov_trace'], 'k-', linewidth=2)
    plt.xlabel('Time (s)')
    plt.ylabel('Covariance Trace')
    plt.title('EKF Uncertainty (Log Scale)')
    plt.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    # Save plots
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    plot_file = os.path.join(output_dir, f'robomaster_analysis_{timestamp}.png')
    plt.savefig(plot_file, dpi=300, bbox_inches='tight')
    print(f"📈 Plots saved to: {plot_file}")
    
    plt.show()

def generate_report(df, pos_stats, vel_stats, theta_stats, bias_stats, output_dir):
    """Generate analysis report"""
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    report_file = os.path.join(output_dir, f'robomaster_analysis_report_{timestamp}.txt')
    
    with open(report_file, 'w') as f:
        f.write("RoboMaster EKF Analysis Report\n")
        f.write("=" * 50 + "\n")
        f.write(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        f.write(f"Data file: {len(df)} samples\n")
        f.write(f"Duration: {df['time_rel'].iloc[-1]:.1f} seconds\n")
        f.write(f"Sample rate: {len(df) / df['time_rel'].iloc[-1]:.1f} Hz\n\n")
        
        f.write("State Vector: [x, y, theta, vx, vy, bias_accel_x, bias_accel_y, bias_angular_velocity]\n\n")
        
        f.write("Position Analysis:\n")
        for key, value in pos_stats.items():
            f.write(f"  {key}: {value:.4f}\n")
        
        f.write("\nVelocity Analysis:\n")
        for key, value in vel_stats.items():
            f.write(f"  {key}: {value:.4f}\n")
        
        f.write("\nOrientation Analysis:\n")
        for key, value in theta_stats.items():
            f.write(f"  {key}: {value:.4f}\n")
        
        f.write("\nBias Analysis:\n")
        for key, value in bias_stats.items():
            f.write(f"  {key}: {value:.6f}\n")
    
    print(f"📄 Report saved to: {report_file}")

def main():
    parser = argparse.ArgumentParser(description='Analyze RoboMaster EKF log files')
    parser.add_argument('--file', required=True, help='Path to RoboMaster EKF CSV log file')
    parser.add_argument('--output', default='analysis_results', help='Output directory for plots and reports')
    
    args = parser.parse_args()
    
    # Create output directory
    os.makedirs(args.output, exist_ok=True)
    
    # Load data
    df = load_robomaster_data(args.file)
    if df is None:
        return
    
    # Analyze trajectory
    pos_stats, vel_stats, theta_stats, bias_stats = analyze_robomaster_trajectory(df)
    
    # Create plots
    plot_robomaster_analysis(df, args.output)
    
    # Generate report
    generate_report(df, pos_stats, vel_stats, theta_stats, bias_stats, args.output)
    
    print(f"\n✅ RoboMaster EKF analysis complete!")
    print(f"📁 Results saved in: {args.output}/")

if __name__ == "__main__":
    main()
