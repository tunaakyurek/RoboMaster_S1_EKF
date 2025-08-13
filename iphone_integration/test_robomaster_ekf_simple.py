#!/usr/bin/env python3
"""
Simple RoboMaster EKF Test
==========================
Test the RoboMaster EKF with real sensor data from debug output
"""

import sys
import os
sys.path.append(os.path.dirname(__file__))

import numpy as np
from pi_phone_connection.ekf_robomaster_8dof import RoboMasterEKF8DOF, RoboMasterState

def test_with_real_data():
    """Test EKF with the exact sensor data from your iPhone"""
    print("🧪 Testing RoboMaster EKF with real iPhone sensor data...")
    
    # Create EKF
    ekf = RoboMasterEKF8DOF()
    
    # Real sensor data from your debug output
    sensor_samples = [
        {'accel': [-0.04745097, -0.19728891, -9.930496230000001], 'gyro': [-0.01818, -0.044191, 0.003759]},
        {'accel': [-0.032186610000000004, -0.20417553000000002, -9.85699971], 'gyro': [0.007608, 0.011885, 0.005069]},
        {'accel': [0.01047708, -0.21704625, -9.6944382], 'gyro': [0.015089, 0.020119, 0.002816]},
        {'accel': [-0.048353490000000006, -0.21135645000000003, -9.91223982], 'gyro': [-0.013738, -0.029818, 0.004932]},
        {'accel': [-0.03248091, -0.2026746, -9.844727400000002], 'gyro': [0.009375, 0.010819, 0.004882]},
    ]
    
    print(f"Initial state: {ekf.get_state()}")
    
    dt = 0.02  # 50 Hz
    
    for i, sample in enumerate(sensor_samples):
        try:
            print(f"\n--- Sample {i+1} ---")
            
            # Prepare control input [ax, ay, omega_z]
            accel = np.array(sample['accel'])
            gyro = np.array(sample['gyro'])
            control_input = np.array([accel[0], accel[1], gyro[2]])
            
            print(f"Control input: {control_input}")
            
            # Prediction step
            print("Calling predict...")
            ekf.predict(dt, control_input)
            print("✅ Predict successful")
            
            # IMU update
            print("Calling IMU update...")
            accel_body = accel[0:2]  # [ax, ay]
            gyro_z = gyro[2]         # omega_z
            ekf.update_imu(accel_body, gyro_z)
            print("✅ IMU update successful")
            
            # Get state
            state = ekf.get_state()
            print(f"State: {state}")
            
            # Check for any NaN or inf values
            state_array = state.to_array()
            if np.any(np.isnan(state_array)) or np.any(np.isinf(state_array)):
                print("❌ WARNING: NaN or Inf detected in state!")
                break
                
        except Exception as e:
            print(f"❌ Error at sample {i+1}: {e}")
            import traceback
            traceback.print_exc()
            break
    
    print(f"\nFinal state: {ekf.get_state()}")
    print(f"Final stats: {ekf.get_statistics()}")

if __name__ == "__main__":
    test_with_real_data()
