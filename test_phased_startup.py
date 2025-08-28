#!/usr/bin/env python3
"""
Test script for RoboMaster phased startup logic
"""

import time
import sys
import os

# Add the iphone_integration path to sys.path
sys.path.append(os.path.join(os.path.dirname(__file__), 'iphone_integration', 'pi_phone_connection'))

from main_integration_robomaster import RoboMasterEKFIntegration
from iphone_sensor_receiver import iPhoneSensorData

def create_test_sensor_data(timestamp, accel_x=0.0, accel_y=0.0, accel_z=9.81, 
                           gyro_x=0.0, gyro_y=0.0, gyro_z=0.0):
    """Create test sensor data"""
    return iPhoneSensorData(
        timestamp=timestamp,
        accel_x=accel_x, accel_y=accel_y, accel_z=accel_z,
        gyro_x=gyro_x, gyro_y=gyro_y, gyro_z=gyro_z,
        mag_x=0.0, mag_y=0.0, mag_z=0.0,
        gps_lat=None, gps_lon=None, gps_alt=None,
        gps_accuracy=None, gps_speed=None, gps_course=None,
        pressure=None, altitude=None, roll=0.0, pitch=0.0, yaw=0.0
    )

def test_phased_startup():
    """Test the phased startup logic"""
    print("Testing RoboMaster phased startup logic...")
    
    # Create integration system with test config
    config = {
        'calibration': {
            'auto_calibrate': True,
            'pre_start_delay_seconds': 2.0,  # 2 seconds for testing
            'duration': 3.0,  # 3 seconds for testing
            'min_samples': 10
        },
        'logging': {
            'enabled': False  # Disable logging for testing
        }
    }
    
    integration = RoboMasterEKFIntegration()
    integration.config = config
    
    # Start the system
    print("Starting integration system...")
    integration.start()
    
    # Simulate sensor data flow
    start_time = time.time()
    current_time = start_time
    
    print(f"System start time: {start_time}")
    print(f"Pre-delay duration: {integration.pre_start_delay_seconds}s")
    print(f"Calibration duration: {integration._calibration_duration}s")
    
    # Phase 1: Pre-delay (should not process data)
    print("\n=== Phase 1: Pre-delay ===")
    for i in range(5):
        current_time = start_time + i * 0.5
        test_data = create_test_sensor_data(current_time)
        
        print(f"  {i+1}: t={current_time-start_time:.1f}s - Processing data...")
        integration._sensor_data_callback(test_data)
        
        print(f"    System start time: {integration.system_start_time}")
        print(f"    Is calibrated: {integration.is_calibrated}")
        print(f"    Calibration data count: {len(integration.calibration_data)}")
        print(f"    EKF thread: {integration.ekf_thread is not None}")
        
        time.sleep(0.1)
    
    # Phase 2: Calibration (should collect data)
    print("\n=== Phase 2: Calibration ===")
    for i in range(10):
        current_time = start_time + 2.1 + i * 0.3  # Start slightly after pre-delay
        test_data = create_test_sensor_data(current_time)
        
        print(f"  {i+1}: t={current_time-start_time:.1f}s - Processing data...")
        integration._sensor_data_callback(test_data)
        
        print(f"    System start time: {integration.system_start_time}")
        print(f"    Is calibrated: {integration.is_calibrated}")
        print(f"    Calibration data count: {len(integration.calibration_data)}")
        print(f"    EKF thread: {integration.ekf_thread is not None}")
        
        time.sleep(0.1)
    
    # Phase 3: EKF running (should process data normally)
    print("\n=== Phase 3: EKF Running ===")
    for i in range(5):
        current_time = start_time + 5.0 + i * 0.5  # Start after calibration
        test_data = create_test_sensor_data(current_time, accel_x=0.1, accel_y=0.1)
        
        print(f"  {i+1}: t={current_time-start_time:.1f}s - Processing data...")
        integration._sensor_data_callback(test_data)
        
        print(f"    System start time: {integration.system_start_time}")
        print(f"    Is calibrated: {integration.is_calibrated}")
        print(f"    Calibration data count: {len(integration.calibration_data)}")
        print(f"    EKF thread: {integration.ekf_thread is not None}")
        
        time.sleep(0.1)
    
    # Stop the system
    print("\nStopping integration system...")
    integration.stop()
    
    print("\nTest completed!")

if __name__ == "__main__":
    test_phased_startup()
