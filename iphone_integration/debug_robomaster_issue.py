#!/usr/bin/env python3
"""
Debug RoboMaster Integration Issue
==================================
Minimal debug script to identify why EKF processing is not working
"""

import sys
import os
sys.path.append(os.path.dirname(__file__))

from pi_phone_connection.iphone_sensor_receiver import iPhoneDataReceiver, iPhoneDataProcessor
import time
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def debug_callback(data):
    """Debug callback to see what data we're getting"""
    print(f"\n=== RAW DATA ===")
    print(f"Timestamp: {data.timestamp}")
    print(f"Accel: [{data.accel_x:.3f}, {data.accel_y:.3f}, {data.accel_z:.3f}]")
    print(f"Gyro: [{data.gyro_x:.3f}, {data.gyro_y:.3f}, {data.gyro_z:.3f}]")
    print(f"Has GPS: lat={getattr(data, 'lat', 'None')}, lon={getattr(data, 'lon', 'None')}")
    
    # Process data
    processor = iPhoneDataProcessor()
    processed = processor.process(data)
    
    print(f"\n=== PROCESSED DATA ===")
    print(f"Keys: {list(processed.keys())}")
    if 'accel' in processed:
        print(f"Processed accel: {processed['accel']}")
    if 'gyro' in processed:
        print(f"Processed gyro: {processed['gyro']}")
    if 'gps' in processed:
        print(f"Processed GPS: {processed['gps']}")
    
    print("=" * 50)

def main():
    print("🔍 Debug RoboMaster Integration Issue")
    print("Will show first 10 data samples...")
    
    receiver = iPhoneDataReceiver(port=5555)
    
    sample_count = 0
    max_samples = 10
    
    def limited_callback(data):
        nonlocal sample_count
        if sample_count < max_samples:
            debug_callback(data)
            sample_count += 1
        elif sample_count == max_samples:
            print(f"\n✅ Captured {max_samples} samples for debugging")
            print("🛑 Stopping debug session...")
            receiver.stop()
            sample_count += 1
    
    try:
        receiver.start(callback=limited_callback)
        
        print("📱 Start iPhone sensor streaming...")
        
        # Wait for samples
        while sample_count <= max_samples and receiver.is_running:
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\n🛑 Stopping debug...")
    finally:
        receiver.stop()

if __name__ == "__main__":
    main()
