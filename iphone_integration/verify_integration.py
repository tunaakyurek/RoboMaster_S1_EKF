#!/usr/bin/env python3
"""
Verification Script for iPhone-EKF Integration
==============================================
This script verifies that all modules are properly connected
and can communicate with each other.

Author: RoboMaster EKF Integration System
Date: 2025
"""

import sys
import os
import importlib
import traceback

def verify_imports():
    """Verify all modules can be imported"""
    print("=" * 60)
    print("VERIFYING MODULE IMPORTS")
    print("=" * 60)
    
    modules_to_test = [
        ('pi_phone_connection.iphone_sensor_receiver', 
         ['iPhoneSensorData', 'iPhoneDataReceiver', 'iPhoneDataProcessor']),
        ('pi_phone_connection.ekf_8dof_formulary', 
         ['EKF8DOF', 'EKF8State']),
        ('pi_phone_connection.main_integration', 
         ['iPhoneEKFIntegration']),
        ('robomaster_control.autonomous_controller', 
         ['AutonomousController', 'ControlMode', 'Waypoint', 'MissionPlanner']),
        ('offline_analysis.python.ekf_analysis', 
         ['EKFDataAnalyzer']),
    ]
    
    results = []
    for module_name, classes in modules_to_test:
        try:
            # Add parent directory to path
            parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            if parent_dir not in sys.path:
                sys.path.insert(0, parent_dir)
            
            # Import module
            module = importlib.import_module(f'iphone_integration.{module_name}')
            
            # Check classes
            missing_classes = []
            for class_name in classes:
                if not hasattr(module, class_name):
                    missing_classes.append(class_name)
            
            if missing_classes:
                results.append((module_name, False, f"Missing: {missing_classes}"))
                print(f"❌ {module_name}: Missing classes: {missing_classes}")
            else:
                results.append((module_name, True, "OK"))
                print(f"✅ {module_name}: All classes found")
                
        except ImportError as e:
            results.append((module_name, False, str(e)))
            print(f"❌ {module_name}: Import failed - {e}")
        except Exception as e:
            results.append((module_name, False, str(e)))
            print(f"❌ {module_name}: Error - {e}")
    
    return results

def verify_connections():
    """Verify modules can work together"""
    print("\n" + "=" * 60)
    print("VERIFYING MODULE CONNECTIONS")
    print("=" * 60)
    
    # Add paths
    parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    sys.path.insert(0, parent_dir)
    sys.path.append(os.path.join(parent_dir, 'iphone_integration', 'pi_phone_connection'))
    sys.path.append(os.path.join(parent_dir, 'iphone_integration', 'robomaster_control'))
    
    tests = []
    
    # Test 1: Create EKF8State and verify conversion
    try:
        from iphone_integration.pi_phone_connection.ekf_8dof_formulary import EKF8State
        state = EKF8State(x=1.0, y=2.0, z=3.0, vz=0.5, 
                         roll=0.1, pitch=0.2, yaw=0.3, yaw_rate=0.1)
        arr = state.to_array()
        state2 = EKF8State.from_array(arr)
        assert abs(state.x - state2.x) < 1e-6
        tests.append(("EKF8State conversion", True, "OK"))
        print("✅ EKF8State conversion: Working")
    except Exception as e:
        tests.append(("EKF8State conversion", False, str(e)))
        print(f"❌ EKF8State conversion: {e}")
    
    # Test 2: Create EKF and run prediction
    try:
        from iphone_integration.pi_phone_connection.ekf_8dof_formulary import EKF8DOF
        import numpy as np
        ekf = EKF8DOF()
        ekf.predict(0.02)
        ekf.update_imu(np.array([0, 0, 9.81]), np.array([0, 0, 0]))
        state = ekf.get_state()
        tests.append(("EKF8DOF operations", True, "OK"))
        print("✅ EKF8DOF operations: Working")
    except Exception as e:
        tests.append(("EKF8DOF operations", False, str(e)))
        print(f"❌ EKF8DOF operations: {e}")
    
    # Test 3: Create sensor data object
    try:
        from iphone_integration.pi_phone_connection.iphone_sensor_receiver import iPhoneSensorData
        import time
        sensor_data = iPhoneSensorData(
            timestamp=time.time(),
            accel_x=0.1, accel_y=0.2, accel_z=9.81,
            gyro_x=0.01, gyro_y=0.02, gyro_z=0.03
        )
        ekf_format = sensor_data.to_ekf_format()
        assert 'imu' in ekf_format
        tests.append(("iPhoneSensorData", True, "OK"))
        print("✅ iPhoneSensorData: Working")
    except Exception as e:
        tests.append(("iPhoneSensorData", False, str(e)))
        print(f"❌ iPhoneSensorData: {e}")
    
    # Test 4: Autonomous controller with EKF8State
    try:
        from iphone_integration.robomaster_control.autonomous_controller import AutonomousController
        from iphone_integration.pi_phone_connection.ekf_8dof_formulary import EKF8State
        
        controller = AutonomousController()
        state = EKF8State(x=1.0, y=2.0, z=3.0, vz=0.5,
                         roll=0.1, pitch=0.2, yaw=0.3, yaw_rate=0.1)
        controller.update_state(state)  # Should handle EKF8State object
        tests.append(("AutonomousController with EKF8State", True, "OK"))
        print("✅ AutonomousController with EKF8State: Working")
    except Exception as e:
        tests.append(("AutonomousController with EKF8State", False, str(e)))
        print(f"❌ AutonomousController with EKF8State: {e}")
    
    # Test 5: Main integration module
    try:
        from iphone_integration.pi_phone_connection.main_integration import iPhoneEKFIntegration
        integration = iPhoneEKFIntegration()
        state = integration.get_current_state()
        stats = integration.get_statistics()
        tests.append(("iPhoneEKFIntegration", True, "OK"))
        print("✅ iPhoneEKFIntegration: Working")
    except Exception as e:
        tests.append(("iPhoneEKFIntegration", False, str(e)))
        print(f"❌ iPhoneEKFIntegration: {e}")
    
    return tests

def verify_dependencies():
    """Verify required Python packages are installed"""
    print("\n" + "=" * 60)
    print("VERIFYING DEPENDENCIES")
    print("=" * 60)
    
    required_packages = {
        'numpy': '1.24.3',
        'scipy': '1.10.1',
        'matplotlib': '3.7.1',
        'pandas': '2.0.2'
    }
    
    results = []
    for package, required_version in required_packages.items():
        try:
            module = importlib.import_module(package)
            version = getattr(module, '__version__', 'unknown')
            
            # Check version compatibility
            if version.startswith(required_version.split('.')[0]):
                results.append((package, True, f"v{version}"))
                print(f"✅ {package}: v{version} (required: {required_version})")
            else:
                results.append((package, True, f"v{version} (required: {required_version})"))
                print(f"⚠️  {package}: v{version} (required: {required_version})")
                
        except ImportError:
            results.append((package, False, "Not installed"))
            print(f"❌ {package}: Not installed (required: {required_version})")
    
    return results

def main():
    """Main verification routine"""
    print("\n" + "=" * 60)
    print("iPhone-EKF Integration Verification Script")
    print("Python Version:", sys.version)
    print("=" * 60)
    
    all_passed = True
    
    # Verify dependencies
    dep_results = verify_dependencies()
    if any(not r[1] for r in dep_results):
        all_passed = False
        print("\n⚠️  Some dependencies are missing. Install with:")
        print("    pip install -r requirements_iphone.txt")
    
    # Verify imports
    import_results = verify_imports()
    if any(not r[1] for r in import_results):
        all_passed = False
    
    # Verify connections
    connection_results = verify_connections()
    if any(not r[1] for r in connection_results):
        all_passed = False
    
    # Summary
    print("\n" + "=" * 60)
    print("VERIFICATION SUMMARY")
    print("=" * 60)
    
    total_tests = len(dep_results) + len(import_results) + len(connection_results)
    passed_tests = sum(1 for r in dep_results + import_results + connection_results if r[1])
    
    print(f"Total tests: {total_tests}")
    print(f"Passed: {passed_tests}")
    print(f"Failed: {total_tests - passed_tests}")
    
    if all_passed:
        print("\n✅ All verifications passed! The system is properly integrated.")
    else:
        print("\n❌ Some verifications failed. Please check the errors above.")
        print("\nCommon fixes:")
        print("1. Install dependencies: pip install -r requirements_iphone.txt")
        print("2. Ensure you're in the correct directory")
        print("3. Check Python version (3.11.2 recommended)")
    
    return 0 if all_passed else 1

if __name__ == "__main__":
    exit(main())
