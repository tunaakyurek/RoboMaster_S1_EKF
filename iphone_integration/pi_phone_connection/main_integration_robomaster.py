"""
RoboMaster iPhone-EKF Integration System
========================================
Main integration using EXACT RoboMaster 8-DOF EKF from formulary
State vector: [x, y, theta, vx, vy, bias_accel_x, bias_accel_y, bias_angular_velocity]

This is the official implementation following RoboMaster_Formulary.pdf
Author: RoboMaster EKF Integration System
Date: 2025
"""

import json
import logging
import time
import threading
import queue
import os
import sys
from datetime import datetime
from typing import Optional, Dict, Any
import numpy as np

# Configure logging
logging.basicConfig(
    level=logging.INFO,  # Use INFO level for cleaner output
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# Import our modules
try:
    from .iphone_sensor_receiver import iPhoneDataReceiver, iPhoneDataProcessor, iPhoneSensorData
    from .ekf_robomaster_8dof import RoboMasterEKF8DOF, RoboMasterState
except ImportError:
    from iphone_sensor_receiver import iPhoneDataReceiver, iPhoneDataProcessor, iPhoneSensorData
    from ekf_robomaster_8dof import RoboMasterEKF8DOF, RoboMasterState

# Optional autonomous controller
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'robomaster_control'))
try:
    from autonomous_controller import AutonomousController, ControlMode, Waypoint
    AUTONOMOUS_AVAILABLE = True
except ImportError:
    AUTONOMOUS_AVAILABLE = False
    logger.warning("Autonomous controller not available")


class RoboMasterEKFIntegration:
    """
    RoboMaster iPhone-EKF integration using exact formulary implementation
    """
    
    def __init__(self, config_file: Optional[str] = None):
        """
        Initialize the RoboMaster integration system
        
        Args:
            config_file: Path to configuration file
        """
        # Load configuration
        self.config = self._load_config(config_file)
        
        # Initialize sensor components
        self.receiver = iPhoneDataReceiver(
            connection_type=self.config.get('connection_type', 'udp'),
            port=self.config.get('port', 5555)
        )
        
        self.processor = iPhoneDataProcessor()
        
        # Initialize RoboMaster 8-DOF EKF
        self.ekf = RoboMasterEKF8DOF(self.config.get('ekf_config', {}))
        
        # Optional autonomous controller
        self.autonomous_controller = None
        if AUTONOMOUS_AVAILABLE and self.config.get('enable_autonomous', False):
            self.autonomous_controller = AutonomousController(
                self.config.get('autonomous_config', {})
            )
            logger.info("Autonomous controller initialized for RoboMaster EKF")
        
        # Data logging (save in iphone_integration/data/)
        self.log_dir = self.config.get('log_dir', './data')
        self.log_file = None
        self.log_writer = None
        
        # State publishing
        self.state_queue = queue.Queue(maxsize=100)
        self.state_callback = None
        
        # Control flags
        self.is_running = False
        self.ekf_thread = None
        self.log_thread = None
        
        # Calibration
        self.is_calibrated = False
        self.calibration_data = []
        self.calibration_start_time = None
        
        # Statistics
        self.stats = {
            'ekf_updates': 0,
            'packets_processed': 0,
            'start_time': None,
            'last_update_time': None
        }
        
        # GPS reference for coordinate conversion
        self.gps_reference = {
            'lat': None,
            'lon': None,
            'set': False
        }
        
        logger.info("RoboMaster iPhone-EKF integration initialized (exact formulary)")
    
    def _load_config(self, config_file: Optional[str]) -> Dict[str, Any]:
        """Load configuration from file or use RoboMaster defaults"""
        default_config = {
            'connection_type': 'udp',
            'port': 5555,
            'ekf_config': {
                # Process noise (from formulary)
                'q_position': 0.01,
                'q_theta': 0.01,
                'q_velocity': 0.1,
                'q_accel_bias': 1e-6,
                'q_gyro_bias': 1e-8,
                
                # Measurement noise (from formulary)
                'r_accel': 0.1,
                'r_gyro': 0.01,
                'r_gps_pos': 1.0,
                'r_gps_vel': 0.5,
                
                # Initial uncertainties
                'init_pos_var': 1.0,
                'init_theta_var': 0.1,
                'init_vel_var': 0.5,
                'init_accel_bias_var': 0.01,
                'init_gyro_bias_var': 0.001
            },
            'calibration': {
                'duration': 5.0,
                'min_samples': 100,
                'auto_calibrate': True
            },
            'logging': {
                'enabled': True,
                'rate': 50  # Hz - matching EKF rate
            }
        }
        
        if config_file and os.path.exists(config_file):
            try:
                with open(config_file, 'r') as f:
                    config = json.load(f)
                # Merge with defaults
                for key, value in default_config.items():
                    if key not in config:
                        config[key] = value
                    elif isinstance(value, dict):
                        for subkey, subvalue in value.items():
                            if subkey not in config[key]:
                                config[key][subkey] = subvalue
                return config
            except Exception as e:
                logger.warning(f"Failed to load config file: {e}")
        
        return default_config
    
    def calibrate(self):
        """Perform sensor calibration for bias estimation"""
        calibration_config = self.config.get('calibration', {})
        duration = calibration_config.get('duration', 5.0)
        min_samples = calibration_config.get('min_samples', 100)
        
        logger.info(f"✅ Starting {duration}s calibration - keep device STATIONARY")
        logger.info("✅ This will estimate accelerometer and gyroscope biases")
        
        self.calibration_data = []
        self.calibration_start_time = time.time()
        
        # Start receiver for calibration
        self.receiver.start(callback=self._calibration_callback)
        
        # Wait for calibration
        while (time.time() - self.calibration_start_time) < duration:
            time.sleep(0.1)
            if len(self.calibration_data) >= min_samples:
                logger.info(f"✅ Collected {len(self.calibration_data)} calibration samples")
        
        # Stop receiver temporarily
        self.receiver.stop()
        
        # Small delay to ensure clean shutdown before restart
        time.sleep(0.5)
        
        # Process calibration data
        if len(self.calibration_data) >= min_samples:
            self._process_calibration_data()
            self.is_calibrated = True
            logger.info("✅ Calibration complete - biases estimated")
        else:
            logger.error(f"❌ Insufficient calibration data: {len(self.calibration_data)} < {min_samples}")
            raise RuntimeError("Calibration failed")
    
    def _calibration_callback(self, data: iPhoneSensorData):
        """Callback for calibration data collection"""
        if self.calibration_start_time and (time.time() - self.calibration_start_time) < 10.0:
            self.calibration_data.append(data)
    
    def _process_calibration_data(self):
        """Process calibration data to estimate initial biases"""
        if not self.calibration_data:
            return
        
        # Calculate sensor biases from stationary data
        accel_x_samples = [d.accel_x for d in self.calibration_data]
        accel_y_samples = [d.accel_y for d in self.calibration_data]
        gyro_z_samples = [d.gyro_z for d in self.calibration_data]
        
        # Estimate biases (assuming stationary on level ground)
        bias_accel_x = np.mean(accel_x_samples)  # Should be ~0 for level ground
        bias_accel_y = np.mean(accel_y_samples)  # Should be ~0 for level ground
        bias_gyro_z = np.mean(gyro_z_samples)   # Should be ~0 when stationary
        
        # Set initial state with estimated biases
        initial_state = RoboMasterState(
            x=0.0, y=0.0, theta=0.0,
            vx=0.0, vy=0.0,
            bias_accel_x=bias_accel_x,
            bias_accel_y=bias_accel_y,
            bias_angular_velocity=bias_gyro_z
        )
        
        self.ekf.reset(initial_state)
        
        # Also update processor calibration for sensor preprocessing
        self.processor.calibrate(self.calibration_data, len(self.calibration_data) * 0.02)
        
        logger.info(f"✅ Estimated biases: accel=[{bias_accel_x:.3f}, {bias_accel_y:.3f}], gyro={bias_gyro_z:.3f}")
    
    def start(self, state_callback: Optional[callable] = None):
        """Start the RoboMaster integration system"""
        if not self.is_calibrated and self.config.get('calibration', {}).get('auto_calibrate', True):
            self.calibrate()
        
        self.state_callback = state_callback
        self.is_running = True
        self.stats['start_time'] = time.time()
        
        # Create log file
        if self.config.get('logging', {}).get('enabled', True):
            self._create_log_file()
        
        # Start receiver
        self.receiver.start(callback=self._sensor_data_callback)
        
        # Start EKF processing thread
        self.ekf_thread = threading.Thread(target=self._ekf_processing_loop)
        self.ekf_thread.daemon = True
        self.ekf_thread.start()
        
        # Start logging thread
        if self.log_file:
            self.log_thread = threading.Thread(target=self._logging_loop)
            self.log_thread.daemon = True
            self.log_thread.start()
        
        logger.info("RoboMaster integration system started")
    
    def stop(self):
        """Stop the integration system"""
        self.is_running = False
        
        # Stop receiver
        self.receiver.stop()
        
        # Wait for threads
        if self.ekf_thread:
            self.ekf_thread.join(timeout=2)
        if self.log_thread:
            self.log_thread.join(timeout=2)
        
        # Close log file
        if self.log_writer:
            self.log_writer.close()
        
        # Print final statistics
        self._print_final_stats()
        
        logger.info("RoboMaster integration system stopped")
    
    def _sensor_data_callback(self, data: iPhoneSensorData):
        """Process incoming sensor data"""
        if not self.is_running:
            return
        
        # Process raw data
        try:
            processed_data = self.processor.process(data)
            # logger.debug(f"Processed data keys: {list(processed_data.keys())}")  # Commented for cleaner output
        except Exception as e:
            logger.error(f"Data processing failed: {e}")
            return
        
        # Add to queue for EKF processing
        if not self.state_queue.full():
            self.state_queue.put((data, processed_data))
            self.stats['packets_processed'] += 1
            # logger.debug(f"Added data to queue, packets_processed: {self.stats['packets_processed']}")  # Commented for cleaner output
        else:
            logger.warning("State queue full, dropping data")
    
    def _ekf_processing_loop(self):
        """Main EKF processing loop following RoboMaster formulary"""
        last_time = time.time()
        logger.info("EKF processing loop started")
        
        while self.is_running:
            try:
                # Get sensor data
                # logger.debug(f"Waiting for data from queue (size: {self.state_queue.qsize()})")  # Commented for cleaner output
                raw_data, processed_data = self.state_queue.get(timeout=0.1)
                # logger.debug("Got data from queue")  # Commented for cleaner output
                current_time = time.time()
                dt = current_time - last_time
                
                # Prepare control input for RoboMaster EKF
                control_input = self._prepare_control_input(raw_data, processed_data)
                
                # Prediction step
                try:
                    self.ekf.predict(dt, control_input)
                    # logger.debug(f"Prediction successful, dt={dt:.3f}, control={control_input}")  # Commented for cleaner output
                except Exception as e:
                    logger.error(f"Prediction failed: {e}")
                    continue
                
                # Update step with IMU
                try:
                    self._update_with_imu(raw_data, processed_data)
                    # logger.debug("IMU update successful")  # Commented for cleaner output
                except Exception as e:
                    logger.error(f"IMU update failed: {e}")
                
                # Update with GPS if available
                try:
                    self._update_with_gps(raw_data, processed_data)
                except Exception as e:
                    logger.error(f"GPS update failed: {e}")
                
                # Get current state
                current_state = self.ekf.get_state()
                
                # Update autonomous controller if available
                if self.autonomous_controller and AUTONOMOUS_AVAILABLE:
                    self._update_autonomous_controller(current_state)
                
                # Call state callback
                if self.state_callback:
                    self.state_callback(current_state)
                
                # Update statistics
                self.stats['ekf_updates'] += 1
                self.stats['last_update_time'] = current_time
                last_time = current_time
                
                # Periodic status print
                if self.stats['ekf_updates'] % 100 == 0:
                    self._print_status(current_state)
            
            except queue.Empty:
                continue
            except Exception as e:
                logger.error(f"EKF processing error: {e}")
                import traceback
                traceback.print_exc()
    
    def _prepare_control_input(self, raw_data: iPhoneSensorData, processed_data: Dict) -> np.ndarray:
        """
        Prepare control input for RoboMaster EKF
        Format: [ax_body, ay_body, omega_z]
        """
        # Get accelerometer data (body frame)
        accel = processed_data.get('accel', [0.0, 0.0, 0.0])
        ax_body = accel[0]
        ay_body = accel[1]
        
        # Get gyroscope data (body frame) 
        gyro = processed_data.get('gyro', [0.0, 0.0, 0.0])
        omega_z = gyro[2]  # Yaw rate
        
        return np.array([ax_body, ay_body, omega_z])
    
    def _update_with_imu(self, raw_data: iPhoneSensorData, processed_data: Dict):
        """Update EKF with IMU measurements"""
        if 'accel' in processed_data and 'gyro' in processed_data:
            accel = np.array(processed_data['accel'])
            gyro = np.array(processed_data['gyro'])
            
            # Use first two accelerometer components and yaw gyro
            accel_body = accel[0:2]  # [ax, ay]
            gyro_z = gyro[2]         # omega_z
            
            self.ekf.update_imu(accel_body, gyro_z)
    
    def _update_with_gps(self, raw_data: iPhoneSensorData, processed_data: Dict):
        """Update EKF with GPS measurements"""
        if 'gps' in processed_data and processed_data['gps']:
            gps_data = processed_data['gps']
            if all(k in gps_data for k in ['lat', 'lon']):
                self._handle_gps_update(gps_data)
    
    def _handle_gps_update(self, gps_data: Dict):
        """Handle GPS updates with coordinate conversion"""
        lat, lon = gps_data['lat'], gps_data['lon']
        
        # Set reference point on first GPS fix
        if not self.gps_reference['set']:
            self.gps_reference['lat'] = lat
            self.gps_reference['lon'] = lon
            self.gps_reference['set'] = True
            logger.info(f"GPS reference set: {lat:.6f}, {lon:.6f}")
        
        # Convert to local coordinates
        lat_to_m = 111320.0
        lon_to_m = 111320.0 * np.cos(np.radians(self.gps_reference['lat']))
        
        x_gps = (lat - self.gps_reference['lat']) * lat_to_m
        y_gps = (lon - self.gps_reference['lon']) * lon_to_m
        
        gps_pos = np.array([x_gps, y_gps])
        
        # Update EKF with GPS position
        self.ekf.update_gps_position(gps_pos)
        
        # If GPS provides velocity, use it too
        if 'speed' in gps_data and 'course' in gps_data:
            speed = gps_data['speed']
            course_rad = np.radians(gps_data['course'])
            
            vx_gps = speed * np.cos(course_rad)
            vy_gps = speed * np.sin(course_rad)
            gps_vel = np.array([vx_gps, vy_gps])
            
            self.ekf.update_gps_velocity(gps_vel)
    
    def _update_autonomous_controller(self, state: RoboMasterState):
        """Update autonomous controller with current state"""
        if self.autonomous_controller:
            # Convert RoboMaster state to controller format
            state_dict = {
                'x': state.x,
                'y': state.y,
                'z': 0.0,  # Ground vehicle
                'yaw': state.theta,
                'vx': state.vx,
                'vy': state.vy
            }
            self.autonomous_controller.update_state(state_dict)
    
    def _create_log_file(self):
        """Create CSV log file for RoboMaster data"""
        os.makedirs(self.log_dir, exist_ok=True)
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"robomaster_ekf_log_{timestamp}.csv"
        self.log_file = os.path.join(self.log_dir, filename)
        
        # CSV header for RoboMaster 8-DOF state
        header = ("timestamp,x,y,theta,vx,vy,bias_accel_x,bias_accel_y,bias_angular_velocity,"
                 "accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,cov_trace\n")
        
        try:
            self.log_writer = open(self.log_file, 'w')
            self.log_writer.write(header)
            logger.info(f"RoboMaster log file created: {self.log_file}")
        except Exception as e:
            logger.error(f"Failed to create log file: {e}")
            self.log_file = None
    
    def _logging_loop(self):
        """Background logging loop"""
        log_rate = self.config.get('logging', {}).get('rate', 50)
        log_interval = 1.0 / log_rate
        last_log_time = 0
        
        while self.is_running and self.log_writer:
            current_time = time.time()
            
            if current_time - last_log_time >= log_interval:
                try:
                    self._write_log_entry()
                    last_log_time = current_time
                except Exception as e:
                    logger.error(f"Logging error: {e}")
            
            time.sleep(0.001)  # Small sleep to prevent busy waiting
    
    def _write_log_entry(self):
        """Write current RoboMaster state to log file"""
        if not self.log_writer:
            return
        
        try:
            state = self.ekf.get_state()
            cov_trace = np.trace(self.ekf.get_covariance())
            latest_data = self.receiver.get_latest_data()
            
            if latest_data:
                # Get sensor data
                accel = [latest_data.accel_x, latest_data.accel_y, latest_data.accel_z]
                gyro = [latest_data.gyro_x, latest_data.gyro_y, latest_data.gyro_z]
                
                # Format log entry for RoboMaster state
                log_entry = (f"{time.time()},{state.x},{state.y},{state.theta},"
                           f"{state.vx},{state.vy},"
                           f"{state.bias_accel_x},{state.bias_accel_y},{state.bias_angular_velocity},"
                           f"{accel[0]},{accel[1]},{accel[2]},"
                           f"{gyro[0]},{gyro[1]},{gyro[2]},{cov_trace}\n")
                
                self.log_writer.write(log_entry)
                self.log_writer.flush()
        
        except Exception as e:
            logger.error(f"Log write error: {e}")
    
    def _print_status(self, state: RoboMasterState):
        """Print periodic status updates"""
        runtime = time.time() - self.stats['start_time']
        rate = self.stats['ekf_updates'] / runtime if runtime > 0 else 0
        
        logger.info(f"Stats: EKF updates={self.stats['ekf_updates']}, "
                   f"Packets={self.stats['packets_processed']}, Rate={rate:.1f} Hz")
        logger.info(f"State: {state}")
    
    def _print_final_stats(self):
        """Print final statistics"""
        runtime = time.time() - self.stats['start_time'] if self.stats['start_time'] else 0
        avg_rate = self.stats['ekf_updates'] / runtime if runtime > 0 else 0
        
        logger.info("Final RoboMaster Statistics:")
        logger.info(f"   Runtime: {runtime:.1f} seconds")
        logger.info(f"   EKF updates: {self.stats['ekf_updates']}")
        logger.info(f"   Packets processed: {self.stats['packets_processed']}")
        logger.info(f"   Average rate: {avg_rate:.1f} Hz")
    
    def get_current_state(self) -> RoboMasterState:
        """Get current RoboMaster EKF state"""
        return self.ekf.get_state()
    
    def get_statistics(self) -> Dict[str, Any]:
        """Get system statistics"""
        runtime = time.time() - self.stats['start_time'] if self.stats['start_time'] else 0
        
        return {
            'runtime': runtime,
            'ekf_updates': self.stats['ekf_updates'],
            'packets_processed': self.stats['packets_processed'],
            'average_rate': self.stats['ekf_updates'] / runtime if runtime > 0 else 0,
            'is_calibrated': self.is_calibrated,
            'ekf_stats': self.ekf.get_statistics()
        }


def main():
    """Main function for RoboMaster integration"""
    import argparse
    
    parser = argparse.ArgumentParser(description='RoboMaster iPhone-EKF Integration')
    parser.add_argument('--config', help='Configuration file path')
    parser.add_argument('--no-calibrate', action='store_true', help='Skip auto-calibration')
    
    args = parser.parse_args()
    
    def state_callback(state: RoboMasterState):
        """State callback for printing"""
        print(f"State: {state}")
    
    # Create RoboMaster integration system
    integration = RoboMasterEKFIntegration(args.config)
    
    if args.no_calibrate:
        integration.config['calibration']['auto_calibrate'] = False
        integration.is_calibrated = True
    
    try:
        # Start system
        integration.start(state_callback)
        
        logger.info("RoboMaster system running. Press Ctrl+C to stop...")
        logger.info("Start iPhone sensor streaming to this device")
        
        while True:
            time.sleep(1)
    
    except KeyboardInterrupt:
        logger.info("Stopping...")
        integration.stop()


if __name__ == "__main__":
    main()
