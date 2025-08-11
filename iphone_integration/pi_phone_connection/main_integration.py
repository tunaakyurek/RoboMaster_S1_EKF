"""
Main Integration Module for iPhone-EKF System
==============================================
Combines iPhone sensor data reception with 8-DOF EKF processing
Follows RoboMaster EKF Formulary specifications

This module orchestrates the complete pipeline:
1. Receives iPhone sensor data
2. Processes through 8-DOF EKF
3. Logs data for offline analysis
4. Provides real-time state estimates

Author: RoboMaster EKF Integration System
Date: 2025
"""

import json
import logging
import time
import threading
import queue
import os
from datetime import datetime
from typing import Optional, Dict, Any
import numpy as np

# Import our modules
from iphone_sensor_receiver import iPhoneDataReceiver, iPhoneDataProcessor, iPhoneSensorData
from ekf_8dof_formulary import EKF8DOF, EKF8State

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


class iPhoneEKFIntegration:
    """
    Main integration class for iPhone sensor data and EKF processing
    """
    
    def __init__(self, config_file: Optional[str] = None):
        """
        Initialize the integration system
        
        Args:
            config_file: Path to configuration file
        """
        # Load configuration
        self.config = self._load_config(config_file)
        
        # Initialize components
        self.receiver = iPhoneDataReceiver(
            connection_type=self.config.get('connection_type', 'udp'),
            port=self.config.get('port', 5555)
        )
        
        self.processor = iPhoneDataProcessor()
        
        self.ekf = EKF8DOF(self.config.get('ekf_config', {}))
        
        # Data logging
        self.log_dir = self.config.get('log_dir', '../data')
        self.log_file = None
        self.log_writer = None
        
        # State publishing
        self.state_queue = queue.Queue(maxsize=100)
        self.state_callback = None
        
        # Control flags
        self.is_running = False
        self.ekf_thread = None
        self.log_thread = None
        
        # Timing
        self.ekf_rate = self.config.get('ekf_rate', 50)  # Hz
        self.log_rate = self.config.get('log_rate', 10)  # Hz
        
        # Calibration
        self.is_calibrated = False
        self.calibration_samples = []
        self.calibration_duration = self.config.get('calibration_duration', 5.0)
        
        # GPS reference
        self.gps_reference = None
        
        # Statistics
        self.stats = {
            'ekf_updates': 0,
            'sensor_packets': 0,
            'processing_time_avg': 0,
            'last_update_time': 0
        }
        
        logger.info("iPhone-EKF Integration initialized")
    
    def _load_config(self, config_file: Optional[str]) -> Dict[str, Any]:
        """Load configuration from file or use defaults"""
        default_config = {
            'connection_type': 'udp',
            'port': 5555,
            'ekf_rate': 50,
            'log_rate': 10,
            'log_dir': '../data',
            'calibration_duration': 5.0,
            'ekf_config': {
                'q_position': 0.01,
                'q_velocity': 0.1,
                'q_orientation': 0.05,
                'q_angular_velocity': 0.1,
                'use_drone_velocity': False
            }
        }
        
        if config_file and os.path.exists(config_file):
            try:
                with open(config_file, 'r') as f:
                    loaded_config = json.load(f)
                    default_config.update(loaded_config)
                    logger.info(f"Configuration loaded from {config_file}")
            except Exception as e:
                logger.error(f"Failed to load config: {e}")
        
        return default_config
    
    def calibrate(self):
        """
        Perform sensor calibration
        Collects static data for bias estimation
        """
        logger.info(f"Starting calibration for {self.calibration_duration} seconds...")
        logger.info("Keep the device stationary!")
        
        self.calibration_samples = []
        calibration_start = time.time()
        
        # Start receiver temporarily for calibration
        self.receiver.start()
        
        while time.time() - calibration_start < self.calibration_duration:
            # Get data from receiver
            try:
                data = self.receiver.get_data_queue().get(timeout=0.1)
                self.calibration_samples.append(data)
            except queue.Empty:
                continue
            
            # Show progress
            elapsed = time.time() - calibration_start
            progress = elapsed / self.calibration_duration * 100
            if int(progress) % 20 == 0:
                logger.info(f"Calibration progress: {progress:.0f}%")
        
        # Stop receiver
        self.receiver.stop()
        
        # Perform calibration
        if self.calibration_samples:
            self.processor.calibrate(self.calibration_samples)
            self.is_calibrated = True
            logger.info(f"Calibration complete with {len(self.calibration_samples)} samples")
        else:
            logger.error("No calibration data collected!")
    
    def start(self, state_callback: Optional[callable] = None):
        """
        Start the integration system
        
        Args:
            state_callback: Optional callback for state updates
        """
        if self.is_running:
            logger.warning("System already running")
            return
        
        # Check calibration
        if not self.is_calibrated:
            logger.warning("System not calibrated. Running calibration...")
            self.calibrate()
        
        self.is_running = True
        self.state_callback = state_callback
        
        # Create log file
        self._create_log_file()
        
        # Start receiver with callback
        self.receiver.start(callback=self._sensor_data_callback)
        
        # Start EKF processing thread
        self.ekf_thread = threading.Thread(target=self._ekf_processing_loop)
        self.ekf_thread.daemon = True
        self.ekf_thread.start()
        
        # Start logging thread
        self.log_thread = threading.Thread(target=self._logging_loop)
        self.log_thread.daemon = True
        self.log_thread.start()
        
        logger.info("iPhone-EKF Integration started")
    
    def stop(self):
        """Stop the integration system"""
        logger.info("Stopping iPhone-EKF Integration...")
        
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
        
        logger.info("iPhone-EKF Integration stopped")
    
    def _sensor_data_callback(self, data: iPhoneSensorData):
        """Callback for new sensor data"""
        self.stats['sensor_packets'] += 1
        # Data will be processed in EKF thread
    
    def _ekf_processing_loop(self):
        """Main EKF processing loop"""
        dt = 1.0 / self.ekf_rate
        last_time = time.time()
        
        while self.is_running:
            try:
                current_time = time.time()
                
                # Get latest sensor data
                sensor_data = self.receiver.get_latest_data()
                
                if sensor_data:
                    # Process sensor data
                    processed = self.processor.process(sensor_data)
                    
                    # Time step
                    actual_dt = current_time - last_time
                    last_time = current_time
                    
                    # Measure processing time
                    start_process = time.time()
                    
                    # EKF prediction
                    self.ekf.predict(actual_dt)
                    
                    # EKF updates
                    if 'accel' in processed and 'gyro' in processed:
                        self.ekf.update_imu(
                            np.array(processed['accel']),
                            np.array(processed['gyro'])
                        )
                    
                    if 'mag' in processed and processed['mag']:
                        self.ekf.update_magnetometer(np.array(processed['mag']))
                    
                    if 'gps' in processed and processed['gps']:
                        self._handle_gps_update(processed['gps'])
                    
                    if 'baro' in processed and processed['baro']:
                        self.ekf.update_barometer(processed['baro']['altitude'])
                    
                    # Update processing time
                    process_time = time.time() - start_process
                    self.stats['processing_time_avg'] = (
                        0.9 * self.stats['processing_time_avg'] + 
                        0.1 * process_time
                    )
                    
                    # Get current state
                    state = self.ekf.get_state()
                    
                    # Store state
                    if not self.state_queue.full():
                        self.state_queue.put({
                            'timestamp': current_time,
                            'state': state,
                            'covariance': self.ekf.get_covariance(),
                            'raw_sensor': sensor_data
                        })
                    
                    # Call state callback
                    if self.state_callback:
                        self.state_callback(state)
                    
                    self.stats['ekf_updates'] += 1
                    self.stats['last_update_time'] = current_time
                
                # Sleep to maintain rate
                sleep_time = dt - (time.time() - current_time)
                if sleep_time > 0:
                    time.sleep(sleep_time)
            
            except Exception as e:
                logger.error(f"EKF processing error: {e}")
    
    def _handle_gps_update(self, gps_data: Dict):
        """Handle GPS update with reference management"""
        if not self.gps_reference:
            # Set first GPS reading as reference
            self.gps_reference = {
                'lat': gps_data['lat'],
                'lon': gps_data['lon']
            }
            logger.info(f"GPS reference set: {self.gps_reference}")
        
        self.ekf.update_gps(
            gps_data['lat'],
            gps_data['lon'],
            gps_data['alt'],
            self.gps_reference['lat'],
            self.gps_reference['lon']
        )
    
    def _logging_loop(self):
        """Data logging loop"""
        dt = 1.0 / self.log_rate
        
        while self.is_running:
            try:
                # Get state from queue
                if not self.state_queue.empty():
                    state_data = self.state_queue.get()
                    
                    # Write to log
                    if self.log_writer:
                        self._write_log_entry(state_data)
                
                time.sleep(dt)
            
            except Exception as e:
                logger.error(f"Logging error: {e}")
    
    def _create_log_file(self):
        """Create log file for data recording"""
        # Create log directory
        os.makedirs(self.log_dir, exist_ok=True)
        
        # Create filename with timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"iphone_ekf_log_{timestamp}.csv"
        filepath = os.path.join(self.log_dir, filename)
        
        try:
            self.log_file = open(filepath, 'w')
            self.log_writer = self.log_file
            
            # Write header
            header = [
                'timestamp',
                'x', 'y', 'z', 'vz',
                'roll', 'pitch', 'yaw', 'yaw_rate',
                'accel_x', 'accel_y', 'accel_z',
                'gyro_x', 'gyro_y', 'gyro_z',
                'cov_trace'
            ]
            self.log_writer.write(','.join(header) + '\n')
            
            logger.info(f"Log file created: {filepath}")
        
        except Exception as e:
            logger.error(f"Failed to create log file: {e}")
    
    def _write_log_entry(self, state_data: Dict):
        """Write a log entry"""
        try:
            state = state_data['state']
            sensor = state_data['raw_sensor']
            cov_trace = np.trace(state_data['covariance'])
            
            # Format data
            row = [
                state_data['timestamp'],
                state.x, state.y, state.z, state.vz,
                state.roll, state.pitch, state.yaw, state.yaw_rate,
                sensor.accel_x if sensor else 0,
                sensor.accel_y if sensor else 0,
                sensor.accel_z if sensor else 0,
                sensor.gyro_x if sensor else 0,
                sensor.gyro_y if sensor else 0,
                sensor.gyro_z if sensor else 0,
                cov_trace
            ]
            
            # Write to file
            self.log_writer.write(','.join(map(str, row)) + '\n')
            self.log_writer.flush()
        
        except Exception as e:
            logger.error(f"Log write error: {e}")
    
    def get_current_state(self) -> Optional[EKF8State]:
        """Get current EKF state"""
        return self.ekf.get_state()
    
    def get_statistics(self) -> Dict[str, Any]:
        """Get system statistics"""
        ekf_stats = self.ekf.get_statistics()
        receiver_stats = self.receiver.get_statistics()
        
        return {
            'system': self.stats,
            'ekf': ekf_stats,
            'receiver': receiver_stats
        }
    
    def reset_ekf(self, initial_state: Optional[EKF8State] = None):
        """Reset EKF to initial conditions"""
        self.ekf.reset(initial_state)
        logger.info("EKF reset")


# Command-line interface
if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description='iPhone-EKF Integration System')
    parser.add_argument('--config', type=str, help='Configuration file path')
    parser.add_argument('--port', type=int, default=5555, help='UDP port')
    parser.add_argument('--calibrate', action='store_true', help='Run calibration')
    parser.add_argument('--duration', type=int, default=60, help='Run duration in seconds')
    
    args = parser.parse_args()
    
    # Create integration system
    integration = iPhoneEKFIntegration(args.config)
    
    # Override port if specified
    if args.port:
        integration.receiver.port = args.port
    
    # Run calibration if requested
    if args.calibrate:
        integration.calibrate()
    
    # Define state callback
    def state_callback(state: EKF8State):
        print(f"State: {state}")
    
    try:
        # Start system
        integration.start(state_callback=state_callback)
        
        # Run for specified duration
        logger.info(f"Running for {args.duration} seconds...")
        start_time = time.time()
        
        while time.time() - start_time < args.duration:
            time.sleep(1)
            
            # Print statistics
            stats = integration.get_statistics()
            logger.info(f"Stats: EKF updates={stats['ekf']['update_count']}, "
                       f"Packets={stats['receiver']['packets_received']}, "
                       f"Rate={stats['receiver']['data_rate']:.1f} Hz")
    
    except KeyboardInterrupt:
        logger.info("Interrupted by user")
    
    finally:
        # Stop system
        integration.stop()
        
        # Final statistics
        final_stats = integration.get_statistics()
        logger.info(f"Final statistics: {final_stats}")
