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
            port=self.config.get('port', 5555),
            host=self.config.get('listen_host', '0.0.0.0')
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
        # Raw sensor logging (independent from EKF)
        self.raw_log_file = None
        self.raw_log_writer = None
        
        # State publishing
        self.state_queue = queue.Queue(maxsize=100)
        self.state_callback = None
        
        # Control flags
        self.is_running = False
        self.ekf_thread = None
        self.log_thread = None
        
        # Calibration and phased startup (pre-delay → calibration → running)
        self.is_calibrated = False
        self.calibration_data = []
        self.calibration_start_time = None
        self._calibration_duration = self.config.get('calibration', {}).get('duration', 5.0)
        self.pre_start_delay_seconds = self.config.get('calibration', {}).get('pre_start_delay_seconds', 5.0)
        self.system_start_time = None  # Track when the system truly started processing
        self.first_packet_time = None
        self._phase = 'idle'  # idle | pre_delay | calibrating | running
        # GPS/Stationary helpers
        self._last_gyro_z = 0.0
        self._gps_origin_samples: list[tuple[float, float]] = []
        self._gps_origin_set = False
        self._gps_lpf = None  # low-pass filtered [x,y]
        
        # Statistics
        self.stats = {
            'ekf_updates': 0,
            'packets_processed': 0,
            'start_time': None,
            'last_update_time': None,
            'last_packet_age_s': None
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
                # Process noise (from formulary) - adjusted for better motion detection
                'q_accel': 0.1,        # Reduced from 0.5 - less conservative acceleration modeling
                'q_gyro': 0.005,       # Reduced from 0.01 - less conservative gyro modeling
                'q_accel_bias': 1e-5,  # Increased from 1e-6 - allow bias to adapt faster
                'q_gyro_bias': 1e-4,   # Increased from 1e-5 - allow gyro bias to adapt faster
                # Measurement noise - reduced for more responsive updates
                'r_accel': 0.05,       # Reduced from 0.1 - trust accelerometer more
                'r_gyro': 0.005,       # Reduced from 0.01 - trust gyroscope more
                'r_gps_pos': 0.5,      # Reduced from 1.0 - trust GPS position more
                'r_gps_vel': 0.2,      # Reduced from 0.5 - trust GPS velocity more
                'r_yaw': 0.2,          # Reduced from 0.5 - trust yaw measurements more
                'r_nhc': 0.05,         # Reduced from 0.1 - trust non-holonomic constraints more
                'r_zupt': 0.005,       # Reduced from 0.01 - trust zero-velocity updates more
                'r_zaru': 0.0005,      # Reduced from 0.001 - trust zero-angular-rate updates more
                # Initial uncertainties - reduced for faster convergence
                'init_pos_var': 0.5,   # Reduced from 1.0 - start with less uncertainty
                'init_theta_var': 0.05, # Reduced from 0.1 - start with less orientation uncertainty
                'init_vel_var': 0.2,   # Reduced from 0.5 - start with less velocity uncertainty
                'init_accel_bias_var': 0.005, # Reduced from 0.01 - start with less bias uncertainty
                'init_gyro_bias_var': 0.005   # Reduced from 0.01 - start with less bias uncertainty
            },
            'calibration': {
                'duration': 5.0,
                'min_samples': 100,
                'auto_calibrate': True
            },
            'logging': {
                'enabled': True,
                'rate': 50
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
        self._calibration_duration = duration
        
        # Start receiver for calibration
        self.receiver.start(callback=self._calibration_callback)
        
        # Wait for calibration: require both time and samples, allow extended window
        max_duration = max(duration * 3.0, duration + 10.0)
        last_log_count = 0
        while True:
            elapsed = time.time() - self.calibration_start_time
            time.sleep(0.1)
            if len(self.calibration_data) >= min_samples and elapsed >= duration:
                logger.info(f"✅ Calibration requirements met: samples={len(self.calibration_data)}, elapsed={elapsed:.1f}s")
                break
            if elapsed >= max_duration:
                logger.warning(f"⚠️ Calibration reached max duration {max_duration:.1f}s with samples={len(self.calibration_data)}")
                break
            if len(self.calibration_data) >= last_log_count + 50:
                last_log_count = len(self.calibration_data)
                logger.info(f"✅ Collected {len(self.calibration_data)} calibration samples (elapsed {elapsed:.1f}s)")
        
        # Stop receiver temporarily
        self.receiver.stop()
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
        if not self.calibration_start_time:
            return
        # Accept samples within configured calibration window (no hard 10s cap)
        if (time.time() - self.calibration_start_time) < (self._calibration_duration or 5.0) * 3.0:
            self.calibration_data.append(data)
    
    def _process_calibration_data(self):
        """Process calibration data to estimate initial biases"""
        if not self.calibration_data:
            return
        
        # Calculate sensor biases from stationary data
        accel_x_samples = [d.accel_x for d in self.calibration_data]
        accel_y_samples = [d.accel_y for d in self.calibration_data]
        gyro_z_samples = [d.gyro_z for d in self.calibration_data]
        
        bias_accel_x = np.mean(accel_x_samples)
        bias_accel_y = np.mean(accel_y_samples)
        bias_gyro_z = np.mean(gyro_z_samples)
        
        # Set initial state with estimated biases
        initial_state = RoboMasterState(
            x=0.0, y=0.0, theta=0.0,
            vx=0.0, vy=0.0,
            bias_accel_x=bias_accel_x,
            bias_accel_y=bias_accel_y,
            bias_angular_velocity=bias_gyro_z
        )
        
        self.ekf.reset(initial_state)
        self.processor.calibrate(self.calibration_data, len(self.calibration_data) * 0.02)
        logger.info(f"✅ Estimated biases: accel=[{bias_accel_x:.3f}, {bias_accel_y:.3f}], gyro={bias_gyro_z:.3f}")
    
    def start(self, state_callback: Optional[callable] = None):
        """Start the RoboMaster integration system"""
        # Use phased calibration driven by incoming packets: pre-delay then calibration then run
        if self.config.get('calibration', {}).get('auto_calibrate', True):
            self._phase = 'idle'
        else:
            self._phase = 'running'
            self.is_calibrated = True
        
        self.state_callback = state_callback
        self.is_running = True
        self.stats['start_time'] = time.time()
        
        # Create log files
        if self.config.get('logging', {}).get('enabled', True):
            self._create_log_file()
            self._create_raw_log_file()
        
        # Start receiver (always-on listener). If stream pauses (e.g., Siri Stop Recording),
        # the socket stays bound and receiver continues buffering/new packets when resumed.
        self.receiver.start(callback=self._sensor_data_callback)
        
        # DO NOT start EKF processing thread immediately - wait for phased startup
        # The EKF thread will be started after calibration is complete
        
        # Start logging thread for raw data only
        if self.raw_log_writer:
            self.log_thread = threading.Thread(target=self._logging_loop)
            self.log_thread.daemon = True
            self.log_thread.start()
        
        logger.info("RoboMaster integration system started - waiting for first sensor data to begin phased startup")
    
    def stop(self):
        """Stop the integration system"""
        self.is_running = False
        self.receiver.stop()
        if self.ekf_thread:
            self.ekf_thread.join(timeout=2)
        if self.log_thread:
            self.log_thread.join(timeout=2)
        if self.log_writer:
            self.log_writer.close()
        if self.raw_log_writer:
            self.raw_log_writer.close()
        self._print_final_stats()
        logger.info("RoboMaster integration system stopped")
    
    def _sensor_data_callback(self, data: iPhoneSensorData):
        """Process incoming sensor data with robust error handling and manage phased startup"""
        if not self.is_running:
            return

        if self.system_start_time is None:
            self.system_start_time = time.time()
            logger.info(f"First sensor data received. Starting pre-delay of {self.pre_start_delay_seconds}s.")
            # Do not process data during pre-start delay
            return

        elapsed_since_system_start = time.time() - self.system_start_time

        if elapsed_since_system_start < self.pre_start_delay_seconds:
            # Still in pre-start delay, just log and discard data
            # logger.debug(f"In pre-start delay ({elapsed_since_system_start:.1f}/{self.pre_start_delay_seconds:.1f}s). Discarding data.")
            return

        if not self.is_calibrated:
            if self.calibration_start_time is None:
                # Pre-delay has just finished, start calibration now
                self.calibration_start_time = time.time()
                logger.info(f"Pre-delay finished. Starting {self._calibration_duration}s calibration.")
                logger.info(f"Debug: system_start_time={self.system_start_time}, pre_delay={self.pre_start_delay_seconds}, calib_start={self.calibration_start_time}")
            
            elapsed_since_calibration_start = time.time() - self.calibration_start_time
            logger.debug(f"Debug: current_time={time.time()}, calib_start={self.calibration_start_time}, elapsed={elapsed_since_calibration_start:.1f}")
            if elapsed_since_calibration_start < self._calibration_duration:
                self.calibration_data.append(data)
                logger.debug(f"Collecting calibration data ({len(self.calibration_data)} samples, {elapsed_since_calibration_start:.1f}/{self._calibration_duration:.1f}s)")
                return # Do not process EKF data during calibration
            else:
                # Calibration duration met, process data and set calibrated
                logger.info(f"Calibration duration met. Processing {len(self.calibration_data)} samples.")
                try:
                    self._process_calibration_data()
                    self.is_calibrated = True
                    logger.info("✅ Calibration complete - biases estimated. Starting EKF processing.")
                    self._start_ekf_and_logging() # Start EKF and logging threads
                except RuntimeError as e:
                    logger.error(f"Calibration failed: {e}. System will not start EKF.")
                    self.stop() # Stop the system if calibration fails
                return # Calibration just finished, don't process this packet as EKF data yet

        # If calibrated and past pre-delay, proceed with normal EKF processing
        try:
            # Validate sensor data quality before processing
            if not self._validate_sensor_data(data):
                logger.warning(f"Skipping invalid sensor data: {data}")
                return
            
            processed_data = self.processor.process(data)
            
            # Additional validation of processed data
            if not self._validate_processed_data(processed_data):
                logger.warning(f"Skipping invalid processed data")
                return
            
            # Track latest yaw-rate for stationary gating
            if 'gyro' in processed_data and len(processed_data['gyro']) >= 3:
                self._last_gyro_z = processed_data['gyro'][2]
                
        except Exception as e:
            logger.error(f"Data processing failed: {e}")
            return
        
        # Only add valid data to queue
        if not self.state_queue.full():
            self.state_queue.put((data, processed_data))
            self.stats['packets_processed'] += 1
        else:
            logger.warning("State queue full, dropping data")

    def _start_ekf_and_logging(self):
        """Start EKF processing and logging threads after calibration is complete"""
        # Start EKF processing thread
        self.ekf_thread = threading.Thread(target=self._ekf_processing_loop)
        self.ekf_thread.daemon = True
        self.ekf_thread.start()
        
        # Start EKF logging thread if not already started
        if self.log_file and not hasattr(self, 'ekf_log_thread'):
            self.ekf_log_thread = threading.Thread(target=self._ekf_logging_loop)
            self.ekf_log_thread.daemon = True
            self.ekf_log_thread.start()
        
        logger.info("EKF processing and logging threads started")
    
    def _validate_sensor_data(self, data: iPhoneSensorData) -> bool:
        """Validate raw sensor data quality"""
        try:
            # Check for NaN or infinite values
            for attr in ['accel_x', 'accel_y', 'accel_z', 'gyro_x', 'gyro_y', 'gyro_z']:
                value = getattr(data, attr)
                if not np.isfinite(value) or abs(value) > 1000:  # Reasonable sensor limits
                    logger.warning(f"Invalid {attr}: {value}")
                    return False
            
            # Validate GPS data if present
            if hasattr(data, 'gps_lat') and data.gps_lat is not None:
                if not (-90 <= data.gps_lat <= 90) or not np.isfinite(data.gps_lat):
                    logger.warning(f"Invalid GPS lat: {data.gps_lat}")
                    return False
            if hasattr(data, 'gps_lon') and data.gps_lon is not None:
                if not (-180 <= data.gps_lon <= 180) or not np.isfinite(data.gps_lon):
                    logger.warning(f"Invalid GPS lon: {data.gps_lon}")
                    return False
            
            # Validate magnetometer data
            for attr in ['mag_x', 'mag_y', 'mag_z']:
                value = getattr(data, attr)
                if not np.isfinite(value) or abs(value) > 10000:  # Reasonable mag limits
                    logger.warning(f"Invalid {attr}: {value}")
                    return False
            
            return True
            
        except Exception as e:
            logger.error(f"Data validation error: {e}")
            return False
    
    def _validate_processed_data(self, processed_data: Dict) -> bool:
        """Validate processed sensor data"""
        try:
            # Check accelerometer data
            if 'accel' in processed_data:
                accel = processed_data['accel']
                if len(accel) != 3 or not all(np.isfinite(v) for v in accel):
                    logger.warning("Invalid accelerometer data")
                    return False
                # Check for reasonable acceleration values (excluding gravity)
                if any(abs(v) > 50 for v in accel[:2]):  # X,Y should be reasonable
                    logger.warning(f"Unreasonable acceleration: {accel}")
                    return False
            
            # Check gyroscope data
            if 'gyro' in processed_data:
                gyro = processed_data['gyro']
                if len(gyro) != 3 or not all(np.isfinite(v) for v in gyro):
                    logger.warning("Invalid gyroscope data")
                    return False
                # Check for reasonable angular velocity values
                if any(abs(v) > 10 for v in gyro):  # Should be reasonable rad/s
                    logger.warning(f"Unreasonable angular velocity: {gyro}")
                    return False
            
            # Check GPS data if present
            if 'gps' in processed_data and processed_data['gps']:
                gps = processed_data['gps']
                if 'lat' in gps and 'lon' in gps:
                    lat, lon = gps['lat'], gps['lon']
                    if not (-90 <= lat <= 90) or not (-180 <= lon <= 180):
                        logger.warning(f"Invalid GPS coordinates: {lat}, {lon}")
                        return False
                    
                    # Check for reasonable speed values
                    if 'speed' in gps:
                        speed = gps['speed']
                        if not np.isfinite(speed) or speed < 0 or speed > 100:  # 0-100 m/s reasonable
                            logger.warning(f"Invalid GPS speed: {speed}")
                            return False
                    
                    # Check for reasonable course values
                    if 'course' in gps:
                        course = gps['course']
                        if not np.isfinite(course) or not (0 <= course <= 360):
                            logger.warning(f"Invalid GPS course: {course}")
                            return False
            
            return True
            
        except Exception as e:
            logger.error(f"Processed data validation error: {e}")
            return False
    
    def _ekf_processing_loop(self):
        """Main EKF processing loop following RoboMaster formulary"""
        last_time = time.time()
        logger.info("EKF processing loop started")
        while self.is_running:
            try:
                raw_data, processed_data = self.state_queue.get(timeout=0.1)
                current_time = time.time()
                dt = current_time - last_time
                
                # Only process EKF data if calibration is complete
                if not self.is_calibrated:
                    logger.debug("EKF not yet calibrated, skipping data")
                    last_time = current_time
                    continue

                control_input = self._prepare_control_input(raw_data, processed_data)
                if control_input is None:
                    # If no control input (e.g., stream pause), keep EKF alive by skipping update
                    # but do not kill the loop; just continue to wait for next packet.
                    last_time = current_time
                    continue
                
                try:
                    self.ekf.predict(dt, control_input)
                except Exception as e:
                    logger.error(f"Prediction failed: {e}")
                    continue
                
                # Constraint helpers (do not change EKF internals)
                try:
                    self.ekf.update_non_holonomic_constraint()
                    self.ekf.update_zero_velocity()
                    self.ekf.update_zero_angular_rate()
                except Exception as e:
                    logger.error(f"Constraint updates failed: {e}")
                
                try:
                    self._update_with_gps(raw_data, processed_data)
                except Exception as e:
                    logger.error(f"GPS update failed: {e}")
                
                current_state = self.ekf.get_state()
                if self.autonomous_controller and AUTONOMOUS_AVAILABLE:
                    self._update_autonomous_controller(current_state)
                if self.state_callback:
                    self.state_callback(current_state)
                
                self.stats['ekf_updates'] += 1
                self.stats['last_update_time'] = current_time
                if self.receiver:
                    latest = self.receiver.get_latest_data()
                    if latest:
                        self.stats['last_packet_age_s'] = max(0.0, current_time - latest.timestamp)
                last_time = current_time
                
                if self.stats['ekf_updates'] % 100 == 0:
                    self._print_status(current_state)
            except queue.Empty:
                # No data available right now (e.g., Siri paused recording). Keep running.
                # Optionally, we could emit a heartbeat or apply constraint updates without new data.
                continue
            except Exception as e:
                logger.error(f"EKF processing error: {e}")
                import traceback
                traceback.print_exc()
    
    def _prepare_control_input(self, raw_data: iPhoneSensorData, processed_data: Dict) -> np.ndarray:
        """Prepare control input for RoboMaster EKF: [ax_body, ay_body, omega_z]"""
        try:
            accel = processed_data.get('accel', [0.0, 0.0, 0.0])
            gyro = processed_data.get('gyro', [0.0, 0.0, 0.0])
            return np.array([accel[0], accel[1], gyro[2]])
        except Exception as e:
            logger.error(f"Control input preparation error: {e}")
            return None
    
    def _is_state_valid(self, state: RoboMasterState) -> bool:
        """Check if EKF state is valid and reasonable"""
        try:
            # Check for NaN or infinite values
            if not all(np.isfinite(v) for v in [state.x, state.y, state.theta, state.vx, state.vy, 
                                               state.bias_accel_x, state.bias_accel_y, state.bias_angular_velocity]):
                return False
            
            # Check for reasonable values
            if abs(state.x) > 10000 or abs(state.y) > 10000:  # 10km limit
                return False
            
            if abs(state.vx) > 100 or abs(state.vy) > 100:  # 100 m/s limit
                return False
            
            if abs(state.bias_accel_x) > 10 or abs(state.bias_accel_y) > 10:  # 10 m/s² bias limit
                return False
            
            if abs(state.bias_angular_velocity) > 1:  # 1 rad/s bias limit
                return False
            
            return True
            
        except Exception:
            return False
    
    def _attempt_recovery(self):
        """Simple recovery mechanism"""
        try:
            logger.info("Attempting system recovery...")
            
            # Reset EKF to last known good state or default
            if hasattr(self, 'last_good_state') and self.last_good_state:
                logger.info("Restoring last known good state")
                self.ekf.reset(self.last_good_state)
            else:
                logger.info("Resetting EKF to default state")
                self.ekf.reset()
            
            # Clear error counters and continue
            logger.info("Recovery completed")
            
        except Exception as e:
            logger.error(f"Recovery failed: {e}")
    
    def _update_with_gps(self, raw_data: iPhoneSensorData, processed_data: Dict):
        """Update EKF with GPS (position and optional velocity)"""
        if 'gps' not in processed_data or not processed_data['gps']:
            return
        gps_data = processed_data['gps']
        if not all(k in gps_data for k in ['lat', 'lon']):
            return
        self._handle_gps_update(gps_data)
    
    def _handle_gps_update(self, gps_data: Dict):
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
        self.ekf.update_gps_position(gps_pos)
        # Optional velocity from speed/course
        if 'speed' in gps_data and 'course' in gps_data:
            speed = gps_data['speed']
            course_rad = np.radians(gps_data['course'])
            vx_gps = speed * np.cos(course_rad)
            vy_gps = speed * np.sin(course_rad)
            self.ekf.update_gps_velocity(np.array([vx_gps, vy_gps]))
            if speed > 0.7:
                yaw_course = course_rad - np.pi/2
                yaw_course = self._normalize_angle(yaw_course)
                self.ekf.update_yaw(yaw_course)
    
    def _update_autonomous_controller(self, state: RoboMasterState):
        if self.autonomous_controller:
            state_dict = {
                'x': state.x,
                'y': state.y,
                'z': 0.0,
                'yaw': state.theta,
                'vx': state.vx,
                'vy': state.vy
            }
            self.autonomous_controller.update_state(state_dict)
    
    def _create_log_file(self):
        """Create CSV log file for RoboMaster state"""
        os.makedirs(self.log_dir, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"robomaster_ekf_log_{timestamp}.csv"
        self.log_file = os.path.join(self.log_dir, filename)
        header = ("timestamp,x,y,theta,vx,vy,bias_accel_x,bias_accel_y,bias_angular_velocity,"
                 "accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,cov_trace\n")
        try:
            self.log_writer = open(self.log_file, 'w')
            self.log_writer.write(header)
            logger.info(f"RoboMaster log file created: {self.log_file}")
        except Exception as e:
            logger.error(f"Failed to create log file: {e}")
            self.log_file = None
    
    def _create_raw_log_file(self):
        """Create CSV log file for raw sensor data (EKF-independent)"""
        os.makedirs(self.log_dir, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"robomaster_raw_log_{timestamp}.csv"
        self.raw_log_file = os.path.join(self.log_dir, filename)
        header = (
            "timestamp,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,"
            "mag_x,mag_y,mag_z,gps_lat,gps_lon,gps_alt,gps_accuracy,gps_speed,gps_course,"
            "pressure,altitude,roll,pitch,yaw\n"
        )
        try:
            self.raw_log_writer = open(self.raw_log_file, 'w')
            self.raw_log_writer.write(header)
            logger.info(f"Raw sensor log file created: {self.raw_log_file}")
        except Exception as e:
            logger.error(f"Failed to create raw log file: {e}")
            self.raw_log_file = None
    
    def _logging_loop(self):
        """Background logging loop for raw sensor data only"""
        log_rate = self.config.get('logging', {}).get('rate', 50)
        log_interval = 1.0 / log_rate
        last_log_time = 0
        while self.is_running and self.raw_log_writer:
            current_time = time.time()
            if current_time - last_log_time >= log_interval:
                try:
                    self._write_raw_log_entry()
                    last_log_time = current_time
                except Exception as e:
                    logger.error(f"Raw logging error: {e}")
            time.sleep(0.001)

    def _ekf_logging_loop(self):
        """Background logging loop for EKF state data"""
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
                    logger.error(f"EKF logging error: {e}")
            time.sleep(0.001)
    
    def _write_log_entry(self):
        """Write current RoboMaster state to log file"""
        if not self.log_writer:
            return
        try:
            state = self.ekf.get_state()
            cov_trace = np.trace(self.ekf.get_covariance())
            latest_data = self.receiver.get_latest_data()
            if latest_data:
                accel = [latest_data.accel_x, latest_data.accel_y, latest_data.accel_z]
                gyro = [latest_data.gyro_x, latest_data.gyro_y, latest_data.gyro_z]
                log_entry = (f"{time.time()},{state.x},{state.y},{state.theta},"
                           f"{state.vx},{state.vy},"
                           f"{state.bias_accel_x},{state.bias_accel_y},{state.bias_angular_velocity},"
                           f"{accel[0]},{accel[1]},{accel[2]},"
                           f"{gyro[0]},{gyro[1]},{gyro[2]},{cov_trace}\n")
                self.log_writer.write(log_entry)
                self.log_writer.flush()
        except Exception as e:
            logger.error(f"Log write error: {e}")
    
    def _write_raw_log_entry(self):
        """Write most recent raw sensor data to dedicated raw log file"""
        if not self.raw_log_writer:
            return
        try:
            latest = self.receiver.get_latest_data()
            if latest is None:
                return
            log_entry = (
                f"{latest.timestamp},{latest.accel_x},{latest.accel_y},{latest.accel_z},"
                f"{latest.gyro_x},{latest.gyro_y},{latest.gyro_z},{latest.mag_x},{latest.mag_y},{latest.mag_z},"
                f"{latest.gps_lat},{latest.gps_lon},{latest.gps_alt},{latest.gps_accuracy},{latest.gps_speed},{latest.gps_course},"
                f"{latest.pressure},{latest.altitude},{latest.roll},{latest.pitch},{latest.yaw}\n"
            )
            self.raw_log_writer.write(log_entry)
            self.raw_log_writer.flush()
        except Exception as e:
            logger.error(f"Raw log write error: {e}")
    
    def _print_status(self, state: RoboMasterState):
        """Print periodic status updates"""
        runtime = time.time() - self.stats['start_time'] if self.stats['start_time'] else 0
        rate = self.stats['ekf_updates'] / runtime if runtime > 0 else 0
        logger.info(f"Stats: EKF updates={self.stats['ekf_updates']}, Packets={self.stats['packets_processed']}, Rate={rate:.1f} Hz")
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
        return self.ekf.get_state()
    
    def get_statistics(self) -> Dict[str, Any]:
        runtime = time.time() - self.stats['start_time'] if self.stats['start_time'] else 0
        return {
            'runtime': runtime,
            'ekf_updates': self.stats['ekf_updates'],
            'packets_processed': self.stats['packets_processed'],
            'average_rate': self.stats['ekf_updates'] / runtime if runtime > 0 else 0,
            'is_calibrated': self.is_calibrated,
            'ekf_stats': self.ekf.get_statistics()
        }
    
    def _normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-π, π]"""
        return (angle + np.pi) % (2 * np.pi) - np.pi


def main():
    """Main function for RoboMaster integration"""
    import argparse
    
    parser = argparse.ArgumentParser(description='RoboMaster iPhone-EKF Integration')
    parser.add_argument('--config', help='Configuration file path')
    parser.add_argument('--no-calibrate', action='store_true', help='Skip auto-calibration')
    args = parser.parse_args()
    
    def state_callback(state: RoboMasterState):
        print(f"State: {state}")
    
    integration = RoboMasterEKFIntegration(args.config)
    if args.no_calibrate:
        integration.config['calibration']['auto_calibrate'] = False
        integration.is_calibrated = True
    try:
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
