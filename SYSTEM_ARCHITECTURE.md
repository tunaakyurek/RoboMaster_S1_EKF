# RoboMaster S1 EKF System Architecture

## System Overview

This document describes the architecture and communication structure of the RoboMaster S1 Extended Kalman Filter (EKF) system for state estimation.

## 1. System Architecture & Communication Structure

### Communication Flow
```
RoboMaster S1 ←→ Raspberry Pi ←→ Ground PC
    (Sensors)      (EKF Processing)   (Visualization)
```

### Node Responsibilities

#### RoboMaster S1 Control Unit
- **Purpose**: Sensor data provider
- **Sensors**: IMU (accelerometer, gyroscope), chassis encoders
- **Communication**: WiFi SDK connection to Raspberry Pi
- **Data Rate**: Up to 50 Hz sensor streaming

#### Raspberry Pi
- **Purpose**: Real-time EKF processing and data relay
- **Processing**: EKF algorithm, sensor fusion, state estimation
- **Communication**: 
  - Inbound: RoboMaster SDK (WiFi)
  - Outbound: ZeroMQ/TCP to Ground PC (WiFi/Ethernet)
- **Data Storage**: Local logging for backup and analysis
- **Real-time Requirements**: 50 Hz EKF updates

#### Ground PC
- **Purpose**: Visualization, analysis, and monitoring
- **Processing**: Real-time plotting, RMSE analysis, trajectory visualization
- **Communication**: ZeroMQ/TCP server (receives from Pi)
- **Storage**: Long-term data archival and post-processing

## 2. Software Stack & Languages

### Programming Languages
- **Python 3.7+**: Primary language for all components
- **JSON**: Configuration and data serialization
- **CSV**: Data logging format

### Key Libraries & Frameworks

#### Mathematical & Scientific Computing
- **NumPy**: Matrix operations, linear algebra for EKF
- **SciPy**: Advanced mathematical functions, filtering
- **Pandas**: Data analysis and manipulation (optional)

#### Communication
- **RoboMaster SDK**: Official DJI SDK for S1 communication
- **ZeroMQ (PyZMQ)**: High-performance network communication
- **Socket**: Fallback networking

#### Visualization
- **Matplotlib**: Real-time plotting and static analysis
- **Matplotlib Animation**: Real-time data visualization

#### Data Management
- **CSV**: Structured data logging
- **JSON**: Configuration and metadata storage

## 3. EKF Implementation Details

### State Vector (12 dimensions)
```
x = [x, y, z, vx, vy, vz, roll, pitch, yaw, wx, wy, wz]
```

Where:
- `[x, y, z]`: Position in world frame (meters)
- `[vx, vy, vz]`: Velocity in world frame (m/s)
- `[roll, pitch, yaw]`: Orientation (radians)
- `[wx, wy, wz]`: Angular velocity in body frame (rad/s)

### Sensor Fusion Strategy
According to the implementation requirements, the EKF relies **solely on sensor measurements**:

#### Primary Sensors (RoboMaster S1)
1. **IMU Accelerometer**: 3-axis acceleration (body frame)
2. **IMU Gyroscope**: 3-axis angular velocity (body frame)
3. **Chassis Encoders**: X, Y position and yaw angle (relative)

#### Future Extensibility
The system is designed to accommodate additional sensors:
- **Magnetometer**: For absolute heading reference
- **Barometer**: For altitude estimation
- **GPS**: For absolute position reference

### EKF Algorithm Flow
1. **Prediction Step**: Use kinematic motion model
2. **IMU Update**: Accelerometer and gyroscope measurements
3. **Chassis Update**: Encoder-based position measurements
4. **Optional Updates**: Magnetometer, GPS, barometer (when available)

## 4. Data Flow Architecture

### Data Types & Frequencies

#### Sensor Data (50 Hz)
```json
{
  "timestamp": 1234567890.123,
  "accel": [ax, ay, az],
  "gyro": [wx, wy, wz],
  "chassis_x": 0.5,
  "chassis_y": 0.3,
  "chassis_yaw": 0.1
}
```

#### EKF State (50 Hz)
```json
{
  "timestamp": 1234567890.123,
  "position": [x, y, z],
  "velocity": [vx, vy, vz],
  "orientation": [roll, pitch, yaw],
  "angular_velocity": [wx, wy, wz],
  "covariance_trace": 1.23
}
```

#### Status Updates (0.2 Hz)
```json
{
  "system_uptime": 123.45,
  "battery_level": 85,
  "data_counts": {"sensor": 1000, "ekf": 1000}
}
```

## 5. Network Architecture

### Protocol Stack
- **Transport Layer**: TCP/IP
- **Messaging**: ZeroMQ PUSH/PULL pattern
- **Serialization**: JSON
- **Error Handling**: Connection timeout, retry logic

### Network Configuration
- **Default Port**: 5555
- **Queue Size**: 1000 messages max
- **Timeout**: 5 seconds
- **Rate Limiting**: 100 Hz maximum transmission

### Security Considerations
- **Network**: Private WiFi network recommended
- **Authentication**: IP-based access control
- **Data Encryption**: Not implemented (private network assumed)

## 6. File System Organization

```
RoboMaster_S1/
├── src/                          # Source code
│   ├── ekf/                      # EKF implementation
│   ├── robomaster_interface/     # RoboMaster S1 communication
│   ├── data_collection/          # Logging and analysis
│   ├── communication/            # Network communication
│   ├── visualization/            # Real-time plotting
│   ├── main.py                   # Raspberry Pi main application
│   └── ground_station.py         # Ground PC application
├── config/                       # Configuration files
├── logs/                         # Data logs (CSV format)
├── data/                         # Processed data and results
├── scripts/                      # Utility scripts
└── tests/                        # Test cases
```

## 7. Real-time Performance Requirements

### Timing Constraints
- **Sensor Data**: 50 Hz (20ms period)
- **EKF Processing**: <10ms per update
- **Network Transmission**: <5ms latency
- **Visualization Update**: 10 Hz (100ms period)

### Resource Requirements
- **CPU**: Raspberry Pi 4 (ARM Cortex-A72)
- **Memory**: 2GB RAM minimum
- **Storage**: 32GB SD card minimum
- **Network**: 802.11n WiFi or Ethernet

## 8. Error Handling & Reliability

### Fault Tolerance
- **Connection Loss**: Automatic reconnection attempts
- **Data Loss**: Local logging backup
- **Sensor Failure**: Graceful degradation
- **Network Issues**: Queue buffering and retry

### Data Integrity
- **Timestamps**: Synchronized across all components
- **Validation**: Sensor data range checking
- **Backup**: Local storage on Raspberry Pi
- **Recovery**: Session restoration capabilities

## 9. Calibration & Configuration

### IMU Calibration
- **Bias Estimation**: Automatic gyroscope bias calculation
- **Gravity Alignment**: Accelerometer calibration
- **Noise Characterization**: Runtime noise estimation

### System Tuning
- **Process Noise**: Adjustable via configuration
- **Measurement Noise**: Per-sensor tuning parameters
- **Filter Gains**: Adaptive or manual tuning

## 10. Output & Analysis Capabilities

### Real-time Outputs
- **2D Trajectory**: Live trajectory visualization
- **State Variables**: Position, velocity, orientation plots
- **Uncertainty**: Covariance trace monitoring
- **Sensor Data**: Raw sensor visualization

### Post-processing Analysis
- **RMSE Calculation**: Estimation vs. ground truth
- **Trajectory Comparison**: EKF vs. chassis odometry
- **Statistical Analysis**: Performance metrics
- **Data Export**: CSV, JSON, and plot exports

This architecture provides a robust, scalable foundation for real-time drone state estimation with comprehensive data analysis capabilities.