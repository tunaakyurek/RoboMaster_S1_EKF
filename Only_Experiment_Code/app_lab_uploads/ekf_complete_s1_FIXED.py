# RoboMaster S1 Complete EKF Implementation - FIXED for Lab
# Removed f-strings and unicode for compatibility
# Single file, ready to copy-paste into RoboMaster app

import time
import math

print("=" * 50)
print("RoboMaster S1 EKF v2.0 FIXED")
print("=" * 50)

# ============================================================================
# CONFIGURATION
# ============================================================================

EKF_FREQUENCY = 5  # Hz
RUNTIME_SECONDS = 30  # Total runtime
USE_LED_FEEDBACK = True  # Visual indicators
PRINT_FREQUENCY = 2  # Hz

# Filter parameters
ACCEL_TRUST = 0.15
GYRO_TRUST = 0.85
CHASSIS_TRUST = 0.95

# Process noise
Q_POSITION = 0.01
Q_ANGLE = 0.05

# Measurement noise
R_ACCEL = 0.5
R_GYRO = 0.1
R_CHASSIS = 0.05

# ============================================================================
# SIMPLE MATRIX CLASS
# ============================================================================

class Matrix:
    def __init__(self, rows, cols, data=None):
        self.rows = rows
        self.cols = cols
        if data:
            self.data = data
        else:
            self.data = [[0.0 for _ in range(cols)] for _ in range(rows)]
    
    @staticmethod
    def eye(n):
        m = Matrix(n, n)
        for i in range(n):
            m.data[i][i] = 1.0
        return m
    
    def get(self, row, col):
        return self.data[row][col]
    
    def set(self, row, col, value):
        self.data[row][col] = value

# ============================================================================
# SIMPLIFIED EKF
# ============================================================================

class S1_EKF:
    def __init__(self):
        # State: [x, y, z, roll, pitch, yaw]
        self.state = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.velocity = [0.0, 0.0, 0.0]
        self.angular_velocity = [0.0, 0.0, 0.0]
        
        self.prev_state = list(self.state)
        self.prev_time = time.time()
        
        # Covariance
        self.P = Matrix.eye(6)
        
        # Complementary filter states
        self.comp_roll = 0.0
        self.comp_pitch = 0.0
        
        # Stats
        self.update_count = 0
    
    def predict(self, dt):
        if dt <= 0:
            return
        
        # Update position based on velocity
        for i in range(3):
            self.state[i] += self.velocity[i] * dt
        
        # Update orientation
        self.state[3] += self.angular_velocity[0] * dt
        self.state[4] += self.angular_velocity[1] * dt
        self.state[5] += self.angular_velocity[2] * dt
        
        # Normalize angles
        for i in range(3, 6):
            while self.state[i] > math.pi:
                self.state[i] -= 2 * math.pi
            while self.state[i] < -math.pi:
                self.state[i] += 2 * math.pi
        
        # Increase uncertainty
        for i in range(6):
            current = self.P.get(i, i)
            self.P.set(i, i, current + Q_POSITION * dt)
    
    def update_imu(self, accel, gyro, dt):
        if not accel or not gyro or dt <= 0:
            return
        
        # Convert gyro to radians
        gyro_rad = [math.radians(g) for g in gyro]
        self.angular_velocity = gyro_rad
        
        # Calculate orientation from accelerometer
        ax, ay, az = accel[0], accel[1], accel[2]
        
        if abs(az) > 0.1:
            accel_roll = math.atan2(ay, az)
            accel_pitch = math.atan2(-ax, math.sqrt(ay*ay + az*az))
        else:
            accel_roll = self.state[3]
            accel_pitch = self.state[4]
        
        # Complementary filter
        self.comp_roll = GYRO_TRUST * (self.comp_roll + gyro_rad[0] * dt) + ACCEL_TRUST * accel_roll
        self.comp_pitch = GYRO_TRUST * (self.comp_pitch + gyro_rad[1] * dt) + ACCEL_TRUST * accel_pitch
        
        self.state[3] = self.comp_roll
        self.state[4] = self.comp_pitch
        self.state[5] += gyro_rad[2] * dt
        
        # Reduce uncertainty
        for i in range(3, 6):
            current = self.P.get(i, i)
            self.P.set(i, i, current * 0.9)
        
        self.update_count += 1
    
    def update_chassis(self, x_mm, y_mm, yaw_deg):
        if x_mm is None or y_mm is None:
            return
        
        x_m = x_mm / 1000.0
        y_m = y_mm / 1000.0
        yaw_rad = math.radians(yaw_deg) if yaw_deg is not None else self.state[5]
        
        self.state[0] = CHASSIS_TRUST * x_m + (1 - CHASSIS_TRUST) * self.state[0]
        self.state[1] = CHASSIS_TRUST * y_m + (1 - CHASSIS_TRUST) * self.state[1]
        
        if yaw_deg is not None:
            self.state[5] = CHASSIS_TRUST * yaw_rad + (1 - CHASSIS_TRUST) * self.state[5]
        
        for i in range(2):
            current = self.P.get(i, i)
            self.P.set(i, i, current * 0.8)
    
    def update_velocity(self, dt):
        if dt <= 0:
            return
        
        for i in range(3):
            self.velocity[i] = (self.state[i] - self.prev_state[i]) / dt
        
        self.prev_state = list(self.state)

# ============================================================================
# MAIN EXECUTION
# ============================================================================

def main():
    print("Initializing...")
    
    # Set robot mode
    robot_ctrl.set_mode(rm_define.robot_mode_gimbal_follow)
    
    # Initialize EKF
    ekf = S1_EKF()
    
    # LED startup
    if USE_LED_FEEDBACK:
        led_ctrl.set_flash(rm_define.armor_all, 5)
        time.sleep(0.5)
        led_ctrl.turn_off(rm_define.armor_all)
    
    # Subscribe to sensors
    try:
        sensor_imu.sub_gyroscope(freq=min(10, EKF_FREQUENCY * 2))
        sensor_imu.sub_accelerometer(freq=min(10, EKF_FREQUENCY * 2))
        chassis_ctrl.sub_position(freq=min(5, EKF_FREQUENCY))
        print("Sensors subscribed")
    except Exception as e:
        print("Sensor error: " + str(e))
        return
    
    # Timing
    start_time = time.time()
    last_update = start_time
    last_print = start_time
    iteration = 0
    
    print("Starting EKF...")
    print("-" * 50)
    
    try:
        while time.time() - start_time < RUNTIME_SECONDS:
            current_time = time.time()
            dt = current_time - last_update
            
            if dt < (1.0 / EKF_FREQUENCY):
                time.sleep(0.01)
                continue
            
            # Get sensor data
            try:
                gyro = sensor_imu.get_gyroscope()
                accel = sensor_imu.get_accelerometer()
                pos = chassis_ctrl.get_position()
            except:
                time.sleep(0.1)
                continue
            
            # EKF Update
            ekf.predict(dt)
            
            if accel and gyro:
                ekf.update_imu(accel, gyro, dt)
            
            if pos:
                ekf.update_chassis(pos[0], pos[1], pos[2] if len(pos) > 2 else None)
            
            ekf.update_velocity(dt)
            
            # LED feedback
            if USE_LED_FEEDBACK:
                if abs(ekf.velocity[0]) > 0.1 or abs(ekf.velocity[1]) > 0.1:
                    led_ctrl.set_flash(rm_define.armor_all, 3)
                else:
                    led_ctrl.set_bottom_led(rm_define.armor_bottom_all, 0, 0, 255, rm_define.effect_always_on)
            
            # Print status
            if current_time - last_print >= (1.0 / PRINT_FREQUENCY):
                elapsed = current_time - start_time
                rate = iteration / elapsed if elapsed > 0 else 0
                
                print("Time: " + str(round(elapsed, 1)) + "s, Rate: " + str(round(rate, 1)) + " Hz")
                print("  Pos: X=" + str(round(ekf.state[0], 3)) + " Y=" + str(round(ekf.state[1], 3)) + "m")
                print("  Ang: R=" + str(round(math.degrees(ekf.state[3]), 1)) + " P=" + str(round(math.degrees(ekf.state[4]), 1)) + " Y=" + str(round(math.degrees(ekf.state[5]), 1)) + " deg")
                print("-" * 50)
                
                last_print = current_time
            
            last_update = current_time
            iteration += 1
    
    except Exception as e:
        print("Error: " + str(e))
    
    finally:
        # Cleanup
        print("Cleaning up...")
        
        try:
            sensor_imu.unsub_gyroscope()
            sensor_imu.unsub_accelerometer()
            chassis_ctrl.unsub_position()
        except:
            pass
        
        # Show completion
        if USE_LED_FEEDBACK:
            for _ in range(3):
                led_ctrl.set_flash(rm_define.armor_all, 10)
                time.sleep(0.2)
                led_ctrl.turn_off(rm_define.armor_all)
                time.sleep(0.2)
        
        elapsed_total = time.time() - start_time
        print("=" * 50)
        print("EKF COMPLETE")
        print("Runtime: " + str(round(elapsed_total, 1)) + " seconds")
        print("Updates: " + str(iteration))
        print("Avg Rate: " + str(round(iteration/elapsed_total, 1)) + " Hz")
        print("=" * 50)

# Run
main()
