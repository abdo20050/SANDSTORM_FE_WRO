import busio
import board
import adafruit_bno08x
from adafruit_bno08x.i2c import BNO08X_I2C
import math
import time
import numpy as np
class GyroError(Exception): pass

class BNO085Sensor:
    """A class to handle BNO085 9-DOF IMU sensor operations"""
    
    def __init__(self, i2c_address=0x4b):
        """Initialize the BNO085 sensor.
        
        Args:
            i2c_address (int): I2C address of the sensor (default: 0x4B)
        """
        self.address = i2c_address
        self.i2c = None
        self.bno = None
        self.initialize_sensor()
        self.gyro_angles = [0,0,0] #x,y,z
        self.prev_gyro_angle_z = 0
        self.previous_time = time.time()
    def scan_i2c_devices(self):
        """Scan for available I2C devices."""
        devices = []
        self.i2c = busio.I2C(board.SCL, board.SDA)
        
        for address in range(128):
            try:
                self.i2c.try_lock()
                self.i2c.writeto(address, b'')
                devices.append(hex(address))
            except:
                pass
            finally:
                self.i2c.unlock()
        return devices
    
    def initialize_sensor(self):
        """Initialize the BNO085 sensor and enable required features."""
        try:
            self.i2c = busio.I2C(board.SCL, board.SDA)
            self.bno = BNO08X_I2C(self.i2c, address=self.address)
            print("BNO085 sensor initialized successfully")
            # # Soft reset
            # self.reset_sensor(type="HARD")
            # Enable required reports
            self.enable_features()
            # if self.bno.calibration_status == 0:
            #     raise GyroError("Gyro need calibration! \nRun 'calibrate_gyro.py'")
            
        except ValueError as e:
            print(f"Error initializing BNO085: {e}")
            print("Make sure the sensor is connected properly and the address is correct.")
            print("Available I2C devices:")
            print(self.scan_i2c_devices())
            raise
    
    def enable_features(self):
        """Enable the necessary sensor features."""
        features = [
            adafruit_bno08x.BNO_REPORT_ACCELEROMETER,
            adafruit_bno08x.BNO_REPORT_GYROSCOPE,
            adafruit_bno08x.BNO_REPORT_MAGNETOMETER,
            adafruit_bno08x.BNO_REPORT_ROTATION_VECTOR
        ]
        
        for feature in features:
            self.bno.enable_feature(feature)
    def reset_sensor(self, type:str = "SOFT"):
        """
        Reset the BNO sensor, soft or hard reset
        Args:
            type = SOFT or HARD
        """
        if type == "SOFT":
            self.bno.soft_reset()
        elif type == "HARD":
            self.bno.hard_reset()
    @staticmethod
    def quat_to_euler(x, y, z, w):
        """Convert quaternion to Euler angles (roll, pitch, yaw).
        
        Args:
            x, y, z, w: Quaternion components
            
        Returns:
            tuple: (roll, pitch, yaw) in degrees
        """
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
            
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return (math.degrees(roll), math.degrees(pitch), math.degrees(yaw))
    
    def update_angles_from_gyro(self, alpha = 0.98):
        # accel_x, accel_y, accel_z = self.bno.acceleration  # m/s^2
        gyro_x, gyro_y, gyro_z = self.bno.gyro  # rad/s
        
        # Calculate accelerometer angles
        # accel_angle_x = (180 / np.pi) * np.arctan(accel_y / np.sqrt(accel_x**2 + accel_z**2))
        # accel_angle_y = (180 / np.pi) * np.arctan(-accel_x / np.sqrt(accel_y**2 + accel_z**2))
                                                                    
        current_time = time.time()
        dt = current_time - self.previous_time
        self.previous_time = current_time
        
        # Integrate gyroscope data
        # gyro_angle_x = gyro_x * dt
        # gyro_angle_y = gyro_y * dt
        gyro_angle_z = gyro_z * dt
        
        # Apply complementary filter
        # self.gyro_angles[0] = alpha * (self.gyro_angles[0] + gyro_angle_x) + (1 - alpha) * accel_angle_x
        # self.gyro_angles[1] = alpha * (self.gyro_angles[1] + gyro_angle_y) + (1 - alpha) * accel_angle_y
        self.gyro_angles[2] += gyro_angle_z * alpha +  (1-alpha)*self.prev_gyro_angle_z 
        # self.gyro_angles[2] = self.gyro_angles[2] % 2*np.pi
        self.prev_gyro_angle_z = gyro_angle_z 

    def get_sensor_data(self):
        """Get all sensor measurements.
        
        Returns:
            dict: Dictionary containing acceleration, gyroscope, 
                 magnetometer, and euler angle measurements
        """
        try:
            accel_x, accel_y, accel_z = self.bno.acceleration  # m/s^2
            gyro_x, gyro_y, gyro_z = self.bno.gyro  # rad/s
            mag_x, mag_y, mag_z = self.bno.magnetic  # µT
            quat_i, quat_j, quat_k, quat_real = self.bno.quaternion
            self.update_angles_from_gyro()
            # Convert quaternion to Euler angles
            roll, pitch, yaw = self.quat_to_euler(quat_i, quat_j, quat_k, quat_real)
            
            return {
                'acceleration': (accel_x, accel_y, accel_z),
                'gyro': (gyro_x, gyro_y, gyro_z),
                'magnetic': (mag_x, mag_y, mag_z),
                'euler': (roll, pitch, yaw),
                'gyro_angles':self.gyro_angles
            }
        except Exception as e:
            print(f"Error reading sensor data: {e}")
            return None
            
    def close(self):
        """Clean up I2C resources."""
        if self.i2c:
            self.i2c.deinit()

def creat_bno_sensor():
    """Creat instance of Bno sensor."""
    return BNO085Sensor()

if __name__ == "__main__":
    bno = creat_bno_sensor()
    try:
        while 1:
            sensor_data = bno.get_sensor_data()
            if not sensor_data:
                print("error: no data")
            else:
                gyro_angle = sensor_data['gyro_angles'][2]
                gyro_angle = np.degrees(gyro_angle,)
                gyro_angle = (gyro_angle + 180) % 360 - 180 
                print(f"yaw reading:{gyro_angle:.2f}", f",status: {bno.bno.calibration_status}")

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("Stopping...")
        bno.close()