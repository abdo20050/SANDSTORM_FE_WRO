import time
import math
import numpy as np
from mpu6050 import mpu6050

class KalmanFilter:
    """Kalman Filter for 1D angle estimation."""
    def __init__(self, process_variance, measurement_variance):
        self.q = process_variance  # Process noise variance
        self.r = measurement_variance  # Measurement noise variance
        self.x = 0.0  # Value
        self.p = 1.0  # Estimation error covariance
        self.k = 0.0  # Kalman gain

    def update(self, measurement):
        # Prediction update
        self.p += self.q
        # Measurement update
        self.k = self.p / (self.p + self.r)
        self.x += self.k * (measurement - self.x)
        self.p *= (1 - self.k)
        return self.x

class MPU6050Sensor:
    """A class to handle MPU6050 6-DOF IMU sensor operations with Kalman filter."""
    def __init__(self, i2c_address=0x68):
        self.initial_yaw = None
        self.address = i2c_address
        self.mpu = mpu6050(self.address)
        self.gyro_offsets = self.calibrate_gyro()
        self.previous_time = time.time()

        # Kalman filters for yaw
        self.kalman_yaw = KalmanFilter(process_variance=0.01, measurement_variance=0.1)
        self.gyro_yaw_rate = 0.0
        self.yaw_angle = 0.0

    def calibrate_gyro(self, samples=100):
        """Calibrate the gyroscope to find offsets."""
        print("Calibrating gyroscope...")
        offsets = [0, 0, 0]
        for _ in range(samples):
            gyro_data = self.mpu.get_gyro_data()
            offsets[0] += gyro_data['x']
            offsets[1] += gyro_data['y']
            offsets[2] += gyro_data['z']
            time.sleep(0.01)
        offsets = [offset / samples for offset in offsets]
        print(f"Gyro offsets: {offsets}")
        return offsets

    def get_yaw(self):
        """Calculate yaw using gyroscope and Kalman filter."""
        gyro_data = self.mpu.get_gyro_data()
        accel_data = self.mpu.get_accel_data()

        # Subtract offsets
        gyro_z = gyro_data['z'] - self.gyro_offsets[2]

        current_time = time.time()
        dt = current_time - self.previous_time
        self.previous_time = current_time

        # Integrate gyroscope to get yaw rate
        self.gyro_yaw_rate += gyro_z * dt

        # Apply Kalman filter to stabilize yaw
        self.yaw_angle = self.kalman_yaw.update(self.gyro_yaw_rate)

        # Set initial yaw
        if self.initial_yaw is None:
            self.initial_yaw = self.yaw_angle

        # Return yaw relative to the initial yaw
        yaw_relative = self.yaw_angle - self.initial_yaw
        return (yaw_relative + 180) % 360 - 180

    def get_sensor_data(self):
        """Get all sensor measurements."""
        try:
            accel_data = self.mpu.get_accel_data()
            gyro_data = self.mpu.get_gyro_data()
            yaw = self.get_yaw()

            return {
                'acceleration': accel_data,
                'gyro': gyro_data,
                'yaw': yaw
            }
        except Exception as e:
            print(f"Error reading sensor data: {e}")
            return None

def create_mpu_sensor():
    """Create an instance of the MPU6050 sensor."""
    return MPU6050Sensor()

if __name__ == "__main__":
    mpu = create_mpu_sensor()
    try:
        while True:
            sensor_data = mpu.get_sensor_data()
            if not sensor_data:
                print("Error: No data")
            else:
                print(f"Yaw reading: {sensor_data['yaw']:.2f}°")
            # time.sleep(0.1)
    except KeyboardInterrupt:
        print("Stopping...")
