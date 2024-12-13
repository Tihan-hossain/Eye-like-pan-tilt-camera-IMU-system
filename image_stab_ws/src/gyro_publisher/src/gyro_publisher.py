#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
from smbus2 import SMBus
import struct
import time
import numpy as np

# I2C Addresses
LSM6DS33_ADDR = 0x6b
OUTX_L_G = 0x22
OUTX_L_XL = 0x28

# Gyroscope bias (to be calculated during calibration)
gyro_bias = [0.0, 0.0, 0.0]

# Initialize the LSM6DS33 (Gyro and Accelerometer)
def initialize_sensor():
    with SMBus(1) as bus:
        # CTRL1_XL (Accelerometer control): ODR_XL = 104 Hz, FS_XL = ±4g, BW_XL = 50 Hz
        bus.write_byte_data(LSM6DS33_ADDR, 0x10, 0x4A)
        # CTRL2_G (Gyroscope control): ODR_G = 104 Hz, FS_G = 2000 dps
        bus.write_byte_data(LSM6DS33_ADDR, 0x11, 0x4C)
        # CTRL3_C (Common control): Enable Block Data Update (BDU)
        bus.write_byte_data(LSM6DS33_ADDR, 0x12, 0x44)
        rospy.loginfo("LSM6DS33 sensor initialized.")

# Read Gyroscope and Accelerometer data
def read_gyro_accel():
    with SMBus(1) as bus:
        # Read 6 bytes each for gyro and accel
        gyro_data = bus.read_i2c_block_data(LSM6DS33_ADDR, OUTX_L_G, 6)
        accel_data = bus.read_i2c_block_data(LSM6DS33_ADDR, OUTX_L_XL, 6)

        # Convert data to signed integers
        gx, gy, gz = struct.unpack('<hhh', bytes(gyro_data))
        ax, ay, az = struct.unpack('<hhh', bytes(accel_data))

        # Convert to standard units if necessary (scaling factors)
        gx *= 0.07  # Convert to dps (degrees per second)
        gy *= 0.07
        gz *= 0.07

        ax *= 0.122  # Convert to g (gravity)
        ay *= 0.122
        az *= 0.122

        return gx, gy, gz, ax, ay, az

# Calibrate the gyroscope
def calibrate_gyroscope(calibration_samples=100):
    global gyro_bias
    rospy.loginfo("Calibrating gyroscope... Please keep the sensor stationary.")
    gyro_readings = []

    for _ in range(calibration_samples):
        gx, gy, gz, _, _, _ = read_gyro_accel()
        gyro_readings.append([gx, gy, gz])
        time.sleep(0.01)  # 10 ms delay between samples

    # Compute the bias as the mean of the readings
    gyro_bias = np.mean(gyro_readings, axis=0)
    rospy.loginfo(f"Gyroscope calibration complete. Bias: {gyro_bias}")

# Main function to publish IMU data
def publish_imu_data():
    rospy.init_node("gyro_publisher", anonymous=True)
    imu_pub = rospy.Publisher("/camera/gyro/sample", Imu, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    # Initialize the sensor
    initialize_sensor()

    # Calibrate the gyroscope
    calibrate_gyroscope()

    while not rospy.is_shutdown():
        try:
            # Read gyroscope and accelerometer data
            gx, gy, gz, ax, ay, az = read_gyro_accel()

            # Subtract the gyroscope bias
            gx -= gyro_bias[0]
            gy -= gyro_bias[1]
            gz -= gyro_bias[2]

            # Create and populate the IMU message
            imu_msg = Imu()
            imu_msg.header.stamp = rospy.Time.now()
            imu_msg.header.frame_id = "imu_link"

            imu_msg.angular_velocity.x = gx
            imu_msg.angular_velocity.y = gy
            imu_msg.angular_velocity.z = gz

            imu_msg.linear_acceleration.x = ax
            imu_msg.linear_acceleration.y = ay
            imu_msg.linear_acceleration.z = az

            # Publish the IMU message
            imu_pub.publish(imu_msg)
            rate.sleep()

        except OSError as e:
            rospy.logerr(f"I2C communication error: {e}")
            time.sleep(1)  # Wait before retrying

if __name__ == "__main__":
    try:
        publish_imu_data()
    except rospy.ROSInterruptException:
        rospy.loginfo("IMU publisher node stopped.")
