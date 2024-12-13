#!/usr/bin/env python3

import rospy
import message_filters
import cv2
import csv
import numpy as np
from sensor_msgs.msg import CompressedImage, Imu
from cv_bridge import CvBridge
import os

# Global variables for video writer and CSV file
video_writer = None
csv_file = None
csv_writer = None
frame_count = 0

# Initialize CV Bridge
bridge = CvBridge()

# Callback for synchronized data
def callback(image_msg, imu_msg):
    global video_writer, csv_writer, frame_count

    # Get timestamp in milliseconds
    timestamp_ms = int(image_msg.header.stamp.to_sec() * 1000)

    # Convert compressed image to OpenCV image
    try:
        np_arr = np.frombuffer(image_msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # Write the video frame
        video_writer.write(cv_image)
    except Exception as e:
        rospy.logerr(f"Failed to process image: {e}")
        return

    # Extract gyro and accelerometer data
    acc_x = imu_msg.linear_acceleration.x
    acc_y = imu_msg.linear_acceleration.y
    acc_z = imu_msg.linear_acceleration.z
    gyro_x = imu_msg.angular_velocity.x
    gyro_y = imu_msg.angular_velocity.y
    gyro_z = imu_msg.angular_velocity.z

    # Write to the CSV file
    csv_writer.writerow([frame_count, timestamp_ms, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z])

    rospy.loginfo(f"Recorded frame {frame_count} and IMU data.")
    frame_count += 1

def main():
    global video_writer, csv_file, csv_writer, frame_count

    # Initialize the ROS node
    rospy.init_node("video_gyro_recorder", anonymous=True)

    # Parameters
    video_filename = rospy.get_param("~video_filename", "output_video.mp4")
    csv_filename = rospy.get_param("~csv_filename", "output_gyro.csv")
    frame_rate = rospy.get_param("~frame_rate", 30)
    image_width = rospy.get_param("~image_width", 720)
    image_height = rospy.get_param("~image_height", 540)

    # Set up video writer
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    video_writer = cv2.VideoWriter(video_filename, fourcc, frame_rate, (image_width, image_height))
    if not video_writer.isOpened():
        rospy.logerr("Failed to open video writer.")
        return

    # Set up CSV file
    csv_file = open(csv_filename, mode='w', newline='')
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(["frame", "timestamp_ms", "org_acc_x", "org_acc_y", "org_acc_z", "org_gyro_x", "org_gyro_y", "org_gyro_z"])

    # Synchronize the image and IMU topics
    image_sub = message_filters.Subscriber("/camera/image_color/compressed", CompressedImage)
    imu_sub = message_filters.Subscriber("/camera/gyro/sample", Imu)
    ts = message_filters.ApproximateTimeSynchronizer([image_sub, imu_sub], queue_size=10, slop=0.1)
    ts.registerCallback(callback)

    rospy.loginfo("Recording video and IMU data...")

    # Keep the node running
    rospy.spin()

    # Release resources
    video_writer.release()
    csv_file.close()
    rospy.loginfo("Recording stopped and resources released.")

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node terminated.")
