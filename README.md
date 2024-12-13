# Open-Hardware Eye-like Compact Pan-Tilt Camera-IMU System

### Authors
- [Abhimanyu Bhowmik](https://github.com/abhimanyubhowmik)
- [Madhushree Sannigrahi](https://github.com/Madhushree2000)
- [Tihan Mahmud Hossain](https://github.com/Tihan-hossain)
- Md Shamin Yeasher Yousha

[![Final Report](https://img.shields.io/badge/Final-Report-brightgreen)](https://github.com/Tihan-hossain/Eye-like-pan-tilt-camera-IMU-system/blob/main/Aerial_Robotics_Report.pdf)
[![Demo Video](https://img.shields.io/badge/Demo-Video-yellow)](https://youtu.be/00TYHoU0B4w)

---

## Hardware Description

The hardware components for the project were provided by the [Autonomous Robots Lab](https://www.autonomousrobotslab.com/). Below are the key components:

- **FLIR Blackfly USB3 Colored Imaging Camera**: High-performance industrial camera for capturing stabilized video feeds.
- **2 Pololu MinIMU-9 v6**: Combines a gyroscope, accelerometer, and magnetometer for precise motion tracking.
- **2 Servos**: Enables smooth movement along pan and tilt axes.
- **Raspberry Pi 4 Model B**: Acts as the main computational hub.
- **Arduino UNO**: Controls servos and processes IMU data.

All CAD files required for assembling the hardware are available [here](https://github.com/Tihan-hossain/Eye-like-pan-tilt-camera-IMU-system/tree/main/CAD_Files).

| **Front View**                                                                                   | **Back View**                                                                                   |
|------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------|
| ![Front View](https://github.com/Tihan-hossain/Eye-like-pan-tilt-camera-IMU-system/raw/main/Images/Front.jpeg) | ![Back View](https://github.com/Tihan-hossain/Eye-like-pan-tilt-camera-IMU-system/raw/main/Images/Back.jpeg) |

This setup enables rotation of the camera along both horizontal (pan) and vertical (tilt) axes, enhancing its flexibility for various applications. The design details include:

- **Tilt Mechanism**: One servo motor mounted on the bottom plate for tilt control, providing movement along the vertical axis (20° to 140°).
- **Pan Mechanism**: Another servo motor on the top plate, enabling a horizontal pan range of 30° to 130°.

The assembled system facilitates precise and smooth camera positioning.

### Assembling the System

Print and assemble the system using the CAD files, ensuring proper alignment of components. Refer to the images below for guidance:


| ![Image 1](https://github.com/Tihan-hossain/Eye-like-pan-tilt-camera-IMU-system/raw/main/Images/1.jpeg)   | ![Image 2](https://github.com/Tihan-hossain/Eye-like-pan-tilt-camera-IMU-system/raw/main/Images/2.jpeg)   |

| ![Image 3](https://github.com/Tihan-hossain/Eye-like-pan-tilt-camera-IMU-system/raw/main/Images/3.jpeg)   | ![Image 4](https://github.com/Tihan-hossain/Eye-like-pan-tilt-camera-IMU-system/raw/main/Images/4.jpeg)   |

---

## Viewpoint Control

Once the system is assembled, configure the control system by following these steps:

1. **Upload the Arduino Code**
   - Download the Arduino code file: [Arduino/minIMU9_Stabilization.ino](https://github.com/Tihan-hossain/Eye-like-pan-tilt-camera-IMU-system/blob/main/Arduino/minIMU9_Stabilization.ino).
   - Use the Arduino IDE to upload the code to the Arduino UNO via serial communication.

2. **Calibration**
   - During startup, the IMU calibrates itself by estimating biases. Ensure the system remains stationary during this process.

3. **Operation**
   - Once calibrated, the system actively compensates for pan and tilt motions, functioning effectively as a gimbal. The servos adjust the camera's orientation in real-time based on IMU readings.

This setup ensures stabilized video output while maintaining flexibility for manually controlling the viewpoint if necessary.

---

## Image Stabilization
This project leverages the **Gyroflow** application for camera feed stabilization. To achieve this, gyro data and the camera's lens profile are used alongside the video feed. Below are the steps to record gyro and camera data for stabilization on a Raspberry Pi.

### 1. Clone and Setup the Repository on Raspberry Pi

#### SSH into Raspberry Pi
```bash
ssh rpi@hostname
```

#### Clone the Repository
```bash
git clone https://github.com/Tihan-hossain/Eye-like-pan-tilt-camera-IMU-system.git
```

#### Navigate to the Workspace
```bash
cd image_stab_ws/src
```

### 2. Install Required ROS Packages
To enable camera integration, install the necessary ROS packages:

```bash
sudo apt install ros-noetic-camera-info-manager \
ros-noetic-diagnostic-updater \
ros-noetic-dynamic-reconfigure \
ros-noetic-image-exposure-msgs \
ros-noetic-image-transport \
ros-noetic-nodelet \
ros-noetic-roscpp \
ros-noetic-sensor-msgs \
ros-noetic-wfov-camera-msgs
```

#### Install the Spinnaker Camera Driver
```bash
git clone -b noetic-devel https://github.com/ros-drivers/flir_camera_driver.git
```

### 3. Build and Source the Workspace
```bash
cd ~/image_stab_ws
catkin_make
source devel/setup.bash
```

### 4. Install the Spinnaker SDK
Visit the [Spinnaker SDK page](https://www.teledynevisionsolutions.com/products/spinnaker-sdk/) to download the Linux version and follow the installation instructions.

### 5. Running the Camera Node
Start the camera node with the following command:
```bash
roslaunch spinnaker_camera_driver camera.launch
```

#### For Color Correction
Run:
```bash
rosrun rqt_reconfigure rqt_reconfigure
```

Load the `image_stab_ws/camera_color.config` file for appropriate settings.

### 6. Connect and Configure the IMU
For gyro readings, connect the Pololu MinIMU 9 to the Raspberry Pi. Follow the steps provided in this repository: [https://github.com/DavidEGrayson/minimu9-ahrs](https://github.com/DavidEGrayson/minimu9-ahrs).

#### Run the Gyro Publisher
```bash
cd ~/image_stab_ws
source devel/setup.bash
rosrun gyro_publisher gyro_publisher.py
```

### 7. Recording Video and Gyro Data
Use the `video_gyro_recorder` package to record video and gyro data:
```bash
rosrun video_gyro_recorder video_gyro_recorder.py \
  _video_filename:="recorded_video.mp4" \
  _csv_filename:="gyro_data.csv"
```

After recording, you should have two files:
- `recorded_video.mp4`
- `gyro_data.csv`

### 8. Stabilizing the Video
Download the [Gyroflow application](https://github.com/gyroflow/) or its [Python Vesrion](https://github.com/gyroflow/gyroflow-python) onto your computer. Fetch the video and gyro CSV files from the Raspberry Pi. Use the `image_stab_ws/blackfly_camera.json` file as the lens profile in Gyroflow to produce a stabilized video.

## 9. Results
The table below showcases the results of stabilization using the Gyroflow application. The "Before Stabilization" video shows the original footage, and the "After Stabilization" video displays the stabilized output.

| **Before Stabilization**                                                                                   | **After Stabilization**                                                                                   |
|------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------|
| <video src=https://github.com/user-attachments/assets/a3851215-90d3-401a-a828-21f44ca3c404/> | <video src=https://github.com/user-attachments/assets/4715f8c0-b036-4d0f-9292-d359e889599b/> |


---

## Contact
For any queries, please contact:
[<b>bhowmikabhimanyu@gmail.com</b>](mailto:bhowmikabhimanyu@gmail.com)
[<b>tihan.mh@gmail.com</b>](mailto:tihan.mh@gmail.com)
