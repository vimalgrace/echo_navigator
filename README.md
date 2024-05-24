## Echo Navigator : The Interactive Robotic Companion

## Description
The Echo Navigator project aims to streamline navigation and enhance safety in college campuses. Utilizing robotics and advanced technologies, it addresses challenges such as navigating vast campuses and locating specific facilities, especially in crowded environments. Its primary goal is to offer visitors an autonomous and informative journey throughout the campus. By harnessing cutting-edge innovations, this system seeks to simplify the complexities of campus navigation, providing a tailored solution for educational institutions.

## Prototpe Model
The video can be found here [video link](https://youtu.be/v_VCSLp4lzo?feature=shared)

<img align="center" src="https://github.com/vimalgrace/echo_navigator/assets/91270314/815d8150-8cc7-4428-a94e-49e0d254bb49"> 
 
## Block Diagram
![image](https://github.com/vimalgrace/echo_navigator/assets/91270314/5aeeca6b-842a-4cb7-a0be-40615586d807)

## Technology Used 

## Hardware
- Raspberry Pi 4B (8GB RAM)
- Arduino Mega 2560
- RPLIDAR A1M8
- Johnson's DC Gear Motor
- L298N Motor Driver
- Speaker
- LM7805 Voltage Regulator
- Wheels (40cm diameter)
- OE-37 Hall Effect Two Channel Magnetic Encoder
- QMC5883L Magnetometer Sensor
- Anti-Slip Motor Coupling
- 6-24V to 5V USB Output Buck converter
- Lithium-ion Battery Pack (11.1 Volts)
- 7 inch Touch Display
- Connecting Cables


## Software
- Ubuntu 22.04 LTS
- Robot Operating System (ROS2) (Distro : Humble)
- Nav2 Stack
- Python
- C++
- Arduino Programming
- OpenCV for Computer Vision
  
## Packages Used:
- **ros2_control** for interfacing raspberry pi with arduino
- **slam_toolbox** for generating map
- **rplidar_ros** for publishing scan data from lidar
- **amcl** for localizing
- **pyttsx3** for speech

## Controllers Used:
- **MPPI Controller** to effectively follow the path

## Usage 
The Echo Navigator system is currently in use across college campuses, providing visitors with autonomous navigation and safety features. Leveraging robotics and advanced technologies, it guides users through campus, helping them locate facilities and navigate crowded areas. Its deployment enhances visitor experience and improves overall campus navigation efficiency.


