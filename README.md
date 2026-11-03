# CONTROL_2WD_ROBOT_BY_UDP_PROTOCOL
# CONTROL_2WD_ROBOT_BY_UDP_PROTOCOL 🤖

A comprehensive ROS 2 (Jazzy) project for controlling a 2-Wheel Drive (2WD) robot equipped with an ESP32 microcontroller via Wi-Fi (UDP Protocol). This repository represents the final project for Module 1 of the AI Robotics Diploma.

## 🌟 Project Features
* **ESP32 UDP API:** Direct, low-latency communication with the robot hardware using a custom Python API.
* **Sensor Integration:** Real-time publishing of Encoder data, Odometry (`/odom`), and IMU feedback (`/imu`).
* **Teleoperation:** Custom keyboard control node (W, A, S, D, Q, E) for manual navigation.
* **Motion Control Service:** A customized ROS 2 Service (`/drive_straight_acc`) that drives the robot in a straight line using a 3-phase motion profile (Acceleration $\rightarrow$ Constant Speed $\rightarrow$ Deceleration) relying on encoder feedback.

---

## 🛠️ Prerequisites & Requirements
* **OS:** Ubuntu 24.04 (or compatible)
* **ROS 2:** Jazzy Jalisco
* **Hardware:** ESP32, L298N Motor Driver, MPU6050, 2x DC Motors with Encoders (TurtleBot3 Burger equivalent).
* **Network:** The ESP32 must be configured as an Access Point (IP: `192.168.4.1`, Port: `8888`).

---

## 🚀 Installation & Workspace Setup

1. **Create a ROS 2 workspace (if you don't have one):**
2. Build the packages:
Make sure you are in the root of your workspace (~/ros2_ws), then build:

💻 Usage & Execution Instructions
Note: Ensure your laptop is connected to the ESP32 Wi-Fi network before running the nodes.

1. Run the Main Robot Node (Sensors & API)
This node establishes the UDP connection, calibrates the IMU, and listens for /cmd_vel commands.

 >>>ros2 run api_esp32_burger_robot task_1_pub
 
 2. Teleoperation (Keyboard Control)
Open a new terminal, source the workspace, and run the teleop node:

 >>>ros2 run api_esp32_burger_robot teleop_keyboard
 
 Controls: W (Forward), S (Backward), A (Left), D (Right), Q (Stop), E (Speed +/-).

4. Drive Straight with Acceleration (Service Server)
To test the custom 3-phase motion service, close the previous nodes and open a new terminal:

>>>ros2 run api_esp32_burger_robot Task3_bot

Wait for the terminal to display Task 4 Service (/drive_straight_acc) is Ready!.
Then, open a second terminal and call the service with your desired parameters:

 >>>ros2 service call /drive_straight_acc task3_interface/srv/Task3bot3 "{acceleration: 2.0, constant_speed: 0.2, stop_speed: 1.5}"
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
