# Road Inspection Project – ROS2 Humble + TurtleBot3

## Author
**Julio Altamirano**

---

## Project Overview
This project, developed for the *EEL 5661 – Robotic Applications* course at **Florida Atlantic University**, demonstrates the use of **ROS2 Humble**, **Gazebo**, and **OpenCV** for automated **road inspection** using a **TurtleBot3** robot.

The robot navigates a simulated environment, captures images of the ground, detects cracks and potholes through classical computer vision techniques (edge detection and morphology), and logs the results in a CSV file for analysis.

---

## Objectives
- Perform road surface inspection using a mobile robot.
- Detect cracks using OpenCV in real time.
- Visualize the results as an overlay image.
- Demonstrate AI-assisted automation in transportation maintenance.

---

## Technologies Used
- **ROS2 Humble Hawksbill**
- **Gazebo Classic**
- **Python (rclpy, OpenCV, NumPy)**
- **TurtleBot3 (Waffle Pi model)**
- **Ubuntu 22.04 LTS**

---

## How to Run

1. **Launch the simulation:**
   ```bash
   source /opt/ros/humble/setup.bash
   ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

   ---
   
2. In a new terminal, run the crack detection node:
   cd ~/ros2_ws
   source install/setup.bash
   ros2 run road_inspection_pkg crack_detector

   ---

3. Use teleoperation to move the robot manually
   ros2 run turtlebot3_teleop teleop_keyboard

   ---

 Video Demonstration
 https://youtu.be/3NyqlJSDwZ0

   ---

Acknowledgment
Special thanks to Dr. Zvi Roth, Florida Atlantic University, for the guidance and support in developing this project.
 

