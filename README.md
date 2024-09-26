# F1-Tenth-Autonomous-Vehicle

## Project Overview

This project focuses on dynamic mapping for an F1-Tenth autonomous vehicle, aiming to ease the computational burden of lane detection and obstacle avoidance by mapping the vehicle's path, road signs, and obstacles in real time.

### Key Features
- **Lane Detection:** Real-time lane detection using computer vision (OpenCV)
- **Obstacle Detection:** Continuous obstacle detection and mapping using lidar
- **Road Sign Detection:** Recognition and response to road signs using Convolutional Neural Networks (CNNs) and YOLO v5
- **2D Mapping:** Plotting the vehicle's path, obstacles, and road signs on a dynamic map using Vicon and lidar data
- **A* Search Algorithm:** Tentatively implemented to find the shortest route between waypoints.

### Technologies Used
- **NVIDIA Jetson TX2:** For processing lane detection, object detection, and mapping
- **ROS (Robot Operating System):** To manage communication between sensors and systems
- **YOLO v5:** For detecting road signs
- **Ackermann Steering:** Controlled using a PID Controller for lane following
- **OpenCV:** For computer vision tasks, including color thresholding and contour detection

### Results
- Successfully implemented real-time lane following using OpenCV and a PID controller.
- Obstacle and sign detection were accurate, but variable lighting conditions and technical difficulties with Vicon tracking required adjustments
- The 2D mapping system was functional and tested on the F1-Tenth car
