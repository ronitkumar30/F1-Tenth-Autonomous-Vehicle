### Challenge

Lane detection and obstacle avoidance are computationally heavy tasks to execute in real-time. We aim to reduce this computation by generating 2D maps of the vehicle's path and identifying traffic signs and obstacles along the way.

---

### Difficulty & Interest

To address this problem, real-time lane detection and obstacle avoidance are required. Our vehicle uses speed and steering angle data to calculate waypoints and adapt to its environment. We also incorporate computer vision to recognize road signs and lidar data to map obstacles dynamically.

---

### System Overview & Methodology

The system includes:
1. **Lane Detection using OpenCV**: A color transformation and contour detection technique is used to detect lanes
2. **Obstacle Detection**: Using lidar data to map obstacles in real time
3. **Road Sign Detection using CNN and YOLO v5**: Detects and identififes road signs using a pretrained model
4. **2D Mapping**: The system plots the vehicle's trajectory and obstacles using data from the Vicon system
5. **A* Search Algorithm**: Implements A* for finding the shortest path between waypoints based on dynamic mapping data

---

### Methodology: Lane Detection

We use OpenCV and contour detection for lane following:
1. **Color Thresholding**: Detects yellow pixels using thresholds
2. **Contours**: Finds the largest contour in the image, assumed to be the lane
3. **Steering Angle Calculation**: Computes steering angle using the arctan2 function based on the contour's position relative to the image center
4. **PID Controller**: Adjusts the steering angle based on real-time error to ensure smooth lane following

---

### Methodology: Road Sign Detection

We used a pretrained YOLO v5 model for road sign detection, trained on the GTSRB dataset with over 50,000 images and 43 classes. Despite some challenges in speed detection and accuracy, YOLO v5 proved effective in identifying signs in real-time.

---

### Methodology: 2D Mapping

We collected the car's pose and control data from Vicon and used lidar data to create a graph of the environment. The next step is to incorporate road sign positions into the map for dynamic updates.

---

### Next Steps

1. Fine-tune steering angles and filter extreme values.
2. Improve trajectory mapping by incorporating steering angle and speed data.
3. Test 2D mapping and the A* search algorithm in real-world conditions.
