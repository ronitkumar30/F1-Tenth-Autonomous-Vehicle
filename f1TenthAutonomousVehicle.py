#!/usr/bin/env python3

from __future__ import print_function

# Python Headers
import os
import csv
import math
import numpy as np
from numpy import linalg as la
import scipy.signal as signal
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import subprocess
import sys
import matplotlib.pyplot as plt
import time

# ROS Headers
import rospy

# GEM Sensor Headers
from std_msgs.msg import String, Bool, Float32, Float64, Float64MultiArray
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class lanenet_detector():
  def __init__(self):
      self.bridge = CvBridge()
      # NOTE
      # Uncomment this line for lane detection of GEM car in Gazebo
   #    self.sub_image = rospy.Subscriber('/gem/front_single_camera/front_single_camera/image_raw', Image, self.img_callback, queue_size=1)
      # Uncomment this line for lane detection of videos in rosbag
      self.sub_image = rospy.Subscriber('D435I/color/image_raw', Image, self.img_callback, queue_size=1)
      self.ctrl_pub  = rospy.Publisher("/vesc/low_level/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=1)
      self.debug_image_pub = rospy.Publisher('debug/yellow_line_detection', Image, queue_size=1)

      self.steering_angles = []
      self.timestamps = []
      self.start_time = time.time()
      self.last_recorded_time = 0
      self.last_steering_angle = 0.0
      self.max_steering_rate = 0.1
      self.steering_pid = PID(kp=0.1, ki=0.01, kd=0.005)

  def img_callback(self, data):
      try:
          # Convert a ROS image message into an OpenCV image
          cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      except CvBridgeError as e:
          print(e)

      yellow_lane = self.color_thresh(cv_image)
      steering_angle, arrow_cords, circle_center, radius = self.line_fit(yellow_lane)
      desired_steering_angle = self.steering_pid.compute(setpoint=0, pv=steering_angle)  # Assuming setpoint is straight forward which could be zero.

    #   steering_change = steering_angle - self.last_steering_angle
    #   steering_change = max(min(steering_change, self.max_steering_rate), -self.max_steering_rate)
    #   steering_angle = self.last_steering_angle + steering_change
    #   self.last_steering_angle = steering_angle

      self.drive_msg = AckermannDriveStamped()
      self.drive_msg.header.frame_id = "f1tenth_control"
      self.drive_msg.header.stamp = rospy.get_rostime()
      self.drive_msg.drive.speed     = 0.5 # m/s, reference speed
    #   self.drive_msg.drive.steering_angle = steering_angle
      self.drive_msg.drive.steering_angle = desired_steering_angle

      current_time = time.time()
      if current_time - self.last_recorded_time >= 0.01:
          self.timestamps.append(current_time - self.start_time)
          self.steering_angles.append(steering_angle+1.5)
          self.last_recorded_time = current_time

      if current_time - self.start_time >= 110:
          self.record_steering_angle(self.steering_angles, self.timestamps)

      self.ctrl_pub.publish(self.drive_msg)

      debug_image = self.create_debug_image(cv_image, yellow_lane, (steering_angle, arrow_cords), (circle_center, radius))

      contours, _ = cv2.findContours(yellow_lane, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
      cv2.drawContours(cv_image, contours, -1, (0, 255, 0), 2)
      try:
          self.debug_image_pub.publish(self.bridge.cv2_to_imgmsg(debug_image, "bgr8"))
      except CvBridgeError as e:
          print(e)

  def detect_red(self, img):
      # Convert the image from BGR to HSV
      hsv_image = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
      # Define range for red color and apply mask
      # Red can wrap around 180 in the HSV space, so two ranges may be needed
      lower_red1 = np.array([0, 70, 50])
      upper_red1 = np.array([10, 255, 255])
      lower_red2 = np.array([170, 70, 50])
      upper_red2 = np.array([180, 255, 255])

      mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
      mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
      red_mask = cv2.bitwise_or(mask1, mask2)

      # Check if red is significant
      if cv2.countNonZero(red_mask) > img.size * 0.1:  # Adjust the 0.1 threshold as necessary
          return True
      return False

  def record_steering_angle(self, steering_angles, timestamps):
      plt.figure()
      plt.plot(self.timestamps, self.steering_angles, c = 'blue', label = 'Steering Angle vs Time')
      plt.xlabel('Time (seconds)')
      plt.ylabel('Steering Angle (radians)')
      plt.title('Steering Angle vs Time')
      plt.grid(True)
      plt.show()

      x, y, theta = 0, 0, 0
      positions_x = [x]
      positions_y = [y]
      for i in range(1, len(timestamps)):
          dt = timestamps[i] - timestamps[i-1]
          theta += np.radians(steering_angles[i]) * dt * 10
          x += 1.3 * np.cos(theta) * dt
          y += 1.3 * np.sin(theta) * dt
          positions_x.append(x)
          positions_y.append(y)
      plt.figure(figsize=(10,5))
      plt.plot(positions_x,positions_y,marker='o')
      plt.title('Car Trajectory')
      plt.xlabel('X Position (meters)')
      plt.ylabel('Y Position (meters)')
      plt.grid(True)
      plt.axis('equal')
      plt.show()


  def create_debug_image(self, img, mask, steering_info, circle_info):
      steering_angle, arrow_cords = steering_info
      circle_center, radius = circle_info
      mask_indices = np.where(mask > 0)
      debug_img = img.copy()
      if circle_center and radius > 0:
          cv2.circle(debug_img, circle_center, radius, (0, 255, 0), 2)
      if np.any(mask_indices):
          min_y, max_y = np.min(mask_indices[0]), np.max(mask_indices[0])
          min_x, max_x = np.min(mask_indices[1]), np.max(mask_indices[1])
          cv2.rectangle(debug_img, (min_x, min_y), (max_x, max_y), (255, 0, 0), 2)

          cv2.arrowedLine(debug_img, arrow_cords[:2], arrow_cords[2:], (0,255,0), 10, tipLength=0.5)
      return debug_img

  def color_thresh(self, img):
 
     # Convert RGB to hls and threshold to binary image using S channel
     #1. Convert the image from RGB to hls
     #2. Apply threshold on S channel to get binary image
     #Hint: threshold on H to remove green grass
     ## TODO    
      hls_image = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)
      # Define range for yellow color and apply mask
      lower_yellow = np.array([15, 30, 115])  # Adjust these values
      upper_yellow = np.array([35, 204, 255])  # Adjust these values
      yellow_mask = cv2.inRange(hls_image, lower_yellow, upper_yellow)
      binary_output = np.zeros_like(yellow_mask)
      binary_output[yellow_mask > 0] = 1

      ####
      return binary_output

  def line_fit(self, binary_warped):
      """
      Find and fit lane lines
      """
      # Assuming you have created a warped binary image called "binary_warped"
      # Take a histogram of the bottom half of the image
      contours, _ = cv2.findContours(binary_warped, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
      frame_center_x = binary_warped.shape[1] / 2
      frame_center_y = binary_warped.shape[0] / 2
      max_radius = -1
      best_center = None
      if contours:
          largest_contour = max(contours, key=cv2.contourArea)
          M = cv2.moments(largest_contour)
          if M['m00'] != 0:
              cx = int(M['m10'] / M['m00'])
              cy = int(M['m01'] / M['m00'])
              (x,y), radius = cv2.minEnclosingCircle(largest_contour)
              best_center = (int(x), int(y))
              max_radius = int(radius)
              dx = cx - frame_center_x
              dy = cy - frame_center_y
              if abs(dx) < 50 or abs(dy) < 50:
                  return 0, (frame_center_x, frame_center_y, frame_center_x, frame_center_y+50), best_center, max_radius
              steering_angle = np.arctan2(dy, dx)
              steering_angle = -steering_angle
              end_x = int(cx + 100 * np.cos(steering_angle))
              end_y = int(cy + 100 * np.sin(steering_angle))
              return steering_angle, (cx, cy, end_x, end_y), best_center, max_radius
      return 0, (frame_center_x, frame_center_y, frame_center_x, frame_center_y+50), best_center, max_radius

class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0
        self.previous_error = 0
        self.last_time = time.time()

    def compute(self, setpoint, pv):
        now = time.time()
        dt = now - self.last_time
        error = setpoint - pv
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt

        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
       
        self.previous_error = error
        self.last_time = now

        return output
           
def lane_follower():
    rospy.init_node('lanenet_detector', anonymous=True)
    try:
       ld = lanenet_detector()
       rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    lane_follower()
