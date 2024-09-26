def img_callback(self, data):
    try:
        # Convert a ROS image message into an OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)

    yellow_lane = self.color_thresh(cv_image)
    steering_angle, arrow_cords, circle_center, radius = self.line_fit(yellow_lane)
    desired_steering_angle = self.steering_pid.compute(setpoint=0, pv=steering_angle)

    self.drive_msg = AckermannDriveStamped()
    self.drive_msg.header.frame_id = "f1tenth_control"
    self.drive_msg.header.stamp = rospy.get_rostime()
    self.drive_msg.drive.speed = 0.5
    self.drive_msg.drive.steering_angle = desired_steering_angle

    self.ctrl_pub.publish(self.drive_msg)
