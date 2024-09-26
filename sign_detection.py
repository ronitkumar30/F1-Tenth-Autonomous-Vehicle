def detect_red(self, img):
    # Convert the image from BGR to HSV
    hsv_image = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
    # Define range for red color and apply mask
    lower_red1 = np.array([0, 70, 50])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 70, 50])
    upper_red2 = np.array([180, 255, 255])

    mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
    red_mask = cv2.bitwise_or(mask1, mask2)

    # Check if red is significant
    if cv2.countNonZero(red_mask) > img.size * 0.1:
        return True
    return False
