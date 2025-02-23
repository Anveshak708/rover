#! /usr/bin/env python3


import cv2
import numpy as np
import pyrealsense2 as rs
import os
import rospy
from std_msgs.msg import String

# Initialize ROS node
rospy.init_node('object_detection_node', anonymous=True)
detection_publisher = rospy.Publisher('/detection', String, queue_size=10)

# Configure Intel RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

# Start the pipeline
profile = pipeline.start(config)

# Get the depth scale for real-world measurement
depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()

# Arrow detection function
def arrow(frame, gray_frame, depth_image):
    MATCH_THRESHOLD = 0.8
    RIGHT_ARROW_PATH = os.path.expanduser("~/Downloads/right_arrow.png")
    LEFT_ARROW_PATH = os.path.expanduser("~/Downloads/left_arrow.png")

    right_arrow = cv2.imread(RIGHT_ARROW_PATH, cv2.IMREAD_GRAYSCALE)
    left_arrow = cv2.imread(LEFT_ARROW_PATH, cv2.IMREAD_GRAYSCALE)
    if right_arrow is None or left_arrow is None:
        rospy.logerr("Error loading template images")
        return frame, "none"

    def match_and_annotate(frame, gray_frame, depth_image, template_img, color, label):
        best_value = -1
        best_location = (-1, -1)
        best_scale = -1

        for scale in np.arange(0.1, 0.5, 0.027):
            resized_template = cv2.resize(template_img, (0, 0), fx=scale, fy=scale)
            result = cv2.matchTemplate(gray_frame, resized_template, cv2.TM_CCOEFF_NORMED)
            _, max_val, _, max_loc = cv2.minMaxLoc(result)

            if max_val > best_value and max_val > MATCH_THRESHOLD:
                best_value = max_val
                best_location = max_loc
                best_scale = scale

        if best_location != (-1, -1):
            w, h = int(template_img.shape[1] * best_scale), int(template_img.shape[0] * best_scale)
            top_left = best_location
            bottom_right = (top_left[0] + w, top_left[1] + h)
            cv2.rectangle(frame, top_left, bottom_right, color, 2)
            cv2.putText(frame, label, (top_left[0], top_left[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)

            # Get depth data from the bounding box
            x1, y1, x2, y2 = top_left[0], top_left[1], bottom_right[0], bottom_right[1]
            depth_crop = depth_image[y1:y2, x1:x2].astype(float)
            depth_crop = depth_crop * depth_scale  # Convert to meters
            dist, _, _, _ = cv2.mean(depth_crop)

            if dist <= 4:  # Only print and display if distance is less than or equal to 4 meters
                rospy.loginfo(f"{label} detected at {dist:.2f} meters")
                cv2.putText(frame, f"Depth: {dist:.2f} m", (top_left[0], bottom_right[1] + 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            return frame, label

        return frame, "none"

    frame, right_detected = match_and_annotate(frame, gray_frame, depth_image, right_arrow, (0, 255, 0), "right")
    frame, left_detected = match_and_annotate(frame, gray_frame, depth_image, left_arrow, (255, 0, 0), "left")

    if right_detected != "none":
        return frame, right_detected
    elif left_detected != "none":
        return frame, left_detected
    return frame, "none"

# Cone detection function
def detect_stop_cones(frame, hsv_frame, depth_image):
    lower_orange = np.array([0, 100, 100])
    upper_orange = np.array([30, 255, 255])

    orange_mask = cv2.inRange(hsv_frame, lower_orange, upper_orange)

    kernel = np.ones((5, 5), np.uint8)
    orange_mask = cv2.morphologyEx(orange_mask, cv2.MORPH_CLOSE, kernel)
    orange_mask = cv2.morphologyEx(orange_mask, cv2.MORPH_OPEN, kernel)

    contours, _ = cv2.findContours(orange_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if 300 < area < 15000:
            x, y, w, h = cv2.boundingRect(cnt)

            aspect_ratio = w / float(h)
            if 0.3 < aspect_ratio < 1.5:
                roi = frame[y:y + h, x:x + w]
                roi_hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

                lower_white = np.array([0, 0, 200])
                upper_white = np.array([180, 50, 255])
                white_mask = cv2.inRange(roi_hsv, lower_white, upper_white)

                white_percentage = np.sum(white_mask > 0) / (w * h) * 100

                if white_percentage > 5:
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 2)
                    cv2.putText(frame, "Stop Cone", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)

                    # Get depth data from the bounding box
                    depth_crop = depth_image[y:y + h, x:x + w].astype(float)
                    depth_crop = depth_crop * depth_scale  # Convert to meters
                    dist, _, _, _ = cv2.mean(depth_crop)

                    if dist <= 4:  # Only print and display if distance is less than or equal to 4 meters
                        rospy.loginfo(f"Stop Cone detected at {dist:.2f} meters")
                        cv2.putText(frame, f"Depth: {dist:.2f} m", (x, y + h + 20),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                        return frame, "cone"

    return frame, "none"

try:
    while not rospy.is_shutdown():
        # Wait for a coherent set of frames: color and depth
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        if not color_frame or not depth_frame:
            continue

        # Convert images to numpy arrays
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        # Prepare grayscale and HSV frames
        gray_frame = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        hsv_frame = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

        # Detect cones
        color_image, detection = detect_stop_cones(color_image, hsv_frame, depth_image)

        if detection == "none":
            # Detect arrows
            color_image, detection = arrow(color_image, gray_frame, depth_image)

        # Publish detection result
        detection_publisher.publish(detection)

        # Display the color image with bounding boxes
        cv2.imshow('Object Detection and Distance Measurement', color_image)

        # Break on pressing 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Stop the pipeline
    pipeline.stop()
    cv2.destroyAllWindows()

