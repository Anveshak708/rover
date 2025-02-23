#!/usr/bin/env python3

import cv2
import numpy as np
import time
import rospy
from std_msgs.msg import String
import os

MATCH_THRESHOLD = 0.8
DETECTION_TIME_THRESHOLD = 15  # in seconds

RIGHT_ARROW_PATH = os.path.expanduser("~/Downloads/right_arrow.png")
LEFT_ARROW_PATH = os.path.expanduser("~/Downloads/left_arrow.png")

def log_initialization():
    print("System initialization...")

def log_template_loaded(template_name):
    print(f"Template {template_name} loaded.")

def log_edge_detection():
    print("Performing edge detection...")

def log_contour_detection():
    print("Detecting contours...")

def log_template_matching():
    print("Performing template matching...")

def edge_detection(image):
    log_edge_detection()
    edges = cv2.Canny(image, 50, 150)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel, iterations=2)
    return edges

def to_grayscale_and_blur(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (7, 7), 0)
    return blurred

def detect_contours(image):
    log_contour_detection()
    processed = edge_detection(to_grayscale_and_blur(image))
    contours, _ = cv2.findContours(processed, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    return contours

def identify_arrow_tip(points, hull_indices):
    remaining_indices = [i for i in range(len(points)) if i not in hull_indices]
    for i in range(2):
        j = (remaining_indices[i] + 2) % len(points)
        if np.array_equal(points[j], points[(remaining_indices[i - 1] - 2 + len(points)) % len(points)]):
            return points[j]
    return np.array([-1, -1])

def determine_direction(approx, tip):
    left_points = sum(1 for pt in approx if pt[0][0] < tip[0][0])
    right_points = sum(1 for pt in approx if pt[0][0] > tip[0][0])

    if left_points > right_points and left_points > 4:
        return "Left"
    if right_points > left_points and right_points > 4:
        return "Right"
    return "None"

def calculate_angle(p1, p2):
    return np.degrees(np.arctan2(p1[1] - p2[1], p1[0] - p2[0]))

def process_frame(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    _, threshold = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY_INV)
    contours, _ = cv2.findContours(threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for cnt in contours:
        approx = cv2.approxPolyDP(cnt, 0.02 * cv2.arcLength(cnt, True), True)
        hull = cv2.convexHull(approx, returnPoints=False)

        if len(hull) > 4 and len(hull) < 6 and len(hull) + 2 == len(approx) and len(approx) > 6:
            tip = identify_arrow_tip(approx, hull)
            if not np.array_equal(tip, np.array([-1, -1])):
                direction = determine_direction(approx, tip)
                if direction != "None":
                    cv2.drawContours(frame, [approx], 0, (0, 255, 0), 5)
                    cv2.putText(frame, direction, (tip[0][0], tip[0][1]), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

def match_and_annotate(frame, template_img, color, label, detection_status, publisher):
    log_template_matching()
    gray_frame = to_grayscale_and_blur(frame)
    best_value = -1
    best_location = (-1, -1)
    best_scale = -1

    for scale in np.arange(0.1, 0.5, 0.027):
        resized_template = cv2.resize(template_img, (0, 0), fx=scale, fy=scale)
        result = cv2.matchTemplate(gray_frame, resized_template, cv2.TM_CCOEFF_NORMED)
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)

        if max_val > best_value and max_val > MATCH_THRESHOLD:
            best_value = max_val
            best_location = max_loc
            best_scale = scale

    if best_location != (-1, -1):
        w, h = int(template_img.shape[1] * best_scale), int(template_img.shape[0] * best_scale)
        top_left = best_location
        bottom_right = (top_left[0] + w, top_left[1] + h)
        cv2.rectangle(frame, top_left, bottom_right, color, 2)

        frame_center = (frame.shape[1] // 2, frame.shape[0] // 2)
        angle = calculate_angle(top_left, frame_center)
        print(f"{label} arrow detected at angle: {angle:.2f}")

        if label not in detection_status:
            detection_status[label] = {"start_time": time.time(), "detected": False}
        else:
            elapsed_time = time.time() - detection_status[label]["start_time"]
            if elapsed_time >= DETECTION_TIME_THRESHOLD:
                if not detection_status[label]["detected"]:
                    print(f"{label} arrow detected continuously for {DETECTION_TIME_THRESHOLD} seconds.")
                    detection_status[label]["detected"] = True
        publisher.publish(label)
    else:
        if label in detection_status:
            detection_status[label]["start_time"] = time.time()
            detection_status[label]["detected"] = False
        publisher.publish("None")

def init_video_capture():
    log_initialization()
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Cannot open webcam")
    return cap

def main():
    rospy.init_node('arrow_detector', anonymous=True)
    publisher = rospy.Publisher('/arrow_direction', String, queue_size=10)

    right_arrow = cv2.imread(RIGHT_ARROW_PATH, cv2.IMREAD_GRAYSCALE)
    left_arrow = cv2.imread(LEFT_ARROW_PATH, cv2.IMREAD_GRAYSCALE)
    if right_arrow is None or left_arrow is None:
        print("Error loading template images")
        return

    log_template_loaded("Right Arrow")
    log_template_loaded("Left Arrow")

    cap = init_video_capture()
    if not cap.isOpened():
        print("Error opening video capture")
        return

    detection_status = {}

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret or frame is None:
            print("Error capturing frame")
            break

        process_frame(frame)
        match_and_annotate(frame, right_arrow, (0, 255, 0), "Right", detection_status, publisher)
        match_and_annotate(frame, left_arrow, (255, 0, 0), "Left", detection_status, publisher)

        cv2.imshow("Video Feed", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

