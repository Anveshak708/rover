#! /usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class RealSenseSubscriber:
    def __init__(self):
        rospy.init_node("realsense_subscriber", anonymous=True)
        
        self.bridge = CvBridge()
        
        # Subscribe to RGB and Depth topics
        self.rgb_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.rgb_callback)
        self.depth_sub = rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.depth_callback)

        self.rgb_image = None
        self.depth_image = None

    def rgb_callback(self, data):
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except Exception as e:
            rospy.logerr("RGB Callback Error: {}".format(e))

    def depth_callback(self, data):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(data, "16UC1")  # Depth in mm
        except Exception as e:
            rospy.logerr("Depth Callback Error: {}".format(e))

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            if self.rgb_image is not None and self.depth_image is not None:
                cv2.imshow("RGB Image", self.rgb_image)
                
                # Normalize depth for visualization
                depth_vis = cv2.normalize(self.depth_image, None, 0, 255, cv2.NORM_MINMAX)
                depth_vis = np.uint8(depth_vis)
                cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
                cv2.imshow("Depth Image", depth_vis)

                key = cv2.waitKey(1) & 0xFF
                if key == 27:  # Exit on ESC
                    break
            rate.sleep()

        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        node = RealSenseSubscriber()
        node.run()
    except rospy.ROSInterruptException:
        pass

