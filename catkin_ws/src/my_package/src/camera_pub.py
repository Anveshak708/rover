#!/usr/bin/env python3

import rospy
import sys
import termios
import tty
from std_msgs.msg import Char

def get_key():
    """Reads a single key press from the keyboard."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key

def teleop_control():
    """Continuously publishes the last entered key until 's' is pressed."""
    rospy.init_node('camera_teleop', anonymous=True)
    pub = rospy.Publisher('camera_control', Char, queue_size=10)
    rate = rospy.Rate(10)  # 10Hz

    print("Use the following keys to control the cameras:")
    print("q: Clockwise 1st servo")
    print("a: Anticlockwise 1st servo")
    print("e: Clockwise 2nd servo")
    print("d: Anticlockwise 2nd servo")
    print("s: Stop all motion")

    last_key = None  # Store the last pressed key

    while not rospy.is_shutdown():
        key = get_key()
        if key in ['q', 'a', 'e', 'd', 's']:
            last_key = key
            rospy.loginfo(f"Sent command: {key}")

        if last_key:
            pub.publish(Char(data=ord(last_key)))  # Publish continuously

        if last_key == 's':  # Stop publishing on 's'
            last_key = None
            rospy.loginfo("Stopping all motion")
        
        if key == '\x03':  # Ctrl+C to exit
            break

        rate.sleep()

if __name__ == '__main__':
    try:
        teleop_control()
    except rospy.ROSInterruptException:
        pass


