#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import math
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError


class image_converter:

# Defines publisher and subscriber
    def __init__(self):
        # initialize the node named image_processing
        rospy.init_node('image_processing', anonymous=True)
        rate = rospy.Rate(30) # 30hz
        robot_joint2_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
        robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
        robot_joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)
        # initialize the bridge between openCV and ROS
        joint_test = [[0.1,0.1,0.1],[0.2,0.2,0.2],[0.3,0.3,0.3],[0.4,0.4,0.4],[0.5,0.5,0.5],[0.6,0.6,0.6],[0.7,0.7,0.7],[0.8,0.8,0.8],[0.9,0.9,0.9],[1,1,1]]
        i=0
        while not rospy.is_shutdown():
            i=i+1
            joint2 = joint_test[1][0]
            joint3 = joint_test[1][1]
            joint4 = joint_test[1][2]
            rospy.sleep(2)
            robot_joint2_pub.publish(joint2)
            robot_joint3_pub.publish(joint3)
            robot_joint4_pub.publish(joint4)
            rate.sleep()      

# call the class
def main(args):
    ic = image_converter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)


