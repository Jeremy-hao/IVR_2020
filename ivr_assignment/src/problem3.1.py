#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError


class image_converter:

    # Defines publisher and subscriber
    def __init__(self):
        # initialize the node named image_processing
        rospy.init_node('image_processing', anonymous=True)
        rate = rospy.Rate(30) # 30hz
        self.vision_endeffect=np.array([0.0,0.0,9.0])
        self.bridge = CvBridge()
        self.joint_sub = rospy.Subscriber("/robot/joint_states",JointState,self.callback)
        self.image_sub1 = rospy.Subscriber("/camera1/robot/image_raw", Image, self.image1_callback)
        rospy.sleep(1)
        self.image_sub2 = rospy.Subscriber("/camera2/robot/image_raw", Image, self.image2_callback)
        
        while not rospy.is_shutdown():
            rate.sleep()
    def callback(self,data):
        estimate_endeffect=self.calculated_forward_kinematics(data.position[0],data.position[1],data.position[2],data.position[3])
        print("estimate_endeffect")
        print(estimate_endeffect)
        print("vision_endeffect")
        print(self.vision_endeffect)
    def image1_callback(self,data):
        try:
            self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        base_frame = self.detect_yellow(self.cv_image1)
        red_frame = self.detect_red(self.cv_image1)
        pixel = self.pixel2meter(self.cv_image1)
        relative_red = base_frame - red_frame
        self.vision_endeffect[1]=-pixel*relative_red[0]
        self.vision=pixel*relative_red[1]
        '''red_mask = cv2.inRange(self.cv_image1, (0, 0, 100), (10, 10, 255))
        yellow_mask = cv2.inRange(self.cv_image1, (0, 100, 100), (0, 200, 200))
        y_line = cv2.line(red_mask, (base_frame[0], base_frame[1]), (red_frame[0], base_frame[1]), color=(255, 255, 255))
        z_line = cv2.line(red_mask, (base_frame[0], base_frame[1]), (base_frame[0], red_frame[1]), color=(255, 255, 255))
        center_line = cv2.line(red_mask, (base_frame[0], base_frame[1]), (red_frame[0], red_frame[1]), color=(255, 255, 255))
        cv2.imshow('Visualization Image 1, Target ZY, red Blob', red_mask)
        cv2.imshow('Visualization Image 1, Target ZY, yellow Blob', yellow_mask)
        cv2.imshow('window',self.cv_image1)
        cv2.waitKey(3)'''
    def image2_callback(self,data):
        try:
            self.cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        base_frame = self.detect_yellow(self.cv_image2)
        red_frame = self.detect_red(self.cv_image2)
        pixel = self.pixel2meter(self.cv_image2)
        relative_red = base_frame - red_frame
        self.vision_endeffect[0]=-pixel*relative_red[0]    
        self.vision_endeffect[2]=(pixel*relative_red[1]+self.vision)/2+0.6
    def calculated_forward_kinematics(self,ja1,ja2,ja3,ja4):
        #ja1, ja2, ja3, ja4 = detect_angles_blob(self,image1,image2)
        end_effector = np.array([3 * np.sin(ja1) * np.sin(ja2) * np.cos(ja3) * np.cos(ja4)
                                  + 3.5 * np.sin(ja1) * np.sin(ja2) * np.cos(ja3)
                                  + 3 * np.cos(ja1) * np.cos(ja4) *np.sin(ja3)
                                  + 3.5 * np.cos(ja1) * np.sin(ja3)
                                  + 3 * np.sin(ja1) * np.cos(ja2)*np.sin(ja4),

                                  -3 * np.cos(ja1) * np.sin(ja2) * np.cos(ja3) * np.cos(ja4)
                                  - 3.5 * np.cos(ja1) * np.sin(ja2) * np.cos(ja3)
                                  + 3 * np.sin(ja1) * np.cos(ja4) * np.sin(ja3)
                                  + 3.5 * np.sin(ja1) * np.sin(ja3)
                                  - 3 * np.cos(ja1) * np.cos(ja2)*np.sin(ja4),

                                  3 * np.cos(ja2) * np.cos(ja3) * np.cos(ja4)
                                  + 3.5 * np.cos(ja2) * np.cos(ja3) - 3 * np.sin(ja2) * np.sin(ja4) + 2.5
                                  ])
        return end_effector
    def detect_red(self,image):
        # Isolate the blue colour in the image as a binary image
        mask = cv2.inRange(image, (0, 0, 100), (10, 10, 255))
        # This applies a dilate that makes the binary region larger (the more iterations the larger it becomes)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=3)
        # Obtain the moments of the binary image
        M = cv2.moments(mask)
        # Calculate pixel coordinates for the centre of the blob
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        return np.array([cx, cy])
    # Detecting the centre of the blue circle
    def detect_blue(self,image):
        mask = cv2.inRange(image, (100, 0, 0), (255, 80, 80))
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=3)
        M = cv2.moments(mask)
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        return np.array([cx, cy])

    # Detecting the centre of the yellow circle
    def detect_yellow(self,image):
        mask = cv2.inRange(image, (0, 100, 100), (0, 200, 200))
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=3)
        M = cv2.moments(mask)
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        return np.array([cx, cy])
        
    def pixel2meter(self,image):
        # Obtain the centre of each coloured blob
        circle1Pos = self.detect_blue(image)
        circle2Pos = self.detect_yellow(image)
        # find the distance between two circles
        dist = np.sum((circle1Pos - circle2Pos)**2)
        return 2.5 / np.sqrt(dist)
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


