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

class TargetEstimator:
    # Publish data
    def __init__(self):
        rospy.init_node('target_pos_cmd', anonymous=True)
        rate = rospy.Rate(30) # 30hz
        self.sphere_template = cv2.imread("src/ivr_assignment/src/sphere.png", 0)
        self.image_sub1 = rospy.Subscriber("/camera1/robot/image_raw", Image, self.image1_callback)
        self.image_sub2 = rospy.Subscriber("/camera2/robot/image_raw", Image, self.image2_callback)
        self.target_position_pub = rospy.Publisher("/target_position_estimate", Float64MultiArray, queue_size=10)
        self.targetx_position_pub = rospy.Publisher("/targetx_position_estimate", Float64, queue_size=10)
        self.targety_position_pub = rospy.Publisher("/targety_position_estimate", Float64, queue_size=10)
        self.targetz_position_pub = rospy.Publisher("/targetz_position_estimate", Float64, queue_size=10)
        self.yellow_blob_center_img2 = np.array([399, 533])
        self.yellow_blob_center_img1 = np.array([399, 533])
        self.to_meters_ratio_img1 = 0.04080782932503862  
        self.to_meters_ratio_img2 = 0.04311306135592269
        self.target_position = Float64MultiArray()
        self.target_position.data = [0.0, 0.0, 0.0]
        self.target_history = [0.0, 0.0, 0.0]
        self.bridge = CvBridge()
        self.target_y = Float64()
        self.target_z = Float64()
        self.target_x = Float64()
        # initialize a publisher to send joints' angular position to the robot
        t0 = rospy.get_time()
        while not rospy.is_shutdown():
            rate.sleep()

    def find_target(self, mask, template, target_history, zy=False):
        # Build template
        w, h = template.shape[::-1]

        # Apply template matching
        res = cv2.matchTemplate(mask, template, cv2.TM_CCOEFF)
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)

        # max_loc is the top left point of the match
        center = (max_loc[0] + w / 2, max_loc[1] + h / 2)

        # To detect the box:
        # box_template = cv2.imread("src/ivr_assignment/src/box.png", 0)
        if max_val < 6800000:
            if zy:
                return np.array([target_history[1], target_history[2]])
            else:
                return np.array([target_history[0], target_history[2]])

        if zy:
            target_history[1] = center[0]
        else:
            target_history[0] = center[0]
        target_history[2] = center[1]
        return np.array([center[0], center[1]])
    def image1_callback(self, data):
        try:
            self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        #cv2.imwrite('image_copy.png', self.cv_image1)
        # IMAGE 1
        # Color masks (BGR)
        orange_mask = cv2.inRange(self.cv_image1, (0, 40, 100), (20, 100, 150))
        kernel = np.ones((3, 3), np.uint8)
        orange_mask = cv2.erode(orange_mask, kernel, iterations=1)
        orange_mask = cv2.dilate(orange_mask, kernel, iterations=1)
        sphere_position = self.find_target(orange_mask, self.sphere_template, self.target_history, True)
        base_frame = self.yellow_blob_center_img1
        sphere_relative_distance = sphere_position - base_frame
        
        # Visualize movement of target
        '''y_line = cv2.line(orange_mask, (base_frame[0], base_frame[1]), ((int)(sphere_position[0]), base_frame[1]), color=(255, 255, 255))
        z_line = cv2.line(orange_mask, (base_frame[0], base_frame[1]), (base_frame[0], (int)(sphere_position[1])), color=(255, 255, 255))
        center_line = cv2.line(orange_mask, (base_frame[0], base_frame[1]), ((int)(sphere_position[0]), (int)(sphere_position[1])), color=(255, 255, 255))
        cv2.imshow('Visualization Image 1, Target ZY', orange_mask)
        cv2.imshow('Original Image 1, Target ZY', self.cv_image1)
        cv2.waitKey(3)'''

        # Publish the results
        self.target_position.data[1] = self.to_meters_ratio_img1 * sphere_relative_distance[0]
        self.target_position.data[2] = -(self.to_meters_ratio_img1 * sphere_relative_distance[1])
        #print("Target position: X={0:.2f}, Y={1:.2f}, Z={2:.2f}".format(self.target_position.data[0],self.target_position.data[1],self.target_position.data[2]), end='\r')
        self.target_y=self.target_position.data[1]
        self.target_z=self.target_position.data[2]
        print(self.target_position.data)
        self.targety_position_pub.publish(self.target_y)
        self.targetz_position_pub.publish(self.target_z)
        #self.target_position_pub.publish(self.target_position)

    def image2_callback(self, data):
        try:
            self.cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        orange_mask = cv2.inRange(self.cv_image2, (0, 40, 100), (20, 100, 150))
        kernel = np.ones((2, 2), np.uint8)
        orange_mask = cv2.erode(orange_mask, kernel, iterations=1)
        orange_mask = cv2.dilate(orange_mask, kernel, iterations=1)
        sphere_position = self.find_target(orange_mask, self.sphere_template, self.target_history, False)
        base_frame = self.yellow_blob_center_img2
        sphere_relative_distance = sphere_position - base_frame

        # Visualize movement of target
        '''x_line = cv2.line(orange_mask, (base_frame[0], base_frame[1]), ((int)(sphere_position[0]), base_frame[1]), color=(255, 255, 255))
        z_line = cv2.line(orange_mask, (base_frame[0], base_frame[1]), (base_frame[0], (int)(sphere_position[1])), color=(255, 255, 255))
        center_line = cv2.line(orange_mask, (base_frame[0], base_frame[1]), ((int)(sphere_position[0]), (int)(sphere_position[1])), color=(255, 255, 255))
        cv2.imshow('Visualization Image 2, Target ZX', orange_mask)
        cv2.imshow('Visualization Image 2, Yellow Blob ZX', self.cv_image2)
        cv2.waitKey(3)'''

        # Publish the results
        self.target_position.data[0] = self.to_meters_ratio_img2 * sphere_relative_distance[0]
        self.target_x=self.target_position.data[0]
        self.targetx_position_pub.publish(self.target_x)
        #self.target_position_pub.publish(self.target_position)

# run the code if the node is called
# call the class
def main(args):
    te = TargetEstimator()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)

