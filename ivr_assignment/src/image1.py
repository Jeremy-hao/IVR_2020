#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
import utils2
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
    # initialize a publisher to send images from camera1 to a topic named image_topic1
    self.image_pub1 = rospy.Publisher("image_topic1",Image, queue_size = 1)
    # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
    self.bridge = CvBridge()
    self.image_sub2 = rospy.Subscriber("/camera2/robot/image_raw",Image,self.callback2)
    self.image_sub1 = rospy.Subscriber("/camera1/robot/image_raw",Image,self.callback1)
    self.blob_pub = rospy.Publisher("/blobs_pos", Float64MultiArray, queue_size=10)
    # initialize the bridge between openCV and ROS
    self.blobs = Float64MultiArray()
    self.blobs_history = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 2.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    
    t0 = rospy.get_time()
    while not rospy.is_shutdown():
        cur_time = np.array([rospy.get_time()])-t0
        joint2 = np.pi/2* np.sin(cur_time * np.pi/15)
        joint3 = np.pi/2* np.sin(cur_time * np.pi/18)
        joint4 = np.pi/2* np.sin(cur_time * np.pi/20)
        #robot_joint2_pub.publish(joint2)
        #robot_joint3_pub.publish(joint3)
        #robot_joint4_pub.publish(joint4)
        rate.sleep()

  def detect_red(self,image):
      # Isolate the blue colour in the image as a binary image
      mask = cv2.inRange(image, (0, 0, 100), (80, 80, 255))
      # This applies a dilate that makes the binary region larger (the more iterations the larger it becomes)
      kernel = np.ones((5, 5), np.uint8)
      mask = cv2.dilate(mask, kernel, iterations=3)
      # Obtain the moments of the binary image
      M = cv2.moments(mask)
      # Calculate pixel coordinates for the centre of the blob
      cx = int(M['m10'] / M['m00'])
      cy = int(M['m01'] / M['m00'])
      return np.array([cx, cy])
 

  # Detecting the centre of the green circle
  def detect_green(self,image):
      mask = cv2.inRange(image, (0, 100, 0), (80, 255, 80))
      kernel = np.ones((5, 5), np.uint8)
      mask = cv2.dilate(mask, kernel, iterations=3)
      M = cv2.moments(mask)
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


  # Calculate the conversion from pixel to meter
  def pixel2meter(self,image):
      # Obtain the centre of each coloured blob
      circle1Pos = self.detect_blue(image)
      circle2Pos = self.detect_yellow(image)
      # find the distance between two circles
      dist = np.sum((circle1Pos - circle2Pos)**2)
      return 2.5 / np.sqrt(dist)

    # Calculate the relevant joint angles from the image
  def detect_joint_angles1(self,image):
    a = self.pixel2meter(image)
    # Obtain the centre of each coloured blob 
    #center = a * self.detect_yellow(image)
    #circle1Pos = a * self.detect_blue(image) 
    #circle2Pos = a * self.detect_green(image) 
    #circle3Pos = a * self.detect_red(image)
    # Solve using trigonometry
    #print(circle2Pos[1]- circle1Pos[1])
    #print(circle2Pos[0] - circle1Pos[0])
    #ja1 = np.pi/2-np.arctan((circle2Pos[1]- circle1Pos[1])/(circle2Pos[0] - circle1Pos[0]))
    #ja2 = np.arctan2(circle1Pos[0]-circle2Pos[0], circle1Pos[1]-circle2Pos[1]) - ja1

    #ja3 = np.arctan((circle2Pos[0]-circle3Pos[0])/(circle2Pos[1]-circle3Pos[1])) - ja1
    #return np.array([ja1, ja3])
    center = self.detect_yellow(image)
    circle1Pos = a*self.detect_blue(image)
    circle2Pos = a*self.detect_green(image)
    circle3Pos = a*self.detect_red(image)
    ja1 = np.arctan((center[0]-circle1Pos[0])/(center[1]-circle1Pos[1]))
    #print(circle1Pos[0])
    print(circle1Pos[1])
    #print(circle2Pos[0])
    print(circle2Pos[1])
    ja2 = np.arctan((circle1Pos[0]-circle2Pos[0])/(circle1Pos[1]-circle2Pos[1]))
    ja3 = np.arctan((circle2Pos[0]-circle3Pos[0])/(circle2Pos[1]-circle3Pos[1]))-ja1-ja2
    return np.array([ja1, ja2, ja3])
    
  def detect_joint_angles2(self,image,j1):
    a = self.pixel2meter(image)
    # Obtain the centre of each coloured blob 
    #center = a * self.detect_yellow(image)
    circle1Pos = a * self.detect_blue(image) 
    circle2Pos = a * self.detect_green(image) 
    circle3Pos = a * self.detect_red(image)
    # Solve using trigonometry
    #ja1 = np.arctan2(center[0]- circle1Pos[0], center[1] - circle1Pos[1])
    ja2 = -np.arctan(np.cos(j1)*(circle1Pos[0]-circle2Pos[0])/(circle1Pos[1]-circle2Pos[1]))
    #ja3 = np.arctan2(circle2Pos[0]-circle3Pos[0], circle2Pos[1]-circle3Pos[1]) - ja2 - ja1
    return np.array([ja2])
  
  # Recieve data from camera 1, process it, and publish
  def callback1(self,data):
    # Recieve the image
    try:
      self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    
    # Uncomment if you want to save the image
    #cv2.imwrite('image_copy.png', self.cv_image1)
    #a = self.detect_joint_angles1(self.cv_image1)
    #print(a)
    #b = self.detect_joint_angles2(self.cv_image2,a[0])
    #print(b)
    
    if len(self.blobs.data) == 0:
        new_blobs = self.blobs_history
    else:
        new_blobs = self.blobs.data
        self.blobs_history = new_blobs
    base_frame=self.detect_yellow(self.cv_image1)
    #print(base_frame)
    green_detected=self.detect_green(self.cv_image1)
    #print(green_detected)
    relative_green = base_frame - green_detected
    img1_meters_ratio=self.pixel2meter(self.cv_image1)
    #print(img1_meters_ratio)
    #img1_meters_ratio=0.04080782932503862
    new_blobs[7] = img1_meters_ratio * relative_green[0]
    new_blobs[8] = img1_meters_ratio * relative_green[1]
    red_detected = self.detect_red(self.cv_image1)
    relative_red = base_frame - red_detected
    new_blobs[10] = img1_meters_ratio * relative_red[0]
    new_blobs[11] = img1_meters_ratio * relative_red[1]
    #print(img1_meters_ratio)
    self.blobs.data = new_blobs
    self.blob_pub.publish(self.blobs)
    print("YE:({0:.1f}, {1:0.2f}, {2:.2f}), BL:({3:.2f}, {4:.2f}, {5:.2f}), GR:({6:.2f}, {7:.2f}, {8:.2f}), RE:({9:.2f}, {10:.2f}, {11:.2f})".format(new_blobs[0], new_blobs[1], new_blobs[2], new_blobs[3], new_blobs[4], new_blobs[5], new_blobs[6], new_blobs[7], new_blobs[8], new_blobs[9], new_blobs[10], new_blobs[11]), end='\r')
    green_mask = cv2.inRange(self.cv_image1, (0, 100, 0), (80, 255, 80))
    y_line = cv2.line(green_mask, (base_frame[0], base_frame[1]), (green_detected[0], base_frame[1]), color=(255, 255, 255))
    z_line = cv2.line(green_mask, (base_frame[0], base_frame[1]), (base_frame[0], green_detected[1]), color=(255, 255, 255))
    center_line = cv2.line(green_mask, (base_frame[0], base_frame[1]), (green_detected[0], green_detected[1]), color=(255, 255, 255))
    cv2.imshow('Visualization Image 1, Target ZY, Green Blob', green_mask)
    cv2.waitKey(3)
    '''im1=cv2.imshow('window1', self.cv_image1)
    cv2.waitKey(1)
    # Publish the results
    try: 
      self.image_pub1.publish(self.bridge.cv2_to_imgmsg(self.cv_image1, "bgr8"))
    except CvBridgeError as e:
      print(e)'''

  def callback2(self,data):
    # Recieve the image
    try:
      self.cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    #a=self.angle_detection_blob(self.cv_image1,self.cv_image1)
    #print(a)
    # Uncomment if you want to save the image
    #cv2.imwrite('image_copy.png', cv_image)
    #im2=cv2.imshow('window2', self.cv_image2)
    #cv2.waitKey(1)
    if len(self.blobs.data) == 0:
        new_blobs = self.blobs_history
    else:
        new_blobs = self.blobs.data
        self.blobs_history = new_blobs
    base_frame=self.detect_yellow(self.cv_image1)
    green_detected=self.detect_green(self.cv_image1)
    relative_green = base_frame - green_detected
    img2_meters_ratio=self.pixel2meter(self.cv_image2)
    #img2_meters_ratio=0.04311306135592269
    new_blobs[6] = img2_meters_ratio * relative_green[0]
    print(img2_meters_ratio * relative_green[1])
    red_detected = self.detect_red(self.cv_image1)
    relative_red = base_frame - red_detected
    new_blobs[9] = img2_meters_ratio * relative_red[0]
    self.blobs.data = new_blobs
    self.blob_pub.publish(self.blobs)
    
  def angle_detection_blob(self,image1, image2):
    xyz_blob = utils2.create_xyz_table(self.cv_image1, self.cv_image1, "yellow")
    green_posn = xyz_blob.loc["green",]
    blue_posn = xyz_blob.loc["blue",]
    yellow_posn = xyz_blob.loc["yellow",]
    red_posn = xyz_blob.loc["red",]
    ja1_blob=0.1
    ja2_blob = np.arctan2((blue_posn[1] - green_posn[1]), -(blue_posn[2] - green_posn[2]))
    #ja2_blob = utils2.angle_normalization(ja2_blob)
    ja3_blob = np.arctan2((blue_posn[0] - green_posn[0]), (blue_posn[2] - green_posn[2]))
    #ja3_blob = utils2.angle_normalization(ja3_blob)
    ja4_blob = np.arctan2(green_posn[1] - red_posn[1], (green_posn[2] - red_posn[2]))-ja2_blob
    #ja4_blob = utils2.angle_normalization(ja4_blob) - ja2_blob
    return np.array([ja1_blob, ja2_blob, ja3_blob, ja4_blob])

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


