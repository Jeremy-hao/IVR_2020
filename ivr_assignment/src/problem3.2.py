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
        self.robot_joint1_pub = rospy.Publisher("/robot/joint1_position_controller/command", Float64, queue_size=10)
        self.robot_joint2_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
        self.robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
        self.robot_joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)
        self.current_joint = np.array([0.0,0.0,0.0,0.0])
        self.joint_sub = rospy.Subscriber("/robot/joint_states",JointState,self.callback)
        self.time_trajectory = rospy.get_time()
        self.time_previous_step = np.array([rospy.get_time()], dtype='float64')
        self.error = np.array([0.0, 0.0,0.0], dtype='float64')
        self.error_d = np.array([0.0, 0.0,0.0], dtype='float64')
        while not rospy.is_shutdown():
            #cur_time = np.array([rospy.get_time()])-t0
            #joint2 = np.pi/2* np.sin(cur_time * np.pi/15)
            #joint3 = np.pi/2* np.sin(cur_time * np.pi/18)
            #joint4 = np.pi/2* np.sin(cur_time * np.pi/20)
            #robot_joint2_pub.publish(joint2)
            #robot_joint3_pub.publish(joint3)
            #robot_joint4_pub.publish(joint4)
            
            rate.sleep()
    def callback(self,data):
        self.current_joint[0]=data.position[0]
        self.current_joint[1]=data.position[1]
        self.current_joint[2]=data.position[2]
        self.current_joint[3]=data.position[3]
        q=self.control_closed(self.current_joint)
        self.robot_joint1_pub.publish(q[0])
        self.robot_joint2_pub.publish(q[1])
        self.robot_joint3_pub.publish(q[2])
        self.robot_joint4_pub.publish(q[3])
    #print(self.current_joint)
    def control_closed(self,current_joint):
        # P gain
        K_p = np.array([[5, 0,0], [0,5, 0],[0,0,5]])
        # D gain
        K_d = np.array([[0.1, 0,0], [0,0.1, 0],[0,0,0.1]])
        # estimate time step
        cur_time = np.array([rospy.get_time()])
        dt = cur_time - self.time_previous_step
        self.time_previous_step = cur_time
        # robot end-effector position
        pos = self.calculated_forward_kinematics(current_joint[0],current_joint[1],current_joint[2],current_joint[3])
        # desired trajectory
        pos_d = self.actual_target_position(cur_time)    #flying_object_location(self,image1,image2,template,0.8)
        # estimate derivative of error
        self.error_d = ((pos_d - pos) - self.error) / (dt+1e-11)
        # estimate error
        self.error = pos_d - pos
        print(self.error)
        J_inv = np.linalg.pinv(self.calculate_jacobian(current_joint[0],current_joint[1],current_joint[2],current_joint[3]))  # calculating the psudeo inverse of Jacobian
        dq_d = np.dot(J_inv, (np.dot(K_d, self.error_d.transpose()) + np.dot(K_p,
                                                                              self.error.transpose())))  # control input (angular velocity of joints)
        q_d = (dt * dq_d)  # control input (angular position of joints)
        return current_joint+q_d
    def actual_target_position(self,curr_time):
        #curr_time = np.array([rospy.get_time() - self.time_trajectory])
        x_d = float((2.5 * np.cos(curr_time * np.pi / 15))+0.5)
        y_d = float(2.5 * np.sin(curr_time * np.pi / 15))
        z_d = float((1 * np.sin(curr_time * np.pi / 15))+7.0)
        m = np.array([[1, 0, 0], [0, -1, 0], [0, 0, 1]])
        return np.array([x_d,y_d,z_d])
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
    def calculate_jacobian(self,ja1,ja2,ja3,ja4):
        jacobian=np.array([[3*np.cos(ja1)*np.sin(ja2)*np.cos(ja3)*np.cos(ja4)
                            +3.5*np.cos(ja1)*np.sin(ja2)*np.cos(ja3)
                            -3*np.sin(ja1)*np.cos(ja4)*np.sin(ja3)
                            -3.5*np.sin(ja1)*np.sin(ja3)
                            +3*np.cos(ja1)*np.cos(ja2)*np.sin(ja4),

                            3*np.sin(ja1)*np.cos(ja2)*np.cos(ja3)*np.cos(ja4)
                            +3.5*np.sin(ja1)*np.cos(ja2)*np.cos(ja3)
                            -3*np.sin(ja1)*np.sin(ja2)*np.sin(ja4),

                            -3*np.sin(ja1)*np.sin(ja2)*np.sin(ja3)*np.cos(ja4)
                            -3.5*np.sin(ja1)*np.sin(ja2)*np.sin(ja3)
                            +3*np.cos(ja1)*np.cos(ja4)*np.cos(ja3)
                            +3.5*np.cos(ja1)*np.cos(ja3),

                            -3*np.sin(ja1)*np.sin(ja2)*np.cos(ja3)*np.sin(ja4)
                            -3*np.cos(ja1)*np.sin(ja4)*np.sin(ja3)
                          +3*np.sin(ja1)*np.cos(ja2)*np.cos(ja4)
                            ],
                          [

                            3*np.sin(ja1)*np.sin(ja2)*np.cos(ja3)*np.cos(ja4)
                            +3.5*np.sin(ja1)*np.sin(ja2)*np.cos(ja3)
                            +3*np.cos(ja1)*np.cos(ja4)*np.sin(ja3)
                            +3.5*np.cos(ja1)*np.sin(ja3)
                              +3*np.sin(ja1)*np.cos(ja2)*np.sin(ja4),

                            -3*np.cos(ja1)*np.cos(ja2)*np.cos(ja3)*np.cos(ja4)
                            -3.5*np.cos(ja1)*np.cos(ja2)*np.cos(ja3)
                            +3*np.cos(ja1)*np.sin(ja2)*np.sin(ja4),

                            +3*np.cos(ja1)*np.sin(ja2)*np.sin(ja3)*np.cos(ja4)
                            +3.5*np.cos(ja1)*np.sin(ja2)*np.sin(ja3)
                            +3*np.sin(ja1)*np.cos(ja4)*np.cos(ja3)
                            +3.5*np.sin(ja1)*np.cos(ja3),

                            +3*np.cos(ja1)*np.sin(ja2)*np.cos(ja3)*np.sin(ja4)
                            -3*np.sin(ja1)*np.sin(ja4)*np.sin(ja3)
                            -3*np.cos(ja1)*np.cos(ja2)*np.cos(ja4)
                          ],
                          [ 0,

                              -3*np.cos(ja3)*np.cos(ja4)*np.sin(ja2)
                              -3.5*np.cos(ja3)*np.sin(ja2)
                              -3*np.sin(ja4)*np.cos(ja2),

                              -3*np.sin(ja3)*np.cos(ja4)*np.cos(ja2)
                              -3.5*np.sin(ja3)*np.cos(ja2),

                              -3*np.cos(ja3)*np.sin(ja4)*np.cos(ja2)
                              -3*np.cos(ja4)*np.sin(ja2)]

        ])
        return jacobian

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


