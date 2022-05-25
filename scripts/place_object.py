#!/usr/bin/env python3


# 28 cm off the ground for lowest piece

from turtle import color
import rospy, cv2, cv_bridge
import numpy as np
import os

from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
#from q_learning_project.msg import RobotMoveObjectToTag, QLearningReward, QMatrix
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan

import moveit_commander
import math


# Path of directory on where this file is located
path_prefix = os.path.dirname(__file__) + "/action_states/"
path_prefix_q = os.path.dirname(__file__) + "/"

def find_mask(hsv, upper, lower):
    mask = cv2.inRange(hsv, lower, upper)
    return mask

class place_object(object):
    def __init__(self):
        # Initialize this node
        rospy.init_node("place_object")

        
        # Initialize the object and the tag we need
        self.object = -1
        self.tag = -1
        # load DICT_4X4_50
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.taking_to_tag = False
        self.move_arm = True

        #based on the row of ar tags either be 1, 2, 3
        self.idpicker = -1

        # Robot arm movement
        # set up ROS / OpenCV bridge
        self.bridge = cv_bridge.CvBridge()

        # initalize the debugging window
        cv2.namedWindow("window", 1)
        # Global variable to control for distance to object
        self.robotpos = 0
                
        #set the robot arm and gripper to its default state 
        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")
        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")
                
       
        arm_joint_goal = [0.0, math.radians(-75), math.radians(0), math.radians(0)]
        self.move_group_arm.go(arm_joint_goal)
        self.move_group_arm.stop()
        rospy.sleep(7)

        gripper_joint_goal = [0.01, -0.01]
        self.move_group_gripper.go(gripper_joint_goal)
        self.move_group_gripper.stop()
        rospy.sleep(3)

        gripper_joint_goal = [-0.01, 0.01]
        self.move_group_gripper.go(gripper_joint_goal)
        self.move_group_gripper.stop()
        rospy.sleep(2)

       

        # Setup publishers and subscribers
 
        # subscribe to the robot's RGB camera data stream
        # self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
        #         Image, self.image_callback)

        # Declare our node as a subscriber to the scan topic and
        #   set self.process_scan as the function to be used for callback.
        rospy.Subscriber("/scan", LaserScan, self.process_scan)

        # Publish robot movements
        self.robot_movement_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        


    def choose_next_action(self):
        
        return 
    
    # def image_callback(self, msg):

        
    #     # take the ROS message with the image and turn it into a format cv2 can use
    #     img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        

    #     # turn the image into a grayscale
    #     grayscale_image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        
    #     self.idpicker = 1
    #     if self.taking_to_tag == True and self.move_arm == True:
    #         print(2)

            

    #         if self.idpicker == 0:
    #             arm_joint_goal = [0.0, math.radians(-10), math.radians(-50), math.radians(-30)]
    #             self.move_group_arm.go(arm_joint_goal)
    #             self.move_group_arm.stop()
    #             rospy.sleep(5)
    #         if self.idpicker == 1:
    #             arm_joint_goal = [0.0, math.radians(30), math.radians(-50), math.radians(-70)]
    #             self.move_group_arm.go(arm_joint_goal)
    #             self.move_group_arm.stop()
    #             rospy.sleep(5)
    #         if self.idpicker == 2:
    #             arm_joint_goal = [0.0, math.radians(60), math.radians(-50), math.radians(-100)]
    #             self.move_group_arm.go(arm_joint_goal)
    #             self.move_group_arm.stop()
    #             rospy.sleep(5)



    #         self.move_arm = False

    #     cv2.imshow("window", img) 
    #     cv2.waitKey(3)

        return
        


    def process_scan(self, data):
        self.idpicker = 1
        if self.taking_to_tag == True and self.move_arm == True:
            print(2)

            

            if self.idpicker == 0:
                arm_joint_goal = [0.0, math.radians(-10), math.radians(-50), math.radians(-30)]
                self.move_group_arm.go(arm_joint_goal)
                self.move_group_arm.stop()
                rospy.sleep(5)
            if self.idpicker == 1:
                arm_joint_goal = [0.0, math.radians(30), math.radians(-50), math.radians(-70)]
                self.move_group_arm.go(arm_joint_goal)
                self.move_group_arm.stop()
                rospy.sleep(5)
            if self.idpicker == 2:
                arm_joint_goal = [0.0, math.radians(60), math.radians(-50), math.radians(-100)]
                self.move_group_arm.go(arm_joint_goal)
                self.move_group_arm.stop()
                rospy.sleep(5)

            self.move_arm = False

        if self.taking_to_tag == False and self.move_arm == True:
            
            for i in range (5):
                r = data.ranges[-i]
                l = data.ranges[i]          
                print(r, l)  
                  
                if ((r <= 1 and r > 0) or (l <= 1 and l > 0)): # If we are close enough to the AR tag                  
                    
                    my_twist = Twist(linear=Vector3(0.0, 0, 0), angular=Vector3(0, 0, 0))
                    self.robot_movement_pub.publish(my_twist) # stop
                    

                    self.taking_to_tag = True
                    rospy.sleep(2)
                if (r >= 1.1 or l >= 1.1):
                    my_twist = Twist(linear=Vector3(0.05, 0, 0), angular=Vector3(0, 0, 0))
                    self.robot_movement_pub.publish(my_twist)  

        if self.move_arm == False:
            
            for i in range (5):
                r = data.ranges[-i]
                l = data.ranges[i]          
                print(r, l)  
                  
                if (((r <= 0.2 and r > 0) or (l <= 0.2 and l > 0)) and self.idpicker == 0) or (((r <= 0.33 and r > 0) or (l <= 0.33 and l > 0)) and self.idpicker == 1) or (((r <= 0.4 and r > 0) or (l <= 0.4 and l > 0)) and self.idpicker == 2): # If we are close enough to the AR tag                  
                    
                    my_twist = Twist(linear=Vector3(0.0, 0, 0), angular=Vector3(0, 0, 0))
                    self.robot_movement_pub.publish(my_twist) # stop
                    rospy.sleep(2)

                    gripper_joint_goal = [0.01, -0.01]
                    self.move_group_gripper.go(gripper_joint_goal)
                    self.move_group_gripper.stop()
                    rospy.sleep(2)

                    my_twist = Twist(linear=Vector3(-0.05, 0, 0), angular=Vector3(0, 0, 0))
                    self.robot_movement_pub.publish(my_twist) # stop
                    rospy.sleep(2)

                    my_twist = Twist(linear=Vector3(0.0, 0, 0), angular=Vector3(0, 0, 0))
                    self.robot_movement_pub.publish(my_twist) # stop
                    rospy.sleep(2)


                    
                else:
                    my_twist = Twist(linear=Vector3(0.05, 0, 0), angular=Vector3(0, 0, 0))
                    self.robot_movement_pub.publish(my_twist) # stop 
        return
            
    
    def run(self):
        # Give time for all publishers to initialize
        rospy.sleep(3)
        # Choose the first action
        self.choose_next_action()
        # Give time to initialize
        # rospy.sleep(5)
        # Keep the program running for 3 iterations
        rospy.spin()
        # while self.iteration < 4:
        #     rospy.spin()

if __name__ == "__main__":
    node = place_object()
    node.run()