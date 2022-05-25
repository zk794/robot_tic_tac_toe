#!/usr/bin/env python3

from turtle import color
import rospy, cv2, cv_bridge
import numpy as np
import os

from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from q_learning_project.msg import RobotMoveObjectToTag, QLearningReward, QMatrix
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan

import moveit_commander
import math

from states_tree import StateTree, StateNode

class ExecuteAction(object):
    def __init__(self):
        # Initialize this node
        rospy.init_node("execute_action")

        self.current_state = 0 # 0 for scanning, 1 for going back with the dumbbell

        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.taking_to_tag = False
        self.scanning = False
        self.going_back = True
        self.board_state = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.number_of_tags = 9
        self.tags = [0, 1, 2, 3, 4, 5, 6, 7, 8]
        self.next_tag = -1
        self.bottom_tag = -1
        self.tag_shown = False
        self.parallel_to_wall = False

        self.depth = 3 # depth for minimax search

        # Robot arm movement
        # set up ROS / OpenCV bridge
        self.bridge = cv_bridge.CvBridge()

        # initalize the debugging window
        # cv2.namedWindow("window", 1)
        # Global variable to control for distance to object
        self.robotpos = 1 # should be 0

        # set the robot arm and gripper to its default state
        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")
        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")

        # arm_joint_goal = [0.0, 0.0, 0.0, 0.0]
        # self.move_group_arm.go(arm_joint_goal)
        # self.move_group_arm.stop()
        # rospy.sleep(5)

        # gripper_joint_goal = [0.01, -0.01]
        # self.move_group_gripper.go(gripper_joint_goal)
        # self.move_group_gripper.stop()
        # rospy.sleep(2)

        # Setup publishers and subscribers

        # subscribe to the robot's RGB camera data stream
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
                Image, self.image_callback)

        # Declare our node as a subscriber to the scan topic and
        #   set self.process_scan as the function to be used for callback.
        rospy.Subscriber("/scan", LaserScan, self.process_scan)

        # Publish robot movements
        self.robot_movement_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        #setup global flags to be used when transitioning between movements
        self.initialized = True
        self.color = False


    def choose_next_action(self):
        # Choose the next action using the minimax algorithm and assign to self.next_tag
        decision_tree = StateTree(self.board_state, 1)
        self.next_tag = decision_tree.pickAction(self.depth)
        self.board_state[self.next_tag] = 1
        self.initialized = True
        print("chose next action {}".format(self.next_tag))
        return


    def image_callback(self, msg):

        if (not self.initialized):
            print("Initializing...")
            return

        if (self.going_back):
            print("Going back branch")
            # take the ROS message with the image and turn it into a format cv2 can use
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # turn the image into a grayscale
            grayscale_image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            # search for tags from DICT_4X4_50 in a GRAYSCALE image
            corners, ids, rejected_points = cv2.aruco.detectMarkers(grayscale_image, self.aruco_dict)

            if self.robotpos == 1: # Keep rotating until we see the tag we want
                my_twist = Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, 0.05))
                self.robot_movement_pub.publish(my_twist)

            if (ids is not None) and [9] in ids: # Only start moving when we see the tag we want
                # index_of_id = ids.tolist().index([self.tag])
                sum_x = 0
                sum_y = 0
                for i in range(4):
                    sum_x += corners[0][0][i][0]
                    sum_y += corners[0][0][i][1]
                cx = sum_x / 4 # Find the center of the tag
                cy = sum_y / 4

                if self.robotpos == 1: # Robot will move until it's close enough to the tag
                    my_twist = Twist(linear=Vector3(0.05, 0, 0), angular=Vector3(0, 0, 0.001*(-cx + 160)))
                    self.robot_movement_pub.publish(my_twist)
        elif (self.scanning): # Only switch to self.scanning once we have reached the initial position
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            print("In the scanning branch")
            # turn the image into a grayscale
            grayscale_image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            # search for tags from DICT_4X4_50 in a GRAYSCALE image
            corners, ids, rejected_points = cv2.aruco.detectMarkers(grayscale_image, self.aruco_dict)
            # turn around to search for tags
            print("ids are", ids)
            tag_chosen = -1
            for tag in self.tags:
                if [tag] in ids:
                    self.tag_shown = True
                    tag_chosen = tag

            if (ids is not None) and (self.tag_shown):
                print("Entered the human action branch")
                self.number_of_tags -= 1
                self.board_state[tag_chosen] = 2
                self.tags.remove(tag_chosen)

                self.tag_shown = False
                self.initialized = False
                self.choose_next_action()
                self.scanning = False
                self.taking_to_tag = False
                self.robotpos = 0
        elif (self.taking_to_tag): # When we have the dumbell and travelling to the tag
            print("taking to tag")
            # take the ROS message with the image and turn it into a format cv2 can use
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # turn the image into a grayscale
            grayscale_image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            # search for tags from DICT_4X4_50 in a GRAYSCALE image
            corners, ids, rejected_points = cv2.aruco.detectMarkers(grayscale_image, self.aruco_dict)
            print("ids are ", ids)
            print("robotpos is", self.robotpos)
            if self.next_tag % 3 == 0:
                self.bottom_tag = 10
            elif self.next_tag % 3 == 1:
                self.bottom_tag = 11
            else:
                self.bottom_tag = 12

            if self.robotpos == 1: # Keep rotating until we see the tag we want
                my_twist = Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, 0.05))
                self.robot_movement_pub.publish(my_twist)

            if (ids is not None) and [self.bottom_tag] in ids: # Only start moving when we see the tag we want
                print("saw the right tag")
                self.robotpos = 0
                index_of_id = ids.tolist().index([self.bottom_tag])
                sum_x = 0
                sum_y = 0
                for i in range(4):
                    sum_x += corners[index_of_id][0][i][0]
                    sum_y += corners[index_of_id][0][i][1]
                cx = sum_x / 4 # Find the center of the tag
                cy = sum_y / 4

                if self.robotpos == 0: # Robot will move until it's close enough to the tag
                    my_twist = Twist(linear=Vector3(0.05, 0, 0), angular=Vector3(0, 0, 0.001*(-cx + 160)))
                    self.robot_movement_pub.publish(my_twist)
        else: # looking for object
            image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            # Color ranges for the 3 dumbells
            # lower_blue = np.array([90, 60, 60])
            # upper_blue = np.array([90, 255, 255])

            lower_green = np.array([30, 60, 60])
            upper_green = np.array([45, 255, 255])

            # lower_pink = np.array([120, 60, 60])
            # upper_pink = np.array([170, 255, 255])

            # this erases all pixels that aren't the right color
            mask = cv2.inRange(hsv, lower_green, upper_green)

            # using moments() function, the center of the pixels is determined
            M = cv2.moments(mask)

            if M['m00'] > 0 and self.robotpos == 0:
                self.color = True
                # center of the pixels in the image
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])

                #turns and moves towards the center of the pixels
                cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
                my_twist = Twist(linear=Vector3(0.1, 0, 0), angular=Vector3(0, 0, 0.002*(-cx + 160)))
                self.robot_movement_pub.publish(my_twist)
            #turns until it finds the pixels of the right color
            elif self.robotpos == 0:
                my_twist = Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, 0.05))
                self.robot_movement_pub.publish(my_twist)


    def process_scan(self, data):

        if (not self.initialized):
            print("Initializing...")
            return
        
        if (self.scanning):
            if data.ranges[-10] == data.ranges[10]:
                self.parallel_to_wall = True
            if self.parallel_to_wall:
                my_twist = Twist(linear=Vector3(0.0, 0, 0), angular=Vector3(0, 0, 0))
                self.robot_movement_pub.publish(my_twist)
            else:
                my_twist = Twist(linear=Vector3(0.0, 0, 0), angular=Vector3(0, 0, 0.05))
                self.robot_movement_pub.publish(my_twist)
        elif (self.going_back):
            for i in range (5):
                r = data.ranges[-i]
                l = data.ranges[i]
                if ((r <= 0.5 and r > 0.4) or (l <= 0.5 and l > 0.4)) and self.robotpos == 1: # If we are close enough to the AR tag
                    print("in the scan going back branch - close enough")
                    self.robotpos = 2
                    my_twist = Twist(linear=Vector3(0.0, 0, 0), angular=Vector3(0, 0, 0))
                    self.robot_movement_pub.publish(my_twist) # stop

                    # Turn 180 degrees
                    my_twist = Twist(linear=Vector3(), angular=Vector3(0, 0, 0.7854)) 
                    self.robot_movement_pub.publish(my_twist)
                    # Turning time. Determined that 2.2 was better than 2 through trial and error
                    rospy.sleep(3) 
                    my_twist = Twist(linear=Vector3(), angular=Vector3())
                    self.robot_movement_pub.publish(my_twist)

                    if (self.current_state == 0):
                        self.scanning = True
                    else:
                        self.taking_to_tag = True
                        self.robotpos = 1

                    self.going_back = False
        if (self.taking_to_tag): # Taking to tag case
            print("taking to tag")
            for i in range (5):
                r = data.ranges[-i]
                l = data.ranges[i]
                if ((r <= 0.4 and r > 0.35) or (l <= 0.4 and l > 0.35)) and self.robotpos == 1: # If we are close enough to the AR tag
                    self.robotpos = 2
                    my_twist = Twist(linear=Vector3(0.0, 0, 0), angular=Vector3(0, 0, 0))
                    self.robot_movement_pub.publish(my_twist) # stop

                    # Put the dumbell down
                    # arm_joint_goal = [0.0, 0.0, 0.0, 0.0]
                    # self.move_group_arm.go(arm_joint_goal)
                    # self.move_group_arm.stop()
                    # rospy.sleep(7)

                    # gripper_joint_goal = [0.01, -0.01]
                    # self.move_group_gripper.go(gripper_joint_goal)
                    # self.move_group_gripper.stop()
                    # rospy.sleep(5)

                    # arm_joint_goal = [0.0, math.radians(-20), 0.0, 0.0]
                    # self.move_group_arm.go(arm_joint_goal)
                    # self.move_group_arm.stop()
                    # rospy.sleep(10)

                    #drive back and start turning
                    my_twist = Twist(linear=Vector3(-0.2, 0, 0), angular=Vector3(0, 0, 0.6))
                    self.robot_movement_pub.publish(my_twist)

                    rospy.sleep(2)

                    my_twist = Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, 0))
                    self.robot_movement_pub.publish(my_twist)
                    rospy.sleep(10)

                    # Reset parameters and choose next action
                    self.robotpos = 0
                    self.current_state = 0
                    self.going_back = True
                    self.taking_to_tag = False
                    rospy.sleep(2)
        else: # When we're looking for dumbells
            r = 0
            l = 0
            for i in range (10):
                r = data.ranges[-i]
                l = data.ranges[i]
                if ((r <= 0.22 and r > 0.2) or (l <= 0.22 and l >0.2)) and self.robotpos == 0 and self.taking_to_tag == False and self.color == True:
                    #stops
                    self.robotpos = 1
                    my_twist = Twist(linear=Vector3(0.0, 0, 0), angular=Vector3(0, 0, 0))
                    self.robot_movement_pub.publish(my_twist)

                    #picks up dumbell
                    # arm_joint_goal = [math.radians(min(r, l)), math.radians(20.0), 0.0, 0.0]
                    # self.move_group_arm.go(arm_joint_goal)
                    # self.move_group_arm.stop()
                    # rospy.sleep(5)

                    # gripper_joint_goal = [-0.01, 0.01]
                    # self.move_group_gripper.go(gripper_joint_goal, wait=True)
                    # self.move_group_gripper.stop()

                    
                        

                        

                    if self.next_tag < 3:
                        arm_joint_goal = [0.0, math.radians(-10), math.radians(-50), math.radians(-30)]
                        self.move_group_arm.go(arm_joint_goal)
                        self.move_group_arm.stop()
                        rospy.sleep(5)
                    if self.next_tag >= 3 and self.next_tag < 6:
                        arm_joint_goal = [0.0, math.radians(30), math.radians(-50), math.radians(-70)]
                        self.move_group_arm.go(arm_joint_goal)
                        self.move_group_arm.stop()
                        rospy.sleep(5)
                    if self.next_tag >= 6 and self.next_tag < 9:
                        arm_joint_goal = [0.0, math.radians(60), math.radians(-50), math.radians(-100)]
                        self.move_group_arm.go(arm_joint_goal)
                        self.move_group_arm.stop()
                        rospy.sleep(5)

           

                    # Moves back and starts turning
                    my_twist = Twist(linear=Vector3(-0.2, 0, 0), angular=Vector3(0, 0, 0.5))
                    self.robot_movement_pub.publish(my_twist)

                    rospy.sleep(1)
                    my_twist = Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, 0))
                    self.robot_movement_pub.publish(my_twist)
                    rospy.sleep(5)

                    #start looking for AR tag
                    self.going_back = True
                    self.current_state = 1
                    self.taking_to_tag = False
                    # self.color = False


    def run(self):
        # Give time for all publishers to initialize
        rospy.sleep(3)
        # Choose the first action
        # self.choose_next_action()
        rospy.spin()


if __name__ == "__main__":
    node = ExecuteAction()
    node.run()

# Master Node
#  - When scanning the board:
#     - If it sees the AR tag - assign 0
#     - All of its own moves - stored as 1
#     - All other tags that are not seen: assign 2

# - Update the state of board
# - Choose the action based on the new state (minimax algo)
# - Update the state again
# - Publish the action

# Execution Node:
# - Receive the aciton
# - Pick up the right dumbbell
# - Move correctly to the AR tag and stick it
# - Go back to the initial position and start scanning - use a flag to indicate the change of the state
