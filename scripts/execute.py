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

        # Initialization of the board state, AR library and multiple flags to keep track of the states
        # the turtlebot is in
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
        self.negative = 1
        self.base_tag = False
        self.game_ended = False
        self.initialized = True

        self.depth = 3 # depth for minimax search

        # Robot arm movement
        # set up ROS / OpenCV bridge
        self.bridge = cv_bridge.CvBridge()

        # Global variable to control for distance to object
        self.robotpos = 1 # should be 0

        # set the robot arm and gripper to its default state
        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")
        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")

        arm_joint_goal = [0.0, 0.0, 0.0, 0.0]
        self.move_group_arm.go(arm_joint_goal)
        self.move_group_arm.stop()
        rospy.sleep(5)

        gripper_joint_goal = [0.01, -0.01]
        self.move_group_gripper.go(gripper_joint_goal)
        self.move_group_gripper.stop()
        rospy.sleep(2)

        # Setup publishers and subscribers

        # subscribe to the robot's RGB camera data stream
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
                Image, self.image_callback)

        # Declare our node as a subscriber to the scan topic and
        #   set self.process_scan as the function to be used for callback.
        rospy.Subscriber("/scan", LaserScan, self.process_scan)

        # Publish robot movements
        self.robot_movement_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # return 0 if game not finished, 1 if robot won, 2 if human won, 3 if draw
    def check_game_done(self):
        lines = [[0,1,2], [3,4,5], [6,7,8], [0,3,6], [1,4,7], [2,5,8], [0,4,8], [2,4,6]]
        for l in lines:
            row = [self.board_state[l[0]], self.board_state[l[1]], self.board_state[l[2]]]
            count0 = row.count(0)
            count1 = row.count(1)
            count2 = row.count(2)
            if count1 == 3:
                return 1
            if count2 == 3:
                return 2

        for e in self.board_state:
            if e == 0: # empty spot so game not over
                return 0
        return 3 # draw


    def choose_next_action(self):
        # Choose the next action using the minimax algorithm and assign to self.next_tag
        decision_tree = StateTree(self.board_state, 1)
        self.next_tag = decision_tree.pickAction(self.depth)
        self.board_state[self.next_tag] = 1
        self.initialized = True
        print("chose next action {}".format(self.next_tag))
        return


    def image_callback(self, msg):
        # Image callback function
        if (not self.initialized):
            print("Initializing...")
            return

        if (self.game_ended):
            return

        if (self.going_back):
            # State that corresponds to the robot going back to the initial position to 
            # either center itself before moving to the board, or to get in the position
            # to scan the board for human's move

            # take the ROS message with the image and turn it into a format cv2 can use
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # turn the image into a grayscale
            grayscale_image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            # search for tags from DICT_4X4_50 in a GRAYSCALE image
            corners, ids, rejected_points = cv2.aruco.detectMarkers(grayscale_image, self.aruco_dict)

            if self.robotpos == 1: # Keep rotating until we see the tag we want
                my_twist = Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, 0.15))
                self.robot_movement_pub.publish(my_twist)
            
            # The initial position was marked with tag number 9
            if (ids is not None) and [9] in ids: # Only start moving when we see the tag we want
                self.base_tag = True
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
        elif (self.scanning): 
            # Only switch to self.scanning once we have reached the initial position
            # The scanning branch corresponds to the robot looking at the playing board, waiting
            # for the human player to indicate what their next move is by revealing the corresponding
            # AR tag

            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # turn the image into a grayscale
            grayscale_image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            # search for tags from DICT_4X4_50 in a GRAYSCALE image
            corners, ids, rejected_points = cv2.aruco.detectMarkers(grayscale_image, self.aruco_dict)

            # turn around to search for tags
            if (ids is not None) and [11] in ids and not self.parallel_to_wall:
                # The robot will center itself on tag number 11, located at the center of the board
                self.robotpos = 0
                index_of_id = ids.tolist().index([11])
                sum_x = 0
                sum_y = 0
                for i in range(4):
                    sum_x += corners[index_of_id][0][i][0]
                    sum_y += corners[index_of_id][0][i][1]
                cx = sum_x / 4 # Find the center of the tag
                cy = sum_y / 4
                # print("cx is", cx)

                if self.robotpos == 0: # Using proportional control to center itself
                    my_twist = Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, 0.01*(-cx + 160)))
                    self.robot_movement_pub.publish(my_twist)
                    if (cx > 140) and (cx < 175): # Once it's centered on tag 11, it will indicate that it is now parallel to the wall and start scanning
                        self.parallel_to_wall = True 
            elif (not self.parallel_to_wall): # In this branch the robot will turn until it sees tag 11
                my_twist = Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, 0.05))
                self.robot_movement_pub.publish(my_twist)
            if self.parallel_to_wall: # Once parallel to the wall, start scanning the board
                # print("Parallel to wall, waiting for tag")
                tag_chosen = -1
                if ids is not None:
                    for tag in self.tags:
                        if [tag] in ids:
                            self.tag_shown = True
                            tag_chosen = tag

                if (ids is not None) and (self.tag_shown):
                    # print("Entered the human action branch")
                    my_twist = Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, 0.05))
                    self.robot_movement_pub.publish(my_twist)
                    self.number_of_tags -= 1
                    self.board_state[tag_chosen] = 2
                    self.tags.remove(tag_chosen)

                    arm_joint_goal = [0.0, 0.0, 0.0, 0.0]
                    self.move_group_arm.go(arm_joint_goal)
                    self.move_group_arm.stop()
                    rospy.sleep(5)

                    gripper_joint_goal = [0.01, -0.01]
                    self.move_group_gripper.go(gripper_joint_goal, wait=True)
                    self.move_group_gripper.stop()
                    rospy.sleep(3)

                    # reset all the flags and change state, also check if the game has ended after the human's move
                    self.tag_shown = False
                    self.initialized = False
                    if self.check_game_done() == 2 or self.check_game_done == 3:
                        my_twist = Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, 0.05))
                        self.robot_movement_pub.publish(my_twist)
                        self.game_ended = True
                        return
                    # Choose next action based on human's move and reset all states
                    self.choose_next_action()
                    self.scanning = False
                    self.taking_to_tag = False
                    self.parallel_to_wall = False
                    self.robotpos = 0
                else: # While the tag is not determined, it will turn left and right to increase the angle of sight of the board
                    if self.parallel_to_wall:
                        my_twist = Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, 0.1 * self.negative))
                        self.robot_movement_pub.publish(my_twist)
                        rospy.sleep(1)
                        self.negative *= -1
        elif (self.taking_to_tag): # When we have the dumbell and travelling to the tag
            # take the ROS message with the image and turn it into a format cv2 can use
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # turn the image into a grayscale
            grayscale_image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            # search for tags from DICT_4X4_50 in a GRAYSCALE image
            corners, ids, rejected_points = cv2.aruco.detectMarkers(grayscale_image, self.aruco_dict)

            # Depending on where the robot wants to place its piece, it will center itself on the corresponding tag 
            # at the bottom of the board
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
                # print("saw the right tag")
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
        else: 
            # This branch corresponds to looking for the right dumbbell once the move has been chosen
            image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            # Color ranges for the 3 dumbells
            lower_blue = np.array([80, 60, 60])
            upper_blue = np.array([110, 255, 255])

            lower_green = np.array([30, 60, 60])
            upper_green = np.array([70, 255, 255])

            lower_pink = np.array([120, 60, 60])
            upper_pink = np.array([170, 255, 255])

            # this erases all pixels that aren't the right color
            if self.next_tag < 3:

                mask = cv2.inRange(hsv, lower_blue, upper_blue)

            if self.next_tag >= 3 and self.next_tag < 6:

                mask = cv2.inRange(hsv, lower_green, upper_green)

            if self.next_tag >=6 and self.next_tag < 9:

                mask = cv2.inRange(hsv, lower_pink, upper_pink)

            # using moments() function, the center of the pixels is determined
            M = cv2.moments(mask)

            if M['m00'] > 0 and self.robotpos == 0:
                self.color = True
                # center of the pixels in the image
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])

                #turns and moves towards the center of the pixels
                cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
                my_twist = Twist(linear=Vector3(0.05, 0, 0), angular=Vector3(0, 0, 0.0015*(-cx + 160)))
                self.robot_movement_pub.publish(my_twist)
            #turns until it finds the pixels of the right color
            elif self.robotpos == 0:
                my_twist = Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, 0.05))
                self.robot_movement_pub.publish(my_twist)


    def process_scan(self, data):

        if (not self.initialized):
            print("Initializing...")
            return

        if self.game_ended:
            return

        if (self.scanning):
            return
        elif (self.going_back): # This branch will stop the turtlebot when it's close enough to the initial position AR tag
            for i in range (5):
                right = data.ranges[-i]
                left = data.ranges[i]
                # If we are close enough to the AR tag
                if (((right <= 0.5 and right > 0.4) or (left <= 0.5 and left > 0.4)) and (self.robotpos == 1 and self.base_tag == True)): 
                    self.robotpos = 2
                    my_twist = Twist(linear=Vector3(0.0, 0, 0), angular=Vector3(0, 0, 0))
                    self.robot_movement_pub.publish(my_twist) # stop

                    # Turn roughly 180 degrees to face the board
                    my_twist = Twist(linear=Vector3(), angular=Vector3(0, 0, 0.7854))
                    self.robot_movement_pub.publish(my_twist)
                    rospy.sleep(3.5)
                    my_twist = Twist(linear=Vector3(), angular=Vector3())
                    self.robot_movement_pub.publish(my_twist)
                    rospy.sleep(3)

                    if (self.current_state == 0): # Go to scanning branch
                        self.scanning = True
                    else: # Take the dumbbell to the board
                        self.taking_to_tag = True
                        self.robotpos = 1

                    # Reset the state
                    self.going_back = False
        if (self.taking_to_tag): # This branch is responsible for stopping the robot when it's close enough to the board
            # print("taking to tag")
            for i in range (3):
                r = data.ranges[-i]
                l = data.ranges[i]
                if ((r <= 0.29 and r > 0.0) or (l <= 0.29 and l > 0.0)) and self.robotpos == 0: # If we are close enough to the AR tag
                    self.robotpos = 2
                    if self.next_tag < 3:
                        rospy.sleep(0.2)
                    my_twist = Twist(linear=Vector3(0.0, 0, 0), angular=Vector3(0, 0, 0))
                    self.robot_movement_pub.publish(my_twist) # stop

                    # Stick the dumbbell to the board
                    gripper_joint_goal = [0.01, -0.01]
                    self.move_group_gripper.go(gripper_joint_goal)
                    self.move_group_gripper.stop()
                    rospy.sleep(5)

                    #drive back and start turning
                    my_twist = Twist(linear=Vector3(-2, 0, 0), angular=Vector3(0, 0, 0.8))
                    self.robot_movement_pub.publish(my_twist)

                    rospy.sleep(3)

                    my_twist = Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, 0))
                    self.robot_movement_pub.publish(my_twist)
                    rospy.sleep(2)
                    
                    # Check if the game has ended after our move
                    if self.check_game_done() == 1 or self.check_game_done() == 3:
                        my_twist = Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, 1))
                        self.robot_movement_pub.publish(my_twist)
                        self.game_ended = True
                        rospy.sleep(2)
                        return

                    # Reset parameters and choose next action
                    self.going_back = True
                    self.current_state = 0
                    self.taking_to_tag = False
                    self.scanning = False
                    self.tag_shown = False
                    self.parallel_to_wall = False
                    self.negative = 1
                    self.base_tag = False
                    self.robotpos = 1

                    rospy.sleep(2)
        else: # This branch will stop the turtlebot once close enough to the dumbbell
            r = 0
            l = 0
            for i in range (7):
                r = data.ranges[-i]
                l = data.ranges[i]
                if ((r <= 0.23 and r > 0.18) or (l <= 0.23 and l >0.18)) and self.robotpos == 0 and self.taking_to_tag == False and self.color == True:
                    self.robotpos = 1
                    my_twist = Twist(linear=Vector3(0.0, 0, 0), angular=Vector3(0, 0, 0))
                    self.robot_movement_pub.publish(my_twist)

                    if self.next_tag < 3:
                        #picks up dumbell

                        arm_joint_goal = [0, math.radians(5.0), 0.0, math.radians(-5)]
                        self.move_group_arm.go(arm_joint_goal)
                        self.move_group_arm.stop()
                        rospy.sleep(7)

                        gripper_joint_goal = [-0.01, 0.01]
                        self.move_group_gripper.go(gripper_joint_goal, wait=True)
                        self.move_group_gripper.stop()
                        rospy.sleep(3)

                        arm_joint_goal = [0.0, math.radians(-10), math.radians(-50), math.radians(-30)]
                        self.move_group_arm.go(arm_joint_goal)
                        self.move_group_arm.stop()
                        rospy.sleep(5)

                    if self.next_tag >= 3 and self.next_tag < 6:
                        #picks up dumbell
                        print(0)
                        arm_joint_goal = [0, math.radians(40.0), 0.0, math.radians(-40)]
                        self.move_group_arm.go(arm_joint_goal)
                        self.move_group_arm.stop()
                        rospy.sleep(7)
                        print(1)

                        gripper_joint_goal = [-0.01, 0.01]
                        self.move_group_gripper.go(gripper_joint_goal, wait=True)
                        self.move_group_gripper.stop()
                        rospy.sleep(3)
                        print(2)

                        arm_joint_goal = [0.0, math.radians(30), math.radians(-50), math.radians(-70)]
                        self.move_group_arm.go(arm_joint_goal)
                        self.move_group_arm.stop()
                        rospy.sleep(5)

                    if self.next_tag >= 6 and self.next_tag < 9:
                        #picks up dumbell
                        arm_joint_goal = [0, math.radians(50.0), 0.0, math.radians(-50)]
                        self.move_group_arm.go(arm_joint_goal)
                        self.move_group_arm.stop()
                        rospy.sleep(7)

                        gripper_joint_goal = [-0.01, 0.01]
                        self.move_group_gripper.go(gripper_joint_goal, wait=True)
                        self.move_group_gripper.stop()
                        rospy.sleep(3)

                        arm_joint_goal = [0.0, math.radians(60), math.radians(-50), math.radians(-100)]
                        self.move_group_arm.go(arm_joint_goal)
                        self.move_group_arm.stop()
                        rospy.sleep(5)

                    # Moves back and starts turning
                    my_twist = Twist(linear=Vector3(-0.2, 0, 0), angular=Vector3(0, 0, 0.5))
                    self.robot_movement_pub.publish(my_twist)

                    rospy.sleep(3)
                    my_twist = Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, 0))
                    self.robot_movement_pub.publish(my_twist)
                    rospy.sleep(5)

                    #start looking for AR tag and reset all the states
                    self.going_back = True
                    self.current_state = 1
                    self.taking_to_tag = False
                    self.scanning = False
                    self.base_tag = False

    def run(self):
        # Give time for all publishers to initialize
        rospy.sleep(3)
        # Keep the code running
        rospy.spin()

if __name__ == "__main__":
    node = ExecuteAction()
    node.run()

