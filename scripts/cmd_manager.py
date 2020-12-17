#!/usr/bin/env python

## @file commandManager.py
#  This node is the main core of the system, it implements the FINITE STATE MACHINE for switch between the 3 state of the robot: NORMAL, PLAY, SLEEP. It also implement the behaviour of the robot for each state, which are:
# NORMAL, move randomly, SLEEP go to home and wait for some time, PLAY when robot sees the ball, have to follow it and move the head when reached. If during PLAY no ball detected switch to normal state.
# This node:
#- Is a client of the server for navigation 
#- Subscribe /ball_status
 

from __future__ import print_function

import roslib
import rospy
import smach
import smach_ros
import time
import random
import sys
import motion_plan.msg
import actionlib
import actionlib_tutorials.msg
import random 

from std_msgs.msg import String
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from exp_assignment2.msg import ball_status
from exp_assignment2.msg import head_status



## X coordinate of the HOME
# @param x_home: Set the x home position ( a priori knowledge )
x_home = 4
## Y coordinate of the HOME
# @param y_home: Set the y home position ( a priori knowledge )
y_home = 4
## Bool Variables
# @param ball_detc: if ball detected
ball_detc = False
# @param ball_check: if ball checked
ball_check = False
# @param ball_reach: if ball reached
ball_reach = False


## Initialization action client for the navigation service.
client = actionlib.SimpleActionClient('robot_reaching_goal', motion_plan.msg.PlanningAction)

## function for randomly choose NORMAL or SLEEP state.
def decision():
    return random.choice(['goToNormal','goToSleep'])
 

##  
def callbackBall(data):
    '''Callback function for the ball_detection subscriber. It receives the ball status and if ball detected it interrupt any action server goals for switching in PLAY state'''
    global ball_detc, ball_check, ball_reach
    ball_detc = data.ball_detc
    ball_reach = data.ball_reach
    if ball_detc == True and ball_check == False:
	ball_check = True
        rospy.loginfo("Ball detected")
	client.cancel_all_goals() 

class Normal(smach.State):
    ''' Class defining the NORMAL state for the Finite State Machine. It send randomly choosen position to the action server that moves the robot while checking if the ball is detected for switching to PLAY state. I set a limit number of iteration for this behaviour, after that switch to SLEEP state. '''
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['goToNormal','goToSleep','goToPlay'])
        self.rate = rospy.Rate(1)  # Loop at 200 Hz
        self.counter = 0
        
    def execute(self,userdata):

        global ball_detc
        
        self.counter = random.randint(1,2) 
        # Goal for the action server
        goal = motion_plan.msg.PlanningGoal()
	rospy.loginfo(" NORMAL state ")
        while not rospy.is_shutdown():  
 	# if ball detected go to PLAY, otherwise after some iterations go to SLEEP
            if ball_check == True and ball_detc == True:
                return 'goToPlay'
            if self.counter == 4:
                return 'goToSleep'           
            # Requesting to the action service to move into random posisitions
	    x_rand = random.randrange(1,5,1)
	    y_rand = random.randrange(1,5,1)
	    rospy.loginfo("Going to: x = %d y = %d", x_rand, y_rand)
            goal.target_pose.pose.position.x = x_rand
            goal.target_pose.pose.position.y = y_rand
	    client.send_goal(goal)
            client.wait_for_result()
	    rospy.loginfo("Goal reached")
            time.sleep(5)
	    self.rate.sleep()
            self.counter += 1
            
        return 'goToSleep' 
        
class Sleep(smach.State):
    ''' Class defining SLEEP state for the FSM. It implement the "returning" home task requesting to the navigation service to go to the home position, after some random time go to NORMAL state'''
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['goToNormal','goToSleep'])
        self.rate = rospy.Rate(200)  # Loop at 200 Hz

    def execute(self, userdata):       
        global x_home
        global y_home

        rospy.loginfo("SLEEP mode")
	# Sending the home posisition to the action service
        goal = motion_plan.msg.PlanningGoal()
        goal.target_pose.pose.position.x = x_home
        goal.target_pose.pose.position.y = y_home
        client.send_goal(goal)
	
        client.wait_for_result()       
	rospy.loginfo("Home reached")
	# wait at home before returning to NORMAL state
        time.sleep(random.randint(3,6))
        self.rate.sleep()
        return 'goToNormal'

class Play(smach.State):
    ''' Class defining PLAY mode for the FSM. It implement the lay behaviour which consist into reaching the ball, if possible (also implement the case where the robot lost the ball ). Then if ball reached turn the head +/- 90 deg. In fact the function publish to the topic /head_state and /joint_head_controller/command. '''
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['goToNormal','goToPlay'])
        
        self.rate = rospy.Rate(200)
	self.joint_pub = rospy.Publisher("joint_head_controller/command",Float64,queue_size=1)  
	self.head_status = rospy.Publisher("head_state",head_status, queue_size = 1)

    def execute(self, userdata):

        rospy.loginfo("I m in PLAY mode")
	global ball_detc, ball_check, ball_reach

	while True:
             if(ball_detc == False): 
		ball_check = False
		rospy.loginfo("Ball lost")
                return 'goToNormal' 
	     if(ball_reach == True):
                rospy.loginfo("Ball Reached")
		# Turn the head +90 deg 
		self.joint_pub.publish(0.785398) 
		time.sleep(5)
		# Turn the head -90 deg 
		self.joint_pub.publish(-0.785398)
		time.sleep(5)
		# Turn head to be in the upright position
		self.joint_pub.publish(0)
		time.sleep(5)
		rospy.loginfo(" End playing with the ball ")
		# Publishing the status of the head to the appropiate topic
		headMsg = head_status()
		headMsg.head_stop = True
		self.head_status.publish(headMsg) 
                
	     time.sleep(3)      
	

        
def main():
    rospy.init_node('cmd_manager')
    # Subscrive to the topic /ball_status for receiving information about the ball
    rospy.Subscriber("ball_status",ball_status, callbackBall)
    client.wait_for_server()

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['container_interface'])
    sm.userdata.sm_counter = 0

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('NORMAL', Normal(), 
                               transitions={'goToSleep':'SLEEP', 
                                            'goToPlay':'PLAY',
                                            'goToNormal':'NORMAL'})
        smach.StateMachine.add('SLEEP', Sleep(), 
                               transitions={'goToSleep':'SLEEP', 
                                            'goToNormal':'NORMAL'})
        smach.StateMachine.add('PLAY', Play(), 
                               transitions={'goToNormal':'NORMAL',
                                            'goToPlay':'PLAY'})


    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    outcome = sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()

