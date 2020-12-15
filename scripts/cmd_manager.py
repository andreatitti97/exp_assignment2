#!/usr/bin/env python

## @package cmd_manager
#  This node is the core of the system, subscribe to 'position' topic and 'cmd_string' topic, and is the client of the 'navigation' service.
#  In this node is implemented the Finite State Machine 
#  which manage the switch between states according to the information coming from the other nodes.
 
## Documentation for a function.
#
#  More details.

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

from std_msgs.msg import String
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from exp_assignment2.msg import ball_status

from exp_assignment2.srv import *

## robot X position variable
# @param X initial X pos of the robot
X = 0  
## robot Y position variable
# @param Y initial Y pos of the robot
Y = 0
## home X position
# @param homeX inital X pos of the home
homeX = 0.3
## home Y position
# @param homeY inital Y pos of the house
homeY = 0
## State variable
# @param state variable that save the PLAY state coming from the user, if sended 
#user_cmd = "NoCmd"
## User Position X
# @param userX is the x position of the user
#userX = 0.2
## User Position Y
# @param userY is the y position of the user
#userY = 0
ball_detc = False
ball_check = False
##initialization action.client for Navigation server
client = actionlib.SimpleActionClient('/robot_reaching_goal', motion_plan.msg.PlanningAction)

## Function that randomize the choice for choosing states NORMAL and SLEEP
def decision():
    return random.choice(['goToNormal','goToSleep'])

## Callback function for the ballDetection subsriber.
# Which recives and handle a ball_state msg   
def callbackBall(data):
    global ball_detc, ball_check
    ball_detc = data.ball_detc
    if ball_detc == True and ball_check == False:
	    ball_check = True
	    rospy.loginfo("ball detected")
	    client.cancel_all_goals()
    
 
## Callback for 'user_cmd' subscriber  
##def callbackSta(data): 
    ##rospy.loginfo(rospy.get_caller_id() + " Received cmd %s", data.data)
    ##global user_cmd 
    ##user_cmd = "play"


## Define state NORMAL
class Normal(smach.State):
    def __init__(self):
        # Init of the function
        smach.State.__init__(self, 
                             outcomes=['goToNormal','goToSleep','goToPlay'])
                             
        self.rate = rospy.Rate(1) 
        self.counter = 0
        
    def execute(self,userdata):

        global ball_detc
        
        self.counter = random.randint(1,2) #For randomize the switch to SLEEP state 
	# Goal for server        
	goal = motion_plan.msg.PlanningGoal()

	while not rospy.is_shutdown():  
            
            if ball_check == True: 
		rospy.loginfo("Tracking the ball")
                return 'goToPlay'
            if self.counter == 5:
                return 'goToSleep'           
            # Request to service for navigate
	    x_rand = random.randrange(1,5,1)
	    y_rand = random.randrange(1,5,1)
	    rospy.loginfo("reaching position x = %d y = %d", x_rand,y_rand)
	    goal.target_pose.pose.position.x = x_rand
            goal.target_pose.pose.position.y = y_rand
	    client.send_goal(goal)
	    client.wait_for_result()
	    rospy-loginfo(" Reach the Goal")
	    time.sleep(5)	
            self.counter += 1
            
        return 'goToSleep' 
        
    

## Define state SLEEP
class Sleep(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['goToNormal','goToSleep'])
        self.rate = rospy.Rate(200)  # Loop at 200 Hz

    def execute(self, userdata):
        
        global homeX
        global homeY

        rospy.loginfo("SLEEP mode")
        goal = motion_plan.msg.PlanningGoal()
        goal.target_pose.pose.position.x = homeX
        goal.target_pose.pose.position.y = homeY
        client.send_goal(goal)
        client.wait_for_result()  
	rospy.loginfo("Reach the Home")     
	time.sleep(random.randint(3,6))
        self.rate.sleep()
        return 'goToNormal'

## Define state PLAY
class Play(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['goToNormal','goToPlay'])
        
        self.rate = rospy.Rate(200)  # Loop at 200 Hz

    def execute(self, userdata):
        
    	rospy.loginfo("PLAY mode")

	while True:
		if(ball_detc == False):
			ball_check = False
			rospy.loginfo("Ball Lost")
			return 'goToNormal' 
			time.sleep(3)

              


        
def main():
    rospy.init_node('cmd_manager')
 
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
                               


    # Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
	main()
