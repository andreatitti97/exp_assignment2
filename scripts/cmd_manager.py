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
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry

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
user_cmd = "NoCmd"

## User Position X
# @param userX is the x position of the user
userX = 0.2
## User Position Y
# @param userY is the y position of the user
userY = 0

##initialization action.client for Navigation server
client = actionlib.SimpleActionClient('/robot_reaching_goal', motion_plan.msg.PlanningAction)

## Function that randomize the choice for choosing states NORMAL and SLEEP
def decision():
    return random.choice(['goToNormal','goToSleep'])

 
## Callback for 'user_cmd' subscriber  
def callbackSta(data): 
    rospy.loginfo(rospy.get_caller_id() + " Received cmd %s", data.data)
    global user_cmd 
    user_cmd = "play"


## Define state NORMAL
class Normal(smach.State):
    def __init__(self):
        # Init of the function
        smach.State.__init__(self, 
                             outcomes=['goToNormal','goToSleep','goToPlay'])
                             
        self.rate = rospy.Rate(1) 
        self.counter = 0
        
    def execute(self,userdata):

        global user_cmd
        
        self.counter = random.randint(1,2) #For randomize the switch to SLEEP state 
	# Goal for server        
	goal = motion_plan.msg.PlanningGoal()

	while not rospy.is_shutdown():  
            
            if user_cmd == "play":
                user_cmd = 'noInput'
                return 'goToPlay'
            if self.counter == 5:
                return 'goToSleep'           
            # Request to service for navigate
	    goal.target_pose.pose.position.x = random.randrange(1,5,1)
            goal.target_pose.pose.position.y = random.randrange(1,5,1)
	    client.send_goal(goal)
	    client.wait_for_result()	
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
	time.sleep(random.randint(3,6))gate to the home
        self.rate.sleep()
        return 'goToNormal'

## Define state PLAY
class Play(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['goToNormal','goToPlay'])
        
        self.rate = rospy.Rate(200)  # Loop at 200 Hz

    def execute(self, userdata):
        
        global X
        global Y 

    	rospy.loginfo("PLAY mode")
	time.sleep(3)

        return 'goToNormal'       


        
def main():
    rospy.init_node('cmd_manager')
 
    rospy.Subscriber("cmd_string", String, callbackSta)
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
                                            'goToNormal':'NORMAL'},
                               
        smach.StateMachine.add('SLEEP', Sleep(), 
                               transitions={'goToSleep':'SLEEP', 
                                            'goToNormal':'NORMAL'},
                               
        smach.StateMachine.add('PLAY', Play(), 
                               transitions={'goToNormal':'NORMAL',
                                            'goToPlay':'PLAY'},
                               


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