#!/usr/bin/env python

## @file commandManager.py
#  This node includes the subsription to State and GetPosition publishers,
#  And implement a finite state machine 
#  which manages the information coming from the two publisher and changes the state of the system in according to them.
# \see getPosition.cpp
# \see Navigation.cpp
# \see State.cpp
 

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



## X position of the home 
# @param homeX here you can set the a priori X position of the house
homeX = -5
## Y position of the home 
# @param homeY here you can set the a priori Y position of the house
homeY = 7
## State variable
# @param state This is the state coming from State node (that's why is a string) and it can be ether play or NoInfo 
ball_detc = False
ball_check = False
ball_reach = False


##init action client for Navigation action server
client = actionlib.SimpleActionClient('/robot_reaching_goal', motion_plan.msg.PlanningAction)
# AATTT ho messo wait server nel main


## This function chose randomly the next state of the FSM
def decision():
    return random.choice(['goToNormal','goToSleep'])
 

## Callback function for the ballDetection subsriber.
# Which recives and handle a ball_state msg   
def callbackBall(data):
    global ball_detc, ball_check, ball_reach
    ball_detc = data.ball_detc
    ball_reach = data.ball_reach
    #rospy.loginfo( ball_check)
    if ball_detc == True and ball_check == False:
	rospy.loginfo("I'm updating the ball_checkvalue")
	ball_check = True
        rospy.loginfo("Ball detected !, current action interrupt")
	client.cancel_all_goals() 

class Normal(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['goToNormal','goToSleep','goToPlay'])
        self.rate = rospy.Rate(1)  # Loop at 200 Hz
	## @param joint_pub to move the head of the robot 
        self.counter = 0
        
    def execute(self,userdata):

        global ball_detc
        
        self.counter = random.randint(1,2) 
        # Creates a goal to send to the action server.
        goal = motion_plan.msg.PlanningGoal()

        while not rospy.is_shutdown():  

            if ball_check == True and ball_detc == True:
		rospy.loginfo("Start to track the ball")
                return 'goToPlay'
            if self.counter == 4:
                return 'goToSleep'           
            # request for the service to move in X and Y position
	    xRandomGoal = random.randrange(1,5,1)
	    yRandomGoal = random.randrange(1,5,1)
	    rospy.loginfo("I'm going to position x = %d y = %d", xRandomGoal, yRandomGoal)
            goal.target_pose.pose.position.x = xRandomGoal
            goal.target_pose.pose.position.y = yRandomGoal
	    client.send_goal(goal)
            client.wait_for_result()
	    rospy.loginfo("Goal reached")
            time.sleep(2)
	    self.rate.sleep()
            self.counter += 1
            
        return 'goToSleep' 
        
    

## It defines the SLEEP state which sleeps for a random period of time.
# Then it makes a request to the Navigation service to go to the home location.
# Finally it returns in the NORMAL state
class Sleep(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['goToNormal','goToSleep'])
        self.rate = rospy.Rate(200)  # Loop at 200 Hz

    def execute(self, userdata):       
        global homeX
        global homeY

        rospy.loginfo("I m in SLEEP mode")
        goal = motion_plan.msg.PlanningGoal()
        goal.target_pose.pose.position.x = homeX
        goal.target_pose.pose.position.y = homeY
        client.send_goal(goal)

        client.wait_for_result()       
	rospy.loginfo("Home reached")
        time.sleep(random.randint(3,6))
        self.rate.sleep()
        return 'goToNormal'

## Class that defines the PLAY state. 
# It move the robot in X Y location and then asks to go back to the user.
class Play(smach.State):
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
                rospy.loginfo("ball reached !!!")
		self.joint_pub.publish(0.785398) 
		time.sleep(5)
		self.joint_pub.publish(-0.785398)
		time.sleep(5)
		self.joint_pub.publish(0)
		time.sleep(5)
		rospy.loginfo("Finito di muovere la testa ")
		headMsg = head_status()
		headMsg.head_stop = True
		self.head_status.publish(headMsg) 
                
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


    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    outcome = sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()

