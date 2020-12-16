#!/usr/bin/env python

# Python libs
import sys
import time

# numpy and scipy
import numpy as np
from scipy.ndimage import filters

import imutils

# OpenCV
import cv2

# Ros libraries
import roslib
import rospy

# Ros Messages
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from exp_assignment2.msg import Ball_state

VERBOSE = False


class image_feature:

    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
     ## initialize the node 
        rospy.init_node('ballDetection', anonymous=True)
     ## topic where we publish
     ## @param image_pub publisher that send the processed and compressed images 
        self.image_pub = rospy.Publisher("/output/image_raw/compressed",
                                         CompressedImage, queue_size=1)
     ## @param vel_pub pub for send to the command manager information reguarding the ball and the corraction to       		##apply to the robot 
        self.ball_state_pub = rospy.Publisher("ball_state",Ball_state, queue_size=1)

     ## @param joint_pub to move the head of the robot 
        self.joint_pub = rospy.Publisher("joint_head_controller/command",Float64,queue_size=1)

     ## @param vel_pub to move the whole robot 
        self.vel_pub = rospy.Publisher("cmd_vel",Twist, queue_size=1)

        ## subscribed Topic
	### @param subsriber to get the compressed images from the camera  
        self.camera_sub = rospy.Subscriber("camera1/image_raw/compressed",
                                           CompressedImage, self.callback,  queue_size=1)

	## @stop it's a stop condition when the robot is too close to the goal 
	self.stop = False

    def callback(self, ros_data):
        '''Callback function of subscribed topic. 
        Here images get converted and features detected'''
        if VERBOSE:
            print ('received image of type: "%s"' % ros_data.format)

        #### direct conversion to CV2 ####
        ## @param image_np is the image decompressed and converted in OpendCv
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # OpenCV >= 3.0:

	## set colours bound
        greenLower = (50, 50, 20)
        greenUpper = (70, 255, 255)

        blurred = cv2.GaussianBlur(image_np, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, greenLower, greenUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        #cv2.imshow('mask', mask)
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        center = None
        ## only proceed if at least one contour was found

        # fare l else che metta il messaggio ObjDet = false
        if len(cnts) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            # only proceed if the radius meets a minimum size
            if radius > 10:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                cv2.circle(image_np, (int(x), int(y)), int(radius),
                           (0, 255, 255), 2)
                cv2.circle(image_np, center, 5, (0, 0, 255), -1)
                
                msg = Ball_state()
                msg.ballDetected = True 		        
                self.ball_state_pub.publish(msg)
		# check if the robot is reached the object 

        	if self.stop == False: 
			
			rospy.loginfo("Ball track")
                	vel = Twist()
                	# 400 is the center of the image 
                	vel.angular.z = -0.002*(center[0]-400)
			# 100 is the radius that we want see in the image, which represent the desired disatance from the object 
                	vel.linear.x = -0.01*(radius-130)
                	self.vel_pub.publish(vel)
			if radius > 129:
				self.stop = True
		else:
			rospy.loginfo("goal reached")
			self.joint_pub.publish(0.785398) 
			time.sleep(5)
			self.joint_pub.publish(-0.785398)
			time.sleep(5)
			self.joint_pub.publish(0)
			time.sleep(5)
			self.stop = False

        else:
	     rospy.loginfo("Ball lost ")
	     msg = Ball_state()
             msg.ballDetected = False 
             self.ball_state_pub.publish(msg)
            

        # update the points queue
        # pts.appendleft(center)
        cv2.imshow('window', image_np)
        cv2.waitKey(2)

        # self.subscriber.unregister()


def main(args):
    '''Initializes and cleanup ros node'''
    ic = image_feature()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down ROS Image feature detector module")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)

