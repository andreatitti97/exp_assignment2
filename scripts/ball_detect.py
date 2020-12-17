#!/usr/bin/env python

##@file ball_detect.py
# This node is responsible for the ball detection, so implement all CV algorithm for recognize the ball, follow it and reache it. In fact the node publish to 2 topics: 
#- image_raw/compressed (send compressed image)
#- /ball_status 
# And subscribe to:
#- image_raw/compressed
#- head_status (for controlling the head movement)


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
from exp_assignment2.msg import ball_status
from exp_assignment2.msg import head_status

VERBOSE = False


class image_feature:

    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
     	## Node Initialization 
        rospy.init_node('ball_detection', anonymous=True)

	## PUBLISHERED TOPICS
	## @param image_pub: ros_publisher that send the compressed image. 
        self.image_pub = rospy.Publisher("/output/image_raw/compressed",
                                         CompressedImage, queue_size=1)
     	## @param ball_status_pub: ros_publisher where we can find the INFO of the ball -> 2 bool one for check detection one for check if ball reached. 
        self.ball_status_pub = rospy.Publisher("ball_status",ball_status, queue_size=1)

     	## @param vel_pub: ros_publisher  that send velocity command to the topic "cmd_vel"
        self.vel_pub = rospy.Publisher("cmd_vel",Twist, queue_size=1)

        ## SUBSCRIBED TOPIC
	### @param camera_sub: for receive the compressed image from the camera
        self.camera_sub = rospy.Subscriber("camera1/image_raw/compressed",
                                           CompressedImage, self.callback,  queue_size=1)
	## @param head_state_sub: information about the head -> 1 bool indicating if head moving or not.
	self.head_state_sub = rospy.Subscriber("head_status",head_status, self.head_statusCallback, queue_size = 1)

	## @head_status: variable for check the head status (motion or not)
	self.head_status = True

	## @ball_reach: variable for check if the ball reach
	self.ball_reach = False 

	## Callback function updating head status 
    def head_statusCallback(self, data): 
	self.head_status = data.HeadMotionStop

	## Callback of the camera_sub subscriber where it's implemented the CV algorithm 
    def callback(self, ros_data):
        '''Callback function of subscribed topic. 
        Here images get converted and features detected'''
        if VERBOSE:
            print ('received image of type: "%s"' % ros_data.format)

        #### Direct conversion to CV2 ####
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
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        center = None
      
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
                
                msg = ball_status()
                msg.ball_detc = True
		msg.ball_reach = self.ball_reach
	        
                self.ball_status_pub.publish(msg)
 
		if self.head_status == True:	
			rospy.loginfo("Ball Detected") 
		        vel = Twist()
		        # 400 is the center of the image 
		        vel.angular.z = -0.002*(center[0]-400)
			# 100 is the radius that we want see in the image, which represent the desired disatance from the object 
		        vel.linear.x = -0.01*(radius-150)
		        self.vel_pub.publish(vel)
			self.ball_reach = False 
			if (radius>=149) :
				rospy.loginfo("Ball Reached")
				self.head_status = False
				self.ball_reach = True 
				msg.ball_reach = self.ball_reach		
				self.ball_status_pub.publish(msg) 

			

        else:
	     msg = ball_status()
             msg.ball_detc = False 
             self.ball_status_pub.publish(msg)
            

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

