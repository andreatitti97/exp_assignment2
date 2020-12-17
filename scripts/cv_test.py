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
from exp_assignment2.msg import ball_status

VERBOSE = False


class image_feature:

    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
     	## initialize the node 
        rospy.init_node('ball_detection', anonymous=True)
     	## PUBLISHERs 
     	## @param image_pub: ros_publisher that send the compressed image. 
        self.image_pub = rospy.Publisher("/output/image_raw/compressed",
                                         CompressedImage, queue_size=1)
     	## @param ball_status_pub: ros_publisher where we can find the INFO of the ball -> 2 bool one for check detection one for check if ball reached. 
        self.ball_status_pub = rospy.Publisher("ball_status",ball_status, queue_size=1)

     	## @param joint_pub: ros_publisher where we publish commands for move head of the robot
        self.joint_pub = rospy.Publisher("joint_head_controller/command",Float64,queue_size=1)

     	## @param vel_pub: ros_publisher  that send velocity command to the topic "cmd_vel" 
        self.vel_pub = rospy.Publisher("cmd_vel",Twist, queue_size=1)

        ## SUBSCRIBERs 
	### @param camera_sub: for receive the compressed image from the camera  
        self.camera_sub = rospy.Subscriber("camera1/image_raw/compressed",
                                           CompressedImage, self.callback,  queue_size=1)

	## @stop boolean variable to stop the robot if is to close to the goal 
	self.stop = False

    def callback(self, ros_data):
        ''' Callback function for the subscribed topic "camera1/image_raw/compressed", here the image is converted and is implemented the feature detection.  '''

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
                msg.ballDetected = True 		        
                self.ball_status_pub.publish(msg)
		# check if the robot is reached the object 

        	if self.stop == False: 
			
			rospy.loginfo("Ball tracking")
                	vel = Twist()
                	# 400 is the center of the image 
                	vel.angular.z = -0.002*(center[0]-400)
			# 100 is the radius that we want see in the image, which represent the desired disatance from the object 
                	vel.linear.x = -0.01*(radius-130)
                	self.vel_pub.publish(vel)
			if radius > 129:
				self.stop = True
		else:
			rospy.loginfo("Goal Reached")
			# Turn the head +90 deg
			self.joint_pub.publish(0.785398) 
			time.sleep(5)
			# Turn the head -90 deg 
			self.joint_pub.publish(-0.785398)
			time.sleep(5)
			# Turn head to be in the upright position
			self.joint_pub.publish(0)
			time.sleep(5)
			self.stop = False

        else:
	     rospy.loginfo("Ball lost ")
	     msg = ball_status()
             msg.ballDetected = False 
             self.ball_status_pub.publish(msg)
            
        cv2.imshow('window', image_np)
        cv2.waitKey(2)

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

