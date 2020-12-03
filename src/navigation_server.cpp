
#include "ros/ros.h"
#include "exp_assignment2/GoTo.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <iostream>

#define Xmax 30 //!< X dimension of the Map 
#define Ymax 30 //!< Y dimension of the Map  

#define homeX 10 //!< X coordinate of the home
#define homeY 10 //!< Y coordinate of the home

#define userX 2 //!< X coordinate of the user
#define userY 2 //!< Y coordinate of the user

/*!This function print useful information fo the user interface suchs as:
 * - If the robot going home (for sleepin)
 * - If the robot is going to some position (during play and normal)
 * - If the robot is returning to the user (after play)
*/

void printlogs(int x ,int y)
{

  	if((x == homeX)&&(y == homeY))
	{
     		ROS_INFO("Returning Home");
    	}
		else if ((x == userX)&&(y == userY))
	{
      		ROS_INFO("Going to user");
    	}
		else{ROS_INFO("Going to: x=%ld, y=%ld wait...",x,y);
   	}
}

/*! This function given a requested goal position first chek if is inside the map boundaries and then simulate the navigation to the posisition and retun it when 'reached'*/
bool goTo(exp_assignment2::GoTo::Request  &request,
         exp_assignment2::GoTo::Response &response)
{ 
	geometry_msgs::Twist vel;  	
	
  	printlogs((long int)request.x,(long int)request.y);
	if((request.x <= Xmax) && (request.y <= Ymax))
	{
		sleep(3); //this 'wait' is for simulate the robot moving 
	     	response.ok = true;
	     
	     	vel.linear.x = request.x	
	     	vel.angular.z = request.y;		
	    	 pub.publish(vel);

		response.currentX = request.x;
		response.currentY = request.y; 
      	}
	else
	{
        	response.ok = false; 
       	}
  
  return true;
}
/*! Main function where the server is initialized*/
int main(int argc, char **argv)
{

	ros::init(argc, argv, "navigation_server"); //node init
	ros::NodeHandle n;
	/// @param pub Publisher to send velocity commands on the topic cmd_vel
	pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 5);
			  
	ros::ServiceServer service = n.advertiseService("navigation", GoTo);
	ROS_INFO(" Ready to go in a new position.");
	ros::spin();

	return 0;
}
