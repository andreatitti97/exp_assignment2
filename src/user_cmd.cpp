/*!
 *\author Andrea Tiranti - S4856315
 *\date 10/11/2020
 *\version 1.0
 *\mainpage Assignment 1 Experimental Robotics Laboratory
 *\section intro_sec Introduction 
 *This software architecture simulate the behaviour of a 'dog robot' that switch beetween 3 different states: NORMAL, SLEEP, PLAY. The software architecture should manage the switch between this 3 states using a finite state machine. 
 *\subsection cmd_manager_sec Command Manager
 * This node handle the Finite State Machine and the navigation of the robot, subscribe 2 topics and it is client of the server navigation
 *\subsection navigation_server_sec Navigation Server
 * This server simulate the navigation. The cmd_manager (client) make a request of navigation performed inside the server that 'simulate' the robot moving.
 * INPUT: X and Y GOAL_positon. OUTPUT: Messagge of "goal reached" if possible, and the goal position.
 *\subsection user_cmd_sec User Command
 * This node simulate the user command (that is a voice cmd), publishing the state PLAY as a string to the cmd_manager, that when receive it should switch to PLAY mode if in NORMAL mode, if in SLEEP mode the cmd is rejected.
 *\subsection pos_generator_sec Position Generator
 * * This node publishes a random position which will be used from the command manager node.
 * In practice it simulates the pointing gesture from the user and the random travel in NORMAL mode as well.
 * \bug 
 *  - If in SLEEP state all PLAY command rejected. 
 *  - During action no posssibility to change the execution by the user.
 *  - Static home and user positions
 *  - Static Map definition
 
*/
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>


/*! Main function where the node is initialized, inside the loop is published th string "play" to the topic /cmd_string */
int main (int argc, char **argv)
{

	ros::init(argc,argv,"user_cmd"); //node initialization 

	ros::Publisher pub;//make the node publisher
	ros::NodeHandle nh;

	pub = nh.advertise<std_msgs::String>("cmd_string", 100);// publisher and topic initialization  

	ros::Rate loop_rate(1); 

	int time = 15; // initialization of the variable which will randomly choose how often to generate a 'play' message

	while(ros::ok()){

	std_msgs::String msg;// msg contain the string to pass to the cmd_manager  

	std::stringstream ss;//the string passed

	msg.data = "play"; 

	ROS_INFO("state detection: %s", msg.data.c_str());//print the string for the use

	pub.publish(msg);// publish to the cmd_manager

	ros::spinOnce();

	sleep(time);//random delay for publishing as request
	time = rand() % 8 + 15; // choose the next time iteration (random)


}

return 0;
}
