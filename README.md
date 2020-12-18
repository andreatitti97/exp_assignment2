# Assignment 2 - Experimental Robotics lab 
# Andrea Tiranti 4856315

## Introduction
The purpouse of the project is to improve the first assignment implementing now a real robot model, that can be use in Gazebo, that switch as before between 3 state using a Final State Machine. The main change with respect to the previous assignment is the _PLAY_ state where it now implemented a CV algorithm for detecting and tracking a green ball in the simulated environment in Gazebo. 

## URDF model 
All files related to this topic can be found inside ../urdf folder.
The robot is very basic, it has a chassis with classic whels and a castor one for moving. The model should be similar to a Dog, so I added the neck and the head (a cylinder and a box respectively ) where I also attached a camera for the CV implementation. The neck is connected to the chassis with a fixed joint while the head is connected with a revolute joint for rotating the head and the camera ( 180 degrees ). All plugins used are inside the _robot.gazebo_ file which contains the camera plugin, the differential driver plugin and de facto allows the control of the robot wheels.
The other files in the folder are for the ball and the human, which were out of the scope of the assignment. 

## Software Architecture.
The main change respect the previous assignment concern the navigation server, which now is implemented using an action server -> _navigation_server_ which given a request by the client (represented by the cmd_manager) move the robot in the simulation. 
The new part is related to the detection and tracking of the ball, made inside the _ball_detect.py_ file, here is implemented the CV algorithm, more details below.
The project is based upon 2 namespace one for the robot -> _robot_ and one for the ball-> _ball_ .
# Installation
In order to use this package please install:
 The __smach__ library (for using FSM)
  ```
  $ sudo apt-get install ros-kinetic-smach-viewer
  ```
 The __ros_control__ package (control joints)
```
$ sudo apt-get install ros-[ROS_version]-ros-control ros-[ROS_version]-ros-controllers
$ sudo apt-get install ros-[ROS_version]-gazebo-ros-pkgs ros-[ROS_version]-gazebo-ros-control
```
The __openCV__ library (CV features)
```
$ sudo apt-get install python-opencv
```
The __numpy__ library (image processing features)
```
$ pip install numpy
```
It's necessary to have installed _ROS kinetic_ and _Gazebo 7_ .
# Running the project
Launch the project 
 ```
 $ roslaunch assignment2 gazebo_world.launch
 ```
See the architecture (another shell)
```
 $ rqt_graph 
```

## CV algorithm
The computer vision part is mainly inside the _ball_detection_ node where are implemented the detection and tracking of the green ball. The algorithm first allows to identify the center of the ball and draw a circle around it, then controlling the dimension of this circle we can send appropiate velocity commands to the robot for reaching the ball. This is the main concept for the _PLAY_ behaviour. All stuff are possible thanks to the OpenCv library. 
## File list
 Folder list:
 - __action__: which contains the definition of file.action files (Planning.action and Planning_ball.action) of the two action server : _navigation_action_ and go_to_point_ball.
 - __config__: contains the _motor_robot_dog.yaml_ file which defines the joint controller parameters
 - __launch__: contains one launch file:
           1. _gazebo_world.launch_ that launch all nodes of the project into 2 namespace, robot and ball.
- __msg__: contains the _ball_state.msg_ and _head_state.msg_ custom messages.
- __scripts__: holds the code of the following nodes:
            1. _ball_detect.py_ implements CV algorithms
            2. _cmd_manager.py_ This node is the main core of the system, it implements the FINITE STATE MACHINE for switch between the 3 state of the robot: NORMAL, PLAY, SLEEP.
            3. _navigation_action.py_ action server which navigate the robot to a position w.r.t the client request.
            4. _go_to_point_ball.py_ action server which navigate the ball to a position w.r.t the client request.
- __urdf__: all .xacro and .gazebo file for the simulation
- __world__: model of the world used in the simulation
#### Exectuables for testing
Two executable files for add the ball and remove the ball in order to test the algorithm 
- _add_ball.sh_ add the ball in position x = 3 and y = -5.
- _rmv_ball.sh_ remove the ball.
## System's limitation and possible improvements
The main limitations concern the control of the robot velocities w.r.t. the tracking of the ball, sometimes the robot overturn  if the ball move towards the robot, and some other little bugs in the movement of the robot. All this problems may be solved with a better PID controller.
Regarding the improvements can be implemented a way to randomize the position of the ball for better testing the behaviour and so modify the controller of the robot. The other big lack is the fact that the robot always have the head straight during _NORMAL_ and _SLEEP_  so the capacity of detection the ball is very limited. 
## Contacts
Andrea Tiranti - andrea.tiranti97@gmail.com
