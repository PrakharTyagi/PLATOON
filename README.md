# Model Truck Platooning

### Requirements
- ROS Kinetic

### Setup

#### Create workspace

	$ mkdir -p ~/catkin_ws/src
	$ cd ~/catkin_ws/
	$ catkin_make

#### Clone repository

	$ cd ~/catkin_ws/src
	$ git clone https://github.com/quilder/modeltruck-platooning.git modeltruck_platooning
	$ cd ..
	$ catkin_make
	$ source devel/setup.bash

### Running

#### Keyboard Controller
In order to run the keyboard controller the following commands need to be
executed in three seperate terminal windows.

	$ roscore
	$ rosrun modeltruck_platooning key_receiver.py {1/2}
	$ rosrun modeltruck_platooning keyboard.py

Using the argument {1/2} of key_receiver.py a truck can be selected. The arrow
keys can be used in the keyboard.py terminal to control the vehicle.

