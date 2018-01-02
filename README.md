# Networked Assisted Platooning - Automatic Control, Project Course

### Requirements
- ROS Kinetic
- Python 2.7

### Setup

#### Create workspace

	$ mkdir -p ~/catkin_ws/src
	$ cd ~/catkin_ws/
	$ catkin_make

#### Clone repository

	$ cd ~/catkin_ws/src
	$ git clone https://github.com/filipmat/PLATOON.git platoon
	$ cd ..
	$ catkin_make
	$ source devel/setup.bash

### Running

#### Platooning
To run platooning, execute the following commands in separate terminal windows:

	$ roscore
	$ rosrun platoon truck_publisher.py
	$ rosrun platoon datasender.py
	$ rosrun platoon platooning.py follower_id
	
	(optional)
	$ rosrun platoon truckplot.py
where follower_id (= 1 or 2) is the ID of the truck that is the follower truck.

What this does:
- The first command launches roscore.
- The second command starts a node that communicates with MoCap and publishes the truck positions to a topic.
- The third command starts a node that sends commands to the trucks.
- The fourth command starts the controller GUI.
- The fifth optional command starts a GUI that plots the positions of the trucks.

#### Running a single truck
To only run one truck that follows the path, replace the fourth command with 

	$ rosrun platoon onetruck.py truck_id
where truck_id is the ID of the truck. 
One could also simply run platooning.py but keeping the follower truck turned off. 

#### Notes
- At the time of writing, the car with orange sides has ID 1 and the one with white sides ID 2. The IDs are determined by the IP-addresses of the trucks. If the IP-addresses have been changed, this needs to be adjusted in datasender.py.
- If the IP-address to the MoCap has changed, this needs to be adjusted in truck_publisher.py.
- The reference path plotted by truckplot.py is only for visual aid. Changing the path in this GUI will not affect the controller. 

### Code structure


