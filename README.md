# Networked Assisted Platooning Project
A project carried out in the course "EL2425  Automatic Control, Project Course, Smaller Course" at KTH, autumn semester 2017.

The project consisted of driving two small-scale trucks in a platoon. 


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
One could also simply run platooning.py but keep the follower truck turned off. 

#### Notes
- At the time of writing, the car with orange sides has ID 1 and the one with white sides ID 2. The IDs are determined by the IP-addresses of the trucks. If the IP-addresses have been changed, this needs to be adjusted in datasender.py.
- If the IP-address to the MoCap has changed, this needs to be adjusted in truck_publisher.py.
- The reference path plotted by truckplot.py is only for visual aid. Changing the path in this GUI will not affect the controller. 


### Code structure

#### truck_publisher.py
Continually fetches truck positions from the MoCap system and publishes the positions to a topic. Uses mocap_source_2.py to communicate with MoCap. 

#### datasender.py
Subscribes to a topic. The data published on the topic consists of the truck ID, and PWM signals for the motor and steering servo. When the subscriber receives data it sends it to the specified truck with sockets.

#### platooning.py
Creates a controller_platooning instance and a controllerGUI for interacting with the controller.

#### onetruck.py
Similar to platooning.py.

#### controller_platooning.py
A controller for platooning. Subscribes to the topic that publishes the truck positions (truck_publisher.py), and publishes the control inputs to the topic that accepts data to be sent to the trucks (datasender.py). 

Keeps a frenetpid instance for each truck that handles path following. Uses translator.py to translate the frenetpid output to PWM values that are sent to the truck.

Uses PID to control the distance between the trucks in seconds. The distance is calculated using the truck positions and the PID controller gives the motor PWM for the follower truck.

The class follows a certain structure so that the GUI can communicate with it. 

#### controller_onetruck.py
Similar to platooning.py.
Only one truck is considered so there is no speed regulation. The speed is kept constant and the truck follows the path using frenetpid.

#### frenetpid.py
Class for path tracking for one truck. Keeps a path instance for the reference path. Uses feedback linearization and PID in order to track the path. When calculating a control signal the input is the truck position and velocity and the output is a desired angular velocity of the truck. 

#### controllerGUI.py
A GUI for starting and stopping the controllers as well as changing control parameters on the fly. Keeps a controller instance that needs to be on a certain format. For example, controller_platooning.py and controller_onetruck.py both contain the methods stop() and start() which the GUI can call, but the logic is handled in the controller classes themselves. 

#### path.py
Class for keeping the reference path, which is a list of coordinates. Contains methods for various math operations related to the path, e.g. generating an elliptical path, calculate orthgonal distance to path, tangents, etc. 

Also contains a class for defining a path freehand in a GUI. 

#### truckplot.py
GUI for plotting the current truck positions and past trajectories. Subscribes to the topic that truck_publisher publishes to.

#### translator.py
Class for translating between from desired velocity and wheel angles to PWM values. For this it uses some measured values and interpolates to get the requested value. 

#### mocap_source_2.py
Provided to us at the start of the project. Class for communication with the MoCap system.
