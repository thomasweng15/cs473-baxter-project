cs473-baxter-project
====================

This repository contains Thomas, Vance, and Sam's semester project for Yale University's 
CPSC 473b Intelligent Robotics Lab course taught by Prof. Brian Scassellati. 


Dependencies / Requirements
---------------------------
* Ubuntu 12.04 LTS
* ROS (groovy distribution)
* Baxter Research Robot from Rethink Robotics
* Baxter Research SDK

For Gazebo simulation (not required):
* baxter_simulator (private access granted by Rethink Robotics)


Installation
------------
Note that Gazebo simulation software is not required to run this project. 

1.	Follow the instructions in the link to setup a ROS workspace and install the RSDK
https://github.com/RethinkRobotics/sdk-docs/wiki/Development-Workstation-Setup-Instructions

2.	Follow the instructions in the README.md to install the Gazebo simulator
https://github.com/RethinkRobotics/baxter_simulator

3. 	Navigate to the 'src' directory of your ROS workspace 
`cd ~/ros_ws/src`

4.	Clone this repository 
`git clone https://github.com/thomasweng15/cs473-baxter-project.git`

5. 	Build and install
	`catkin_make`
	`catkin_make install`


Environment
-----------



Running the Software
--------------------
1. Navigate to your ROS workspace
`cd ~/ros_ws`

2. Make sure Baxter is on, then run the Baxter shell file and enable Baxter
`./baxter.sh`
`rosrun baxter_tools enable_robot.py -e`

4. Run the 'glove' script to attach Baxter's compressor piece 
`rosrun cs473_baxter glove.py -g grip`

5. Run the main 'start' script run a trial. 
`rosrun cs473_baxter start.py`


File Structure
--------------
Here is a rundown of the file directory structure
and what is stored in the most important sections:
```
cs473-baxter-project/
	cs473_baxter/
		config/				YAML configuration file(s). 
		data/				Timestamped images and data for each trial. 
		scripts/			Python scripts for running the program. 
		stl/				STL files for 3D printing the compressor piece for Baxter's gripper.
		CMakeLists.txt
		package.xml
	cs473_gazebo/
		launch/				Gazebo launch file
		worlds/				Gazebo world model
		CMakeLists.txt
		package.xml
	.gitignore
	.gitmodules				Links the cs473vision git repo as a submodule to this one. 
	LICENSE
	README.md
```

The `scripts/` directory deserves special scrutiny:
```
scripts/
	cs473vision/
	...
```


License
-------
This software is licensed under the MIT License. See `LICENSE` for details. 
