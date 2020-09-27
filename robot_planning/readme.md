# Real-time navigation using D* in unknown cluttered environment

![](/robot_planning/results/astar.gif)

## Prerequisites
The following software packages need to be installed before preceeding with this demo. The setup is installed on Ubuntu 16.04. 
1. Ros Kinetic: http://wiki.ros.org/kinetic/Installation
2. Turtlebot simulation packages:

```
sudo apt-get install ros-kinetic-turtlebot-gazebo ros-kinetic-turtlebot-apps ros-kinetic-turtlebot-rviz-launchers
```
3. Gazebo: http://gazebosim.org/tutorials?tut=install_ubuntu
4. Catkin: http://wiki.ros.org/catkin
5. Hokuyu Lidar Sensor: http://wiki.ros.org/turtlebot/Tutorials/indigo/Adding%20a%20lidar%20to%20the%20turtlebot%20using%20hector_models%20%28Hokuyo%20UTM-30LX%29
```
sudo apt-get install ros-kinetic-urg-node
```
6. Follow the ROS tutorials if you are new to ROS: http://wiki.ros.org/ROS/Tutorials

## Running the code
Download the repository and build it.
'''
cd ~/catkin_ws/src/ 
git clone https://github.com/sapan-ostic/Astar_RRT_Gazebo.git
cd ..
catkin_make
source devel/setup.bash
'''

Tip: Include source ~/catkin_ws/devel/setup.bash in the ~/.bashrc file to run it everytime a new terminal is created.

## Running the simulation
To simulate the turtlebot in Gazebo, run following launch commands in different terminals:

```
roslaunch robot_planning simulation.launch

roslaunch robot_planning planner.launch

roslaunch robot_planning controller.launch
```

Press ctrl+c to quit the simulation.
