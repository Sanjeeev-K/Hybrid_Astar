#include "ros/ros.h"
#include "sensing.h"

class Planner{
	public:
		ros::NodeHandle n;
		ros::Subscriber sub_costmap;
		Sensing sensor();

		Planner();
		void Astar();
		void move();
};