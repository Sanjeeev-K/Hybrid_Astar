#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"


class Sensor{
	public:
		bool collisionFlag;
		void scanCb(const sensor_msgs::LaserScan::ConstPtr& scan);
};		

void Sensor::scanCb(const sensor_msgs::LaserScan::ConstPtr& scan) {
	collisionFlag = false;
	for (int i = 3; i < 6 ; i++) { //scan->ranges.size()
 		if (scan->ranges[i] < 1) {
    		collisionFlag = true;
    	}
    }
}

class MoveRobot{
	private:
		Sensor scan;
		ros::NodeHandle n;
		ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 100);
		ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/laserscan", 50, &Sensor::scanCb, &scan);

	public:
		void move();	
};

void MoveRobot::move(){
	geometry_msgs::Twist vel;

	vel.linear.x = 0.0;
	vel.linear.y = 0.0;
	vel.linear.z = 0.0;
	vel.angular.x = 0.0;
	vel.angular.y = 0.0;
	vel.angular.z = 0.0;

	if (scan.collisionFlag == true){
		vel.angular.z = 0.5;
		vel.linear.x = 0;
	
	} else {
		vel.angular.z = 0.0;
		vel.linear.x = 0.5;	
	}

	pub.publish(vel);
}

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "Walker");
  		
	MoveRobot robot;
	ros::Rate loopRate(100);

	while (ros::ok()) {

		robot.move();

	    ros::spinOnce();
	    loopRate.sleep();
	}
}