#include <ros/ros.h>
#include "nav_msgs/Odometry.h"
#include <visualization_msgs/Marker.h>
#include <robot_planning/trajData.h>
#include <robot_planning/state.h>
#include <string>

#include <cmath>

class Visualize{\
public:
	std::vector<float> goal {0,0};
	robot_planning::state start;

	robot_planning::trajData planned_path;
	robot_planning::trajData travelled_path;
	
	ros::Publisher pub_planned_path;
	ros::Publisher pub_start_goal;
	ros::Publisher pub_travelled_path;

	Visualize(ros::NodeHandle &nh);  // constructor
	void RobotPathCbk(robot_planning::trajData::ConstPtr msg);
    void odomCbk(nav_msgs::Odometry::ConstPtr msg); // Callback for odom

	void PlotPlannedPath();
	void PlotTravelledPath();
	void PlotStartGoal();
	void tutorial();
	float distance(robot_planning::state &a, robot_planning::state &b);
};

Visualize::Visualize(ros::NodeHandle &nh){
	if(nh.hasParam("costmap_node/costmap/width")){
	   nh.getParam("planner/goal", goal);
	}  
	else
		ROS_ERROR("Did not find parameters !");

	pub_planned_path = nh.advertise<visualization_msgs::Marker>("visualization_plan", 1);
	pub_start_goal = nh.advertise<visualization_msgs::Marker>("visualization_start_goal", 1);
	pub_travelled_path = nh.advertise<visualization_msgs::Marker>("visualization_path",1);

	start.x = 0;
	start.y = 0;
	travelled_path.data.push_back(start);
}

void Visualize::RobotPathCbk(robot_planning::trajData::ConstPtr msg){  
  planned_path.data = msg->data;
}

float Visualize::distance(robot_planning::state &a, robot_planning::state &b){
	float dist = sqrt(pow(b.x-a.x, 2) + pow(b.y-a.y, 2));
	return fabs(dist);
}

void Visualize::odomCbk(nav_msgs::Odometry::ConstPtr msg){  

  robot_planning::state state;
  state.x = msg->pose.pose.position.x;
  state.y = msg->pose.pose.position.y;

  // std::cout << "distance: " << distance(travelled_path.data.back(), state) << std::endl;
  if (distance(travelled_path.data.back(), state) > 0.01)
  	travelled_path.data.push_back(state);
}

void Visualize::PlotStartGoal(){
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
	marker.ns = "basic_shapes";
    marker.id = 1;
    
    uint32_t shape = visualization_msgs::Marker::CYLINDER;
    marker.type = shape;
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = goal[0];
    marker.pose.position.y = goal[1];
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

        // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.01;

    marker.lifetime = ros::Duration();

    pub_start_goal.publish(marker);
}

void Visualize::PlotPlannedPath(){
	visualization_msgs::Marker	points, line_strip;
	line_strip.header.frame_id = "/map";
	line_strip.header.stamp = ros::Time::now();
	line_strip.ns = "visualizer";
	line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.pose.orientation.w = 1.0;

    line_strip.id = 0;

    line_strip.type = visualization_msgs::Marker::LINE_STRIP;

    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line_strip.scale.x = 0.05;
    
    // Line strip is blue
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;
    line_strip.lifetime = ros::Duration();

    // Create the vertices for the points and lines
    for (int i=0; i < planned_path.data.size(); i++)
    {
      geometry_msgs::Point p;
      robot_planning::state state = planned_path.data[i];
      p.x = state.x;
      p.y = state.y;
      p.z = 0;

      line_strip.points.push_back(p);
    }

    pub_planned_path.publish(line_strip);
}

void Visualize::PlotTravelledPath(){
	visualization_msgs::Marker	points, line_strip;
	line_strip.header.frame_id = "/map";
	line_strip.header.stamp = ros::Time::now();
	line_strip.ns = "visualizer";
	line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.pose.orientation.w = 1.0;

    line_strip.id = 0;

    line_strip.type = visualization_msgs::Marker::LINE_STRIP;

    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line_strip.scale.x = 0.05;
    
    // Line strip is blue
    line_strip.color.r = 1.0;
    line_strip.color.a = 1.0;
    line_strip.lifetime = ros::Duration();

    // Create the vertices for the points and lines
    for (int i=0; i < travelled_path.data.size(); i++)
    {
      geometry_msgs::Point p;
      robot_planning::state state = travelled_path.data[i];
      p.x = state.x;
      p.y = state.y;
      p.z = 0;

      line_strip.points.push_back(p);
    }

    pub_travelled_path.publish(line_strip);
}


int main( int argc, char** argv )
{
  ros::init(argc, argv, "visualizer");
  ros::NodeHandle nh;
  
  Visualize visualizer(nh);
  ros::Subscriber subs_path = nh.subscribe("/planned_path", 1, &Visualize::RobotPathCbk, &visualizer);
  

  std::string robot = "turtlebot";
  nh.getParam("robot", robot);

  ros::Subscriber subs_odom = nh.subscribe("/odometry/filtered", 1, &Visualize::odomCbk, &visualizer);

  if (robot=="turtlebot"){
    subs_odom = nh.subscribe("/odom", 1, &Visualize::odomCbk, &visualizer);
    ROS_INFO("Turtlebot selected");
  }
    
  else if (robot=="husky"){
    subs_odom = nh.subscribe("/odometry/filtered", 1, &Visualize::odomCbk, &visualizer);
    ROS_INFO("Husky robot selected");
  }  

  ros::Rate loop_rate(100); //update Frequency
  
  while (ros::ok())
  {
    visualizer.PlotPlannedPath();
    visualizer.PlotTravelledPath();
    visualizer.PlotStartGoal();
    ros::spinOnce();    
    loop_rate.sleep();
  }
  
  return 0;
}