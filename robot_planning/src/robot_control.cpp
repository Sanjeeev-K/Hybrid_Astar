#include <iostream>
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <robot_planning/trajData.h>
#include "robot_planning/state.h"
#include <stack>  
#include <cmath>

// #define M_PI 3.1457

using namespace std;

class Controller{
   
  public:

    // Controller variables
    robot_planning::trajData planned_path;
    
    robot_planning::state targetState;
    robot_planning::state goalState;
    robot_planning::state currentState;
    robot_planning::state prevState;

    float desired_heading;
    float heading;

    float kp_lin = 0.0;
    float kd_lin = 0.0;
    float ki_lin = 0.0;

    float kp_ang = 0.0;
    float kd_ang = 0.0;
    float ki_ang = 0.0;

    float GOAL_EPS = 0.1;
    float SAMPLING_RADIUS = 0.5;
    float MAX_VEL = 0.05;
    bool TERMINATE = false;

    float prev_err_linear = 0;
    float d_err_linear = 0;
    float int_err_linear = 0;

    float prev_err_angular = 0;
    float d_err_angular = 0;
    float int_err_angular = 0;

    float linear_vel = 0;
    float angular_vel = 0;

    string robot = "turtlebot";

    ros::Publisher pub_vel;

    Controller(ros::NodeHandle &nh);  // constructor
    void odomCbk(nav_msgs::Odometry::ConstPtr msg); // Callback for odom
    void RobotPathCbk(robot_planning::trajData::ConstPtr msg);
    float distance();
    float goaldistance();
    void PIDController();
    void publishVel();
};

Controller::Controller(ros::NodeHandle &nh){ //constructor

  ROS_INFO("Controller module initialized ...");
  
  if(nh.hasParam("robot_control/kp_lin")){
    nh.getParam("robot_control/kp_lin", kp_lin);
    nh.getParam("robot_control/kd_lin", kd_lin);
    nh.getParam("robot_control/ki_lin", ki_lin);

    nh.getParam("robot_control/kp_ang", kp_ang);
    nh.getParam("robot_control/kd_ang", kd_ang);
    nh.getParam("robot_control/ki_ang", ki_ang);
    nh.getParam("robot_control/goalX", targetState.x);
    nh.getParam("robot_control/goalY", targetState.y);
    nh.getParam("robot_control/MAX_VEL", MAX_VEL); 
    nh.getParam("robot_control/SAMPLING_RADIUS", SAMPLING_RADIUS); 
    nh.getParam("robot_control/GOAL_EPS", GOAL_EPS);
    nh.getParam("robot", robot);   
  }

  else
    ROS_ERROR("Did not find parameters");

  if (robot=="turtlebot")
    pub_vel = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 100);
  else if (robot=="husky")  
    pub_vel = nh.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel", 100);
  else
    ROS_ERROR("robot_control.cpp : Velocity Publisher not assigned !");
}

void Controller::odomCbk(nav_msgs::Odometry::ConstPtr msg){
  
  currentState.x  = msg->pose.pose.position.x;
  currentState.y = msg->pose.pose.position.y;

  float qz = msg->pose.pose.orientation.z;
  float qw = msg->pose.pose.orientation.w;

  heading = atan2(2*(qw*qz), (1 - 2*qz*qz));

  PIDController();
}

float Controller::distance(){
  float dist;
  dist = sqrt(pow(targetState.x - currentState.x,2) + pow(targetState.y - currentState.y,2));
  
  return dist;  
}

float Controller::goaldistance(){
  float dist;
  dist = sqrt(pow(goalState.x - currentState.x,2) + pow(goalState.y - currentState.y,2));
  
  return dist;  
}

void Controller::RobotPathCbk(robot_planning::trajData::ConstPtr msg){
  
  planned_path.data = msg->data;
  targetState = (planned_path.data)[0];
  goalState = (planned_path.data).back();

  cout << "targetState.x: "<< targetState.x << " targetState.y: "<< targetState.y << endl;
  if (goaldistance() < GOAL_EPS)
    TERMINATE = true;
}

void Controller::PIDController(){
  for (int temp = 0; distance() < SAMPLING_RADIUS && temp < planned_path.data.size(); temp++)
    targetState = (planned_path.data)[temp];
  
  desired_heading = atan2(-currentState.y + targetState.y, -currentState.x + targetState.x); 
  // cout << "heading: " << heading << " desired_heading: " << desired_heading << endl;
  
  // if (desired_heading > M_PI)
  //   desired_heading -= 2*M_PI;
  
  // else if (desired_heading < -M_PI)
  //   desired_heading += 2*M_PI;

  float err_linear = distance();
  d_err_linear = err_linear - prev_err_linear;
  int_err_linear = int_err_linear + err_linear;
  
  // cout << "err_linear: " << err_linear << endl;

  float err_angular = -heading + desired_heading;
  
  if (err_angular >= M_PI)
    err_angular -= 2*M_PI;
  
  else if (err_angular <= -M_PI)
    err_angular += 2*M_PI;
        
  // cout << "heading: " << heading << " desired_heading: " << desired_heading << endl;

  d_err_angular = err_angular - prev_err_angular;
  int_err_angular = int_err_angular + err_angular;
  
  linear_vel = kp_lin * err_linear + kd_lin * d_err_linear + ki_lin*int_err_linear;
  angular_vel = kp_ang * err_angular + kd_ang * d_err_angular + ki_ang*int_err_angular;
  
  cout << "err_linear: " << err_linear << " err_angular: "<< err_angular << endl;

  linear_vel = max(min(linear_vel,  MAX_VEL) ,-MAX_VEL);
  cout << "linear_vel: " << linear_vel << " angular_vel: "<< angular_vel << endl;
  prev_err_linear = err_linear;
  prev_err_angular = err_angular;


}

void Controller::publishVel(){
  geometry_msgs::Twist cmd;

  cmd.linear.x = linear_vel;
  cmd.linear.y = 0;
  cmd.linear.z = 0;

  cmd.angular.x = 0;
  cmd.angular.y = 0;
  cmd.angular.z = angular_vel;

  // cout<< "linear_vel: " << linear_vel << " angular_vel: " << angular_vel <<endl; 
  pub_vel.publish(cmd);
}

int main(int argc, char **argv){

  ros::init(argc, argv, "robot_control");
  ros::NodeHandle nh;
  
  Controller robot_control(nh);

  string robot = "turtlebot";
  nh.getParam("robot", robot);

  ros::Subscriber subs_odom = nh.subscribe("/odometry/filtered", 1, &Controller::odomCbk, &robot_control);
  
  if (robot=="turtlebot")
    subs_odom = nh.subscribe("/odom", 1, &Controller::odomCbk, &robot_control);
  else if (robot=="husky")
    subs_odom = nh.subscribe("/odometry/filtered", 1, &Controller::odomCbk, &robot_control);

  ros::Subscriber subs_path = nh.subscribe("/planned_path", 1, &Controller::RobotPathCbk, &robot_control);
  ros::Rate loop_rate(100); // Control Frequency

  while(ros::ok() and !robot_control.TERMINATE){

    // cout << robot_control.TERMINATE << endl;
    
    robot_control.publishVel();
    ros::spinOnce();
    
    loop_rate.sleep();
  }
  
  return 0;
}