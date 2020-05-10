#include <iostream>

#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "nav_msgs/OccupancyGrid.h"

using namespace std;

class Sensing{
   
  public:
    double grid_resolution;
    int grid_size;
    int grid_connections;
    ros::NodeHandle nh;

    struct sNode{
      bool bObstacle = false;       // Is the node an obstruction?
      bool bVisited = false;        // Have we searched this node before?
      float fGlobalGoal;            // Distance to goal so far
      float fLocalGoal;             // Distance to goal if we took the alternative route
      float x;                        // Nodes position in 2D space
      float y;


      vector<sNode*> vecNeighbours; // Connections to neighbours
      sNode* parent;                // Node connecting to this node that offers shortest parent
    };

    vector<vector<sNode>> nodes; // initializing map to represent all nodes

    Sensing();  // constructor
    void costmapCb(const nav_msgs::OccupancyGridConstPtr grid); // Callback for costmap


  private:
    void make_connections();      // add neighbors to the nodes 

};

