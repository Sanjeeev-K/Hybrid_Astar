#include<iostream>
#include<vector>
#include <math.h>
#include<queue>
#include "matplotlibcpp.h"
#include "hybrid_breadth_first.h"
namespace plt = matplotlibcpp;
using namespace std;
#define PI 3.14159265
// #define PI 3.17

double SPEED = 1.45;
double LENGTH = 0.5;
int num_theta = 90;
double delta_step = 5;

// Sets up maze grid
int X = 1;
int _ = 0;

vector<vector<int>> GRID = {
  {_,X,X,_,_,_,_,_,_,_,X,X,_,_,_,_,},
  {_,X,X,_,_,_,_,_,_,X,X,_,_,_,_,_,},
  {_,X,X,_,_,_,_,_,X,X,_,_,_,_,_,_,},
  {_,X,X,_,_,_,_,X,X,_,_,_,X,X,X,_,},
  {_,X,X,_,_,_,X,X,_,_,_,X,X,X,_,_,},
  {_,X,X,_,_,X,X,_,_,_,X,X,X,_,_,_,},
  {_,X,X,_,X,X,_,_,_,X,X,X,_,_,_,_,},
  {_,X,X,X,X,_,_,_,X,X,X,_,_,_,_,_,},
  {_,X,X,X,_,_,_,X,X,X,_,_,_,_,_,_,},
  {_,X,X,_,_,_,X,X,X,_,_,X,X,X,X,X,},
  {_,X,_,_,_,X,X,X,_,_,X,X,X,X,X,X,},
  {_,_,_,_,X,X,X,_,_,X,X,X,X,X,X,X,},
  {_,_,_,X,X,X,_,_,X,X,X,X,X,X,X,X,},
  {_,_,X,X,X,_,_,X,X,X,X,X,X,X,X,X,},
  {_,X,X,X,_,_,_,_,_,_,_,_,_,_,_,_,},
  {X,X,X,_,_,_,_,_,_,_,_,_,_,_,_,_,}};

//   vector<vector<int>> GRID = {
//   {_,_,_},
//   {_,_,_},
//   {_,_,_}};

vector<double> START = {0.0,0.0,0.0};
vector<int> GOAL = {(int)GRID.size()-1, (int)GRID[0].size()-1};

void plot(double x, double y){
    std::vector<double> x1, y1;
    x1.push_back(x);
    y1.push_back(y);
    plt::plot(x1,y1,"cx");
}

void plot_grid(double x, double y){
    std::vector<double> x1, y1;
    x1.push_back(x);
    y1.push_back(y);
    plt::plot(x1,y1,"bx");
}

void plot_path(double x, double y){
    std::vector<double> x1, y1;
    x1.push_back(x);
    y1.push_back(y);
    plt::plot(x1,y1,"go");
}

class Element{
    public:
        double x;
        double y;
        double theta;
        double f;
        double cost;
    Element(double xx, double yy, double t1){
        x = xx; y = yy; theta = t1;f = 0.0; cost = 0.0;
    }
    Element(){
        x = 0.0; y = 0.0; theta = 0.0; f = 0.0; cost = 0.0;
    }
};

bool operator>(Element const & a, Element const & b)
{
    return a.f > b.f;
}

// Function to Normalize Theta
double Normalize(double theta){
    double normalized_theta = theta;
    //Normalizing theta ..so that it lies in [0,2pi)
    while(normalized_theta<0){
        cout<<"Theta++ at theta_neighbor = "<<normalized_theta<<endl;
        normalized_theta += 2*PI;
    }
    while(normalized_theta>=2*PI){
        cout<<"Theta-- at theta_neighbor = "<<normalized_theta<<endl;
        normalized_theta -= 2*PI;

    }
    if(normalized_theta<0){
        cout<<"********************************Underflow*****************************************"<<endl;
        // break;
    }
    if(normalized_theta>=2*PI){
        cout<<"*******************************Overflow********************************************"<<endl;
        // break;
    }
    return normalized_theta;
}


vector<Element> add_neighbors(Element current){
    vector<Element> res;
    double x_current = current.x;
    double y_current = current.y;
    double theta_current = current.theta;
    for(double delta = -35; delta<40; delta += delta_step){
        double x = x_current + SPEED * cos(theta_current);
        double y = y_current + SPEED * sin(theta_current);
        double delta_in_radians = (delta*PI)/180.0;
        double omega = (SPEED / LENGTH) * tan(delta_in_radians);
        double theta = theta_current + omega;
        theta = Normalize(theta);
        Element new_ele(x,y,theta);
        res.push_back(new_ele);
    }
    return res;
}

//Function to return theta_id
int theta_id(double theta){
    int id =  floor( (theta*num_theta)/(2*PI) );
    if(id<0 || id>=num_theta){
        cout<<"*******************************Num_Theta Segmentation fault**************************"<<endl;
    }
    return id;
}

int main(){

    //Plot Grid and Start locations
plot(START[0],START[1]);
plot(GOAL[0],GOAL[1]);
// for(int i = 0; i<GRID.size(); i++){
//     for(int j = 0; j<GRID[0].size(); j++){
//         if(GRID[i][j]==1){
//             plot_grid(i,j);
//         }
//     }
// }
double lim_x = GRID[0].size();
  double lim_y = GRID[0].size();
  for(int i = 0; i<GRID.size(); i++){
    for(int j=0; j<GRID[0].size(); j++){
      vector<double> a1 {double(i),double(i)};
      vector<double> b1 {double(j),lim_y};
      vector<double> a2 {double(i),lim_x};
      vector<double> b2 {double(j),double(j)};

      
      plt::plot(a1,b1,"k");
      plt::plot(a2,b2,"k");



      vector<double> x11 = {double(i)};
      vector<double> y11 = {double(j)};
      if(GRID[i][j]==1){
        plt::plot(x11,y11,"rs");
        vector<double> a3 = {double(i), double(i+1)};
        vector<double> b3 = {double(j), double(j+1)};
        vector<double> a4 = {double(i), double(i+1)};
        vector<double> b4 = {double(j+1), double(j)};
        plt::plot(a3,b3,"r-");
        // plt::pause(.5);
        plt::plot(a4,b4,"r-");
        // plt::pause(.5);
      }

    }
  }

    bool reached_goal = false;

    vector< vector < vector <int> > > explored ( num_theta, vector<vector<int>> ( GRID.size(), vector<int> ( GRID[0].size() , 0) ));
    // vector<vector< vector<Element>>> a(num_theta, vector<vector<Element>>   (          (GRID.size(), vector<Element> (GRID[0].size()))            );
    vector<Element> a(GRID[0].size());
    vector<vector<Element>> b (GRID.size(),a);
    vector<vector<vector<Element>>> parent (num_theta,b);



    Element start(START[0],START[1],START[2]);
    // parent[0][0][0] = start;


    priority_queue< Element, vector<Element>, greater<Element> > open_list;
    open_list.push(start);
    Element Parent_of_goal;

    while(!open_list.empty()){
        Element current = open_list.top();
        open_list.pop();
        double x = current.x;
        double y = current.y;
        double theta = Normalize(current.theta);
        plot(x,y);
        // plt::pause(.1);
        cout<<"Debugging here1"<<endl;
        if(int(x)==GOAL[0] && int(y)==GOAL[1]){
            cout<<"Reached Goal";
            reached_goal = true;
            Parent_of_goal = parent[theta_id(theta)][int(x)][int(y)];
            break;
        }
        else{
            vector<Element> neighbors;
            neighbors = add_neighbors(current);
            cout<<"Number of neighbors added = "<<neighbors.size()<<endl;

            //check if neighbors are within Grid && not on obstacle. //check if neighbors are unexplored.
            for(auto i:neighbors){
                if(i.x>=0 && i.x<=GRID.size() && i.y>=0 && i.y<=GRID[0].size() && GRID[int(i.x)][int(i.y)]==0){
                    cout<<"Here1"<<endl;
                    double theta_neighbor = i.theta;
                    cout<<"Here2"<<endl;

                    theta_neighbor = Normalize(theta_neighbor);
                    cout<<"Here3"<<endl;

                    int theta_neighbor_id = theta_id(theta_neighbor);
                    cout<<"ID generated is "<<theta_neighbor_id<<endl;

                    if(explored[theta_neighbor_id][int(i.x)][int(i.y)]==0){
                        //Heiristic - Euclidian distance 

                        cout<<"Debugging here2"<<endl;
                        double goal_x = GOAL[0];
                        double goal_y = GOAL[1];
                        double h = sqrt( pow(i.x - goal_x,2) + pow(i.y - goal_y,2) );
                        double c = i.cost + 1; //Check for change after changing resolution
                        double f = h+c;
                        parent[theta_neighbor_id][int(i.x)][int(i.y)] = current;
                        i.cost = c;
                        i.f = f;
                        open_list.push(i);
                        explored[theta_neighbor_id][int(i.x)][int(i.y)]=1;
                        cout<<"Added Valid Neighbor at x,y = ("<<i.x<<" "<<i.y<<" )"<<endl;

                    }

                }
            }

        }
    }

    if(reached_goal==false){
        cout<<"Path to Goal not found"<<endl;
    }

    //create vector to store all parents.
    vector<Element> path;
    while(1){
        double x_1 = Parent_of_goal.x;
        double y_1 = Parent_of_goal.y;
        double theta_1 = Normalize( Parent_of_goal.theta );
        int id_1 = theta_id(theta_1);
        path.push_back(Parent_of_goal);
        
        Parent_of_goal = parent[id_1][int(x_1)][int(y_1)];
        if(x_1==START[0] && y_1==START[1]){
            break;
        }
    }
    for(int i = path.size(); i>=0; i--){
        double x_1 = path[i].x;
        double y_1 = path[i].y;
        plot_path(x_1,y_1);
        plt::pause(.1);
    }

    cout<<"Code Over"<<endl;
    plt::show();

    



    // While Open List isn't empty

        // Current = ... . Remove Current from Open. 

        // If Current = Goal. Exit. 

        // Obtain neighbors of Current.

        // Check if Neighbors are valid. Check if Neighbors are unexplored. 
            // If yes, Calculatute their Heuristic, Cost, Parent node and then Add them to Open List.

        // Sort the list? (Else, use a priority queue)

        

    // plot(x,y,theta, next_x, next_y, next_theta);
}
