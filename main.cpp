#include <iostream>
#include <vector>
#include "hybrid_breadth_first.h"
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;
#include <unistd.h>

using std::cout;
using std::endl;

// Sets up maze grid
int X = 1;
int _ = 0;

/**
 * TODO: You can change up the grid maze to test different expansions.
 */
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

vector<double> START = {0.0,0.0,0.0};
vector<int> GOAL = {(int)GRID.size()-1, (int)GRID[0].size()-1};

int main() {
  cout << "Finding path through grid:" << endl;
  
  // Creates an Empty Maze and for testing the number of expansions with it
  for(int i = 0; i < GRID.size(); ++i) {
    cout << GRID[i][0];
    for(int j = 1; j < GRID[0].size(); ++j) {
      cout << "," << GRID[i][j];
    }
    cout << endl;
  }

  HBF hbf = HBF();
  double lim_x = GRID[0].size();
  double lim_y = GRID[0].size();
  for(int i = 0; i<GRID.size(); i++){
    for(int j=0; j<GRID[0].size(); j++){
      // plt::plot(j, GRID.size()-1 - i)
      // vector<double> x11 = {double(j)};
      // vector<double> y11 = {double(GRID.size()-1 - i)};
      // vector<double> x_lim = {double(i),double(GRID.size()-1)};
      // vector<double> y_lim = {double(j),double(GRID[0].size()-1)};
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
// {{"color", "red"}, {"marker": "o"}, {"linestyle": "--"}}
// {{"fillstyle", "full"}, {"markersize": "25"}}

  HBF::maze_path get_path = hbf.search(GRID,START,GOAL);

  vector<HBF::maze_s> show_path = hbf.reconstruct_path(get_path.came_from, 
                                                       START, get_path.final);

  cout << "show path from start to finish" << endl;
  vector<double> x_coordinates, y_coordinates;
  for(int i = show_path.size()-1; i >= 0; --i) {
      HBF::maze_s step = show_path[i];
      cout << "##### step " << step.g << " #####" << endl;
      cout << "x " << step.x << endl;
      cout << "y " << step.y << endl;
      cout << "theta " << step.theta << endl;
      // x_coordinates = {double(step.y)};
      // y_coordinates = { double( GRID.size()-1 - step.x) };
      x_coordinates = {double(step.x)};
      y_coordinates = { double(step.y) };
      plt::plot(x_coordinates,y_coordinates,"bo");
      // sleep(0.5);
      plt::pause(.1);

  }

  
  plt::show();
  // plt::pause(100 );
  // plt::plot(x_coordinates,y_coordinates);
  

  return 0;
}