#include<iostream>
#include<vector>
#include <math.h>
#include "matplotlibcpp.h"
#include "hybrid_breadth_first.h"
namespace plt = matplotlibcpp;
using namespace std;

double SPEED = 1.45;
double LENGTH = 0.5;

void plot(double x1, double y1, double theta, double x2, double y2, double theta2){
    double x_prev = x1;
    double y_prev = y1;
    double x_next = x_prev;
    double y_next = y_prev;
    for(int i = 0; i<=100; i++){
        x_next = x_prev + (SPEED * cos(theta)  )*1/100;
        y_next = y_prev + (SPEED * sin(theta) ) *1/100;
        vector<double> vec_x = {x_next,x_prev};
        vector<double> vec_y = {y_next, y_prev};
        plt::plot(vec_x,vec_y);
        x_prev = x_next;
        y_prev = y_next;
        
        // delta2 = delta * i/100
        // double omega = SPEED / LENGTH * tan(delta2);
        // theta = theta + omega;

        theta += (theta2 - theta)*1/100;



        if(theta > 2*3.1415) {  
            theta = theta - 2*3.1415;
        }
    }
    cout<<endl<<"Next x is :"<<x_next;
    cout<<endl<<"Next y is :"<<y_next<<endl;
    plt::ylim(-1,11);
    plt::xlim(-1,11);
    plt::show();
}

int main(){
    cout<<"Testing124124";
    // vector<int> a{1,2,3};
    // vector<int> b{1,2,4};
    // plt::plot({1,3,2,4});
    // plt::plot(a,b);
    // plt::show();
    
    double theta = 0;
    vector<int> start = {0,0};
    double x = start[0];
    double y = start[1];
    
    double next_x = x + SPEED * cos(theta);
    double next_y = y + SPEED * sin(theta);
    
    double delta_in_degrees = -35;
    double delta = delta_in_degrees * 3.1415 / 180;
    double omega = SPEED / LENGTH * tan(delta);
    double next_theta = theta + omega;

    cout<<endl<<"Next x is :"<<next_x;
    cout<<endl<<"Next y is :"<<next_y<<endl;

    plot(x,y,theta, next_x, next_y, next_theta);
}