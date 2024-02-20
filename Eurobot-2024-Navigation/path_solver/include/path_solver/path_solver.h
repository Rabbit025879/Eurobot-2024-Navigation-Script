#ifndef PATH_SOLVER_H
#define PATH_SOLVER_H
// ROS
#include "ros/ros.h"
// tf2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// messages
#include "std_msgs/Bool.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PolygonStamped.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Odometry.h"
#include "obstacle_detector/Obstacles.h"
// Service
#include <nav_msgs/GetPlan.h>
// std
#include <vector>
#include <algorithm>
#include <string> 

#define robot_size 0.14
#define rival_size 0.15
#define pot_size 0.1
#define nodes_cost_param 1.2
#define divied_path 60 
#define normal_inflation_rival 0.03
#define normal_inflation_pot 0.03
#define normal_inflation_plant 0.03
#define normal_inflation_wall 0.01
#define peace_radius 0.07
#define aggressive_radius 0.02

enum class Step{
    Checking = 1,
    Planning,
    Selecting,
    Publishing,
    Finishing
};

struct Point{
    double x = 0.0;
    double y = 0.0;
};

struct Slope{
    double I = 0.0;
    double II = 0.0;
    double III = 0.0;
    double IIII = 0.0;
};

//Given a point outside of the circle, find the tangent line
class Line_tan{
    public:
        //Set up the line - PC (PC stands for point - circle)
        void line_param_insert_pc(Point point_on_line, Point point_outside, double radius);
        //Set up the line - CC (CC stands for circle - circle)
        void line_param_insert_cc(Point obs1, Point obs2, double radius1, double radius2);
        //This function has no connection to the class, it is just a function to find intersections, it won't change any parameters !! 
        // 2 lines for t & 2 lines for u
        Point line_intersection_22(Line_tan t, Line_tan u, int n);
        //This function has no connection to the class, it is just a function to find intersections, it won't change any parameters !! 
        // 1 line for t & 4 lines for u
        Point line_intersection_14(Line_tan t, Line_tan u, int n, Point comfirm_point);
        // 1 line for t & 2 lines for u
        Point line_intersection_12(Line_tan t, Line_tan u, Point comfirm_point, Point goal);
    private:
        //Input parameters
        Point point_on_line;
        Point point_outside;    
        //Slope calculate
        Slope tan;
        //The parameter of the equation of the two tangent line -> 1-2 for pc, 1-4 for cc
        double a1 = 0.0;
        double b1 = 0.0;
        double c1 = 0.0;
        
        double a2 = 0.0;
        double b2 = 0.0;
        double c2 = 0.0;
        
        double a3 = 0.0;
        double b3 = 0.0;
        double c3 = 0.0;
        
        double a4 = 0.0;
        double b4 = 0.0;
        double c4 = 0.0;
        //The parameter of the equation which aims to find the slope of tangent line
        double m2 = 0.0;
        double m1 = 0.0;
        double m0 = 0.0;
};

#endif