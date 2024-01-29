#ifndef GOAL_POINTER_H
#define GOAL_POINTER_H

#include <ros/ros.h>
//tf2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
//messages
#include<std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>

#define PI 3.14159
#define max_speed_linear 0.5
#define max_speed_angular 1.0

enum class Mode{
    Facing = 0,
    Moving,
    Turnning
};

enum class Velocity_Status{
    Acceleration = 0,
    Max_Speed,
    Deceleration,
    Approaching
};

#endif