#ifndef OBS_POSITION_H_
#define OBS_POSITION_H_

#include <ros/ros.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PointStamped.h"

enum class Motion_type 
{
    STATIC,
    RECIPROCATION
};

class Obstacle
{
public:

    std::string from;
    Motion_type motion_type;
    std::vector<std::vector<double>> position;
    double stdev;
private:

};

#endif // OBS_POSITION_H