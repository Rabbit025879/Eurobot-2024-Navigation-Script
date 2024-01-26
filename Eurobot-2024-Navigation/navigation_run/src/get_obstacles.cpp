#include <ros/ros.h>
#include <costmap_converter/ObstacleMsg.h>
#include <costmap_converter/ObstacleArrayMsg.h>
#include <geometry_msgs/Point32.h>
#include <nav_msgs/Odometry.h>
#include <string>


std::string oppo_name_, my_name_;
double x, y, x_vel, y_vel;

void oppo_callback(const nav_msgs::Odometry::ConstPtr& data) {
    x = data->pose.pose.position.x;
    y = data->pose.pose.position.y;
    x_vel = data->twist.twist.linear.x;
    y_vel = data->twist.twist.linear.y;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "get_obstacles");
    
    ros::NodeHandle nh, private_nh("~");

    private_nh.param("oppo_name", oppo_name_, std::string(""));
    private_nh.param("my_name", my_name_, std::string(""));

    std::string odom_topic_ = "odom";
    if (oppo_name_ != "") odom_topic_ = '/' + oppo_name_ + '/' + odom_topic_;
    // nav_msgs::Odometry
    ros::Subscriber sub_oppo = nh.subscribe(odom_topic_, 1, oppo_callback);

    // costmap_converter::ObstacleMsg
    std::string my_topic_ = "move_base/TebLocalPlannerROS/obstacles";
    if (my_name_ != "") my_topic_ = '/' + my_name_ + '/' + my_topic_;  
    ros::Publisher pub_obst = nh.advertise<costmap_converter::ObstacleArrayMsg>(my_topic_, 1);

    ROS_INFO("my topic %s %s", my_topic_.c_str(), odom_topic_.c_str());
    ros::Rate r(20.0);
    while (ros::ok()) {
        ros::spinOnce();
        costmap_converter::ObstacleArrayMsg obsArray;
        costmap_converter::ObstacleMsg obsMsg;

        obsMsg.header.stamp = ros::Time::now();
        obsMsg.header.frame_id = my_name_ + "/" + "map";

        obsMsg.radius = 0.144;
        obsMsg.id = 0;
        geometry_msgs::Point32 pt;
        pt.x = x, pt.y = y;
        obsMsg.polygon.points.push_back(pt);
        obsMsg.velocities.twist.linear.x = x_vel;
        obsMsg.velocities.twist.linear.y = y_vel;

        obsArray.header.frame_id = my_name_ + "/" + "map";
        obsArray.header.stamp = ros::Time::now();
        obsArray.obstacles.push_back(obsMsg); 

        pub_obst.publish(obsArray);
        
        r.sleep();
    }
    return 0;
}