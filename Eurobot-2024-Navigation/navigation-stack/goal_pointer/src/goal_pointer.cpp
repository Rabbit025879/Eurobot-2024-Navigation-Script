#include "goal_pointer.h"

bool is_sim = 1;

//DEBUG
int DEBUG = 0;
double buffer = 0.0;
bool print_once = 0;
//Target goal
double goal_x = 0.0;
double goal_y = 0.0;
//Final goal
double final_goal_x = 0.0;
double final_goal_y = 0.0;
//Robot pose
double pose_sim_x = 0.0;
double pose_sim_y = 0.0;
double pose_real_x = 0.0;
double pose_real_y = 0.0;
double pose_x = 0.0;
double pose_y = 0.0;
//Convert
nav_msgs::Odometry OS;
geometry_msgs::PoseWithCovarianceStamped OR;
geometry_msgs::PoseStamped G;

//Get node goal
void getgoal(const geometry_msgs::PoseStamped& goal_pose){
    goal_x = goal_pose.pose.position.x;
    goal_y = goal_pose.pose.position.y;
    G = goal_pose;
}
//Get final goal
void getfinalgoal(const geometry_msgs::PoseStamped& final_goal_pose){
    final_goal_x = final_goal_pose.pose.position.x;
    final_goal_y = final_goal_pose.pose.position.y;
}
//Get robot pose for simulation
void getodom_sim(const nav_msgs::Odometry& robot_sim_pose){
    pose_sim_x = robot_sim_pose.pose.pose.position.x;
    pose_sim_y = robot_sim_pose.pose.pose.position.y;
    OS = robot_sim_pose;
}
//Get robot pose for real
void getodom_real(const geometry_msgs::PoseWithCovarianceStamped& robot_real_pose){
    pose_real_x = robot_real_pose.pose.pose.position.x;
    pose_real_y = robot_real_pose.pose.pose.position.y;
    OR = robot_real_pose;
}
//velocity controller
double vel_controller(double diff){
    double vel_output = 0.5;
    double a = 0;
    double initial_diff = 0;
    bool once = 0;
    int kp = 0.5;

    if(once == 0){
        initial_diff = diff;
        once = 1;
    }

    // if(diff <= 0.2*initial_diff)    vel_output = 0.2;
    // else if(diff >= 0.2*initial_diff)   vel_output = 0.2;
    // else    vel_output = 0.6;
    // vel_output = diff*kp;
    // ROS_INFO("diff -> %lf", diff);
    // ROS_INFO("diff -> %lf", vel_output);
    // if(vel_output > 0.5)    vel_output = 0.5;

    return vel_output;
}
// double Extract_Velocity(VELOCITY vel_type, RobotState goal_pos, double acceleration) {
//     double output_vel = 0;
//     if (working_mode_ == MODE::TRACKING) {
//         if (vel_type == VELOCITY::LINEAR) {
//             RobotState origin(0, 0, 0);
//             double last_vel = velocity_state_.distanceTo(origin);
//             // RobotState last_vel = velocity_state_;
//             // acceleration
//             double d_vel = acceleration * dt_;
//             ROS_INFO_STREAM("[path_exec]" << __LINE__ << "dvel: " << d_vel);

//             double xy_err = cur_pose_.distanceTo(goal_pose_);
//             linear_integration_ += xy_err;

//             ROS_INFO_STREAM("path exec" << __LINE__ << "out vel: " << output_vel);
//             output_vel = (xy_err + 0.03) * linear_kp_ ;

//             if (output_vel > std::min(linear_max_vel_, last_vel + d_vel)) {
//                 output_vel = std::min(linear_max_vel_, last_vel + d_vel);
//             }

//         }

//         else if (vel_type == VELOCITY::ANGULAR) {
//             double d_vel = acceleration * dt_;
//             output_vel = velocity_state_.theta_ + d_vel;
//             double theta_err = (Angle_Limit_Checking(goal_pos.theta_ - cur_pose_.theta_));
//             double last_vel = velocity_state_.theta_;
//             if (fabs(theta_err) < angular_brake_distance_) {
//                 output_vel = theta_err * angular_kp_;
//             }

//             // Saturation
//             if (output_vel > angular_max_vel_)
//                 output_vel = angular_max_vel_;
//             if (output_vel < -angular_max_vel_)
//                 output_vel = -angular_max_vel_;
//         }
//     }
//     return output_vel;
// }
//Facing
int facing_dirction(double diff_spin, double diff_face){
    int dir_facing = 0;

    //Determine direction
    // if((diff_face*diff_spin) < 0){
    if(diff_spin >= 0 && diff_spin <= PI/2)  dir_facing = 1;
    else if(diff_spin > PI/2)   dir_facing = -1;
    else if(diff_spin < 0 && diff_spin >= -PI/2)  dir_facing = -1;
    else if(diff_spin < -PI/2)  dir_facing = 1;
    // }
    // else{

    // }

    // if(diff_face >= 0){
    //     if(diff_face <= PI/2)   dir_facing = 1;
    //     else if(diff_face > PI/2)   dir_facing = -1;
    // }
    // else if(diff_face < 0){
    //     if(diff_face >= -PI/2)  dir_facing = -1;
    //     else if(diff_face < -PI/2)  dir_facing = 1;
    // }
    return dir_facing;
}
//Moving
int moving_direction(double diff_face, double dis_prev, double dis_now){
    int dir_moving = 1;
    int data = 0;
    //Closer or farer
    if(dis_now <= dis_prev) data = 1;
    else    data = -1;
    //Determine direction
    if(fabs(diff_face) < 0.05) dir_moving = 1*data;
    else if(fabs(diff_face) > PI-0.05) dir_moving = -1*data;
    return dir_moving;
}
//Spinning
int spinning_direction(double diff_spin){
    int dir_spinning = 0;
    //Determine direction
    if(diff_spin >= 0)  dir_spinning = 1;
    else if(diff_spin < 0)  dir_spinning = -1;
    return dir_spinning;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "goal_pointer");
    // ros::Rate freq(20);
    ros::NodeHandle nh;
    ros::NodeHandle simple_nh("move_base_simple");
    ros::Publisher linear_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    ros::Publisher goal_reached_pub = nh.advertise<std_msgs::Bool>("goal_reached", 1);
    ros::Publisher goal_pointer_pub = nh.advertise<nav_msgs::Odometry>("goal_pointer", 1);  //For RVIZ visualization
    ros::Subscriber goal_sub = nh.subscribe("point", 1, getgoal);
    ros::Subscriber final_goal_sub = simple_nh.subscribe("goal", 1, getfinalgoal);
    ros::Subscriber pose_sim_sub = nh.subscribe("odom",1,getodom_sim);
    ros::Subscriber pose_real_sub = nh.subscribe("ekf_pose",1,getodom_real);

    //Finite state machine
    Mode point_to_point_process = Mode::Facing;

    //New goal?
    bool new_goal = 0;
    double goal_x_ed = 0.0;
    double goal_y_ed = 0.0;

    //States
    double dis = 0.0;
    double diff_x = 0.0;
    double diff_y = 0.0;
    double diff_spin = 0.0;
    double diff_spin_initial = 0.0;
    double diff_face = 0.0;
    double diff_face_initial = 0.0;
    double dis_prev = 0.0;
    double dis_now = 0.0;

    double goal_face = 0.0;
    double robot_face = 0.0;
    double path_face = 0.0;
    
    //cmd_vel
    double base_vel_x = 0.0;
    double base_vel_spin = 0.0;

    //delay
    int delay = 0;
    int wait = 0;

    //Massages      
    std_msgs::Bool  goal_reached;
    geometry_msgs::Twist cmd_vel_override; //cmd_vel
    geometry_msgs::PoseStamped goal_pose;  //goal
    geometry_msgs::PoseStamped final_goal_pose;  //final goal
    nav_msgs::Odometry robot_pose;         //odom
    nav_msgs::Odometry goal_pointer;       //goal pointer

    goal_reached.data = false;

    while(ros::ok()){
        //Callback
        ros::spinOnce();
        if(is_sim == true){
            pose_x = pose_sim_x;
            pose_y = pose_sim_y;
        }
        else{
            pose_x = pose_real_x;
            pose_y = pose_real_y;
        }
        // ROS_INFO("pose.goal_pointer -> (%lf, %lf)", pose_x, pose_y);
        //--------------------------------------------------------
        //Calculations
        //--------------------------------------------------------

        //Convert robot pose (quaterion -> RPY)
        if(is_sim == 1){
            tf2::Quaternion oS;
            tf2::fromMsg(OS.pose.pose.orientation, oS);
            tf2::Matrix3x3 ot(oS);
            double _OS, yaw_OS;
            ot.getRPY(_OS, _OS, yaw_OS);
            if(yaw_OS >= 0)  robot_face = yaw_OS;
            else robot_face = 2*PI - fabs(yaw_OS);
        }
        else{
            tf2::Quaternion oR;
            tf2::fromMsg(OR.pose.pose.orientation, oR);
            tf2::Matrix3x3 ot(oR);
            double _OR, yaw_OR;
            ot.getRPY(_OR, _OR, yaw_OR);
            if(yaw_OR >= 0)  robot_face = yaw_OR;
            else robot_face = 2*PI - fabs(yaw_OR);
        }

        //Convert robot pose (quaterion -> RPY)
        tf2::Quaternion g;
        tf2::fromMsg(G.pose.orientation, g);
        tf2::Matrix3x3 gt(g);
        double _G, yaw_G;
        gt.getRPY(_G, _G, yaw_G);
        if(yaw_G >= 0)  goal_face = yaw_G;
        else goal_face = 2*PI - fabs(yaw_G);
        //Convert path face (RPY -> quaterion)
        tf2::Quaternion r;
        r.setRPY(0,0,path_face);
        //Calculate the difference between target goal & robot pose
        diff_x = goal_x - pose_x;   //difference between x
        diff_y = goal_y - pose_y;   //difference between y
        //Calculate angular difference between goal & robot(-PI to PI)
        if(goal_face >= robot_face){
            if(goal_face-robot_face <= PI)    diff_spin = goal_face - robot_face; 
            else if(goal_face-robot_face > PI) diff_spin = -(2*PI - (goal_face - robot_face));
        }
        else if(goal_face < robot_face){
            if(robot_face-goal_face <= PI)  diff_spin = -(robot_face - goal_face);
            else if(robot_face-goal_face > PI) diff_spin = 2*PI - (robot_face - goal_face);
        }
        dis = sqrt(pow(diff_x,2)+pow(diff_y,2));    //Calculate the distance between target goal & robot pose
        //Calculate angular difference between path & robot(-PI to PI)
        if(path_face >= robot_face){
            if(path_face-robot_face <= PI)    diff_face = path_face - robot_face; 
            else if(path_face-robot_face > PI) diff_face = -(2*PI - (path_face - robot_face));
        }
        else if(path_face < robot_face){
            if(robot_face-path_face <= PI)  diff_face = -(robot_face - path_face);
            else if(robot_face-path_face > PI) diff_face = 2*PI - (robot_face - path_face);
        }

        //--------------------------------------------------------
        //Point to point
        //-------------------------------------------------------- 
        //If received a new goal
        if(goal_x != goal_x_ed || goal_y != goal_y_ed){
            new_goal = 1;
            point_to_point_process = Mode::Facing;
            diff_spin_initial = diff_spin;
            diff_face_initial = diff_face;
            wait = 0;
            //Determine goal pointer
            if(diff_y > 0)  path_face = fabs(acos(diff_x/dis));    //Calculate path twist
            else if(diff_y < 0)   path_face = 2*PI - fabs(acos(diff_x/dis));
            else if(diff_y == 0 && diff_x > 0) path_face = 0;
            else if(diff_y == 0 && diff_x < 0) path_face = PI;
            //If goal suddenly changed
            // facing_done = 0;
            // moving_done = 0;
            // spinning_done = 0;
            // base_vel_x = 0;
            // base_vel_spin = 0;
        }

        if(new_goal){
            wait++;
            if(wait >= 22){
                //First, we face toward our goal
                if(point_to_point_process == Mode::Facing){
                    DEBUG = 1;
                    //Execute
                    if(fabs(diff_face) < 0.05 || fabs(diff_face) > PI-0.05){
                        delay++;
                        if(delay <= 22){
                            base_vel_spin = 0;
                            point_to_point_process = Mode::Moving;
                            delay++;
                        }
                    }
                    else{
                        base_vel_spin = /*vel_controller(fabs(diff_face))*/0.7*facing_dirction(diff_spin_initial, diff_face_initial);
                    }
                }

                //Then, we move along x-axis
                if(point_to_point_process == Mode::Moving){   
                    DEBUG = 2;
                    dis_now = sqrt(pow((goal_x - pose_x),2)+pow((goal_y - pose_y),2));
                    //Tolerance
                    if(fabs(diff_x) < 0.05 && fabs(diff_y) < 0.05){
                        base_vel_x = 0;
                        goal_reached.data = true;
                        point_to_point_process = Mode::Turnning;
                        // if(goal_x == final_goal_x && goal_y == final_goal_y){
                        //     delay++;
                        //     if(delay >= 22){
                        //         point_to_point_process = Mode::Turnning;
                        //         delay = 0;
                        //     }
                        // }
                        // else{
                        //     delay++;
                        //     if(delay >= 22){
                        //         goal_reached.data = false;                              
                        //         delay = 0;
                        //         point_to_point_process = Mode::Turnning;
                        //     }
                        // }
                    }
                    else if(fabs(diff_x) >= 0.05 || fabs(diff_y) >= 0.05)   base_vel_x = vel_controller(dis_now)*moving_direction(diff_face, dis_prev, dis_now);  
                    dis_prev = dis_now;
                }

                // Finally, we rotate to the target pose
                if(point_to_point_process == Mode::Turnning){
                    DEBUG = 3;
                    if(fabs(diff_spin) < 0.05){
                        delay++;
                        if(delay >= 22){
                            base_vel_spin = 0;
                            new_goal = 0;
                            goal_reached.data = false;
                            delay = 0;
                        }
                    }
                    else{
                        base_vel_spin = /*vel_controller(fabs(diff_spin))*/0.7*spinning_direction(diff_spin);
                    }            
                }   
            }
        }
        
        //--------------------------------------------------------

        //Publish velocity
        cmd_vel_override.linear.x = base_vel_x; 
        cmd_vel_override.angular.z = base_vel_spin;
        linear_vel_pub.publish(cmd_vel_override);
        //Publish goal reached or not
        goal_reached_pub.publish(goal_reached);
        //Publish goal pointer
        goal_pointer.pose.pose.position.x = robot_pose.pose.pose.position.x;
        goal_pointer.pose.pose.position.y = robot_pose.pose.pose.position.y;
        goal_pointer.pose.pose.position.z = robot_pose.pose.pose.position.z;

        goal_pointer.pose.pose.orientation.z = r.getZ();
        goal_pointer.pose.pose.orientation.w = r.getW();

        goal_pointer_pub.publish(goal_pointer);

        // ROS_ERROR("goal_pointer -> Data = %lf", double(new_goal));

        //DEBUG
        // if(double(point_to_point_process) != buffer) print_once = 0;  
        // buffer = double(point_to_point_process);        
        // if(print_once == 0){
        //     ROS_ERROR("goal_pointer -> Data = %lf", buffer);
        //     print_once = 1;
        // }
        // ROS_INFO("path_face = %lf", path_face);
        // ROS_WARN("robot_face = %lf", robot_face);
        // ROS_WARN("O.pose.pose.orientation = %lf", O.pose.pose.orientation.z);
        // ROS_FATAL("new_goal = %d", new_goal);
        // ROS_FATAL("DEBUG = %d", DEBUG);
        // ROS_WARN("goal_x = %lf", goal_x);
        // ROS_WARN("goal_x_ed = %lf", goal_x_ed);
        // ROS_WARN("diff_spin = %lf", diff_spin);
        // ROS_WARN("diff_face = %lf", diff_face);
        // ROS_FATAL("spinning_done = %d", spinning_done);
        // ROS_WARN("diff_x = %lf", diff_x);
   
        goal_x_ed = goal_x;
        goal_y_ed = goal_y;

        // freq.sleep();
    }
}