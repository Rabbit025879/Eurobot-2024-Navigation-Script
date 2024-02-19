#include "pathExecutor.h"



RobotState::RobotState(double x, double y, double theta) {
    x_ = x;
    y_ = y;
    theta_ = theta;
}

double RobotState::distanceTo(RobotState pos) {
    return sqrt(pow(x_ - pos.x_, 2) + pow(y_ - pos.y_, 2));
}

ObstState::ObstState(double x, double y, double vel_x, double vel_y) {
    x_ = x;
    y_ = y;
    vel_x_ = vel_x;
    vel_y_ = vel_y;
}

PathExecutor::PathExecutor(ros::NodeHandle& nh, ros::NodeHandle& nh_local) {
    nh_ = nh;
    nh_local_ = nh_local;
    std_srvs::Empty empt;
    params_srv_ = nh_local_.advertiseService("params", &PathExecutor::initializeParams, this);
    initializeParams(empt.request, empt.response);
    initialize();
}

PathExecutor::~PathExecutor() {
    // nh_local_.deleteParam("<param name>");
    nh_local_.deleteParam("frame");
    nh_local_.deleteParam("control_frequency");
    nh_local_.deleteParam("xy_tolerance");
    nh_local_.deleteParam("theta_tolerance");
    nh_local_.deleteParam("lookahead_d");
    nh_local_.deleteParam("linear_max_vel");
    nh_local_.deleteParam("linear_acceleration");
    nh_local_.deleteParam("angular_max_vel");
    nh_local_.deleteParam("angular_acceleration");
    nh_local_.deleteParam("can_reverse_drive");
    nh_local_.deleteParam("replan_frequency_divider");

    // nh_local.deleteParam("weight_obstacle");
    nh_local_.deleteParam("min_obstacle_dist");
    nh_local_.deleteParam("replan_threshold");
    nh_local_.deleteParam("pose_type");

    // spin and dock param
    nh_local_.deleteParam("spin_kp");
    nh_local_.deleteParam("dock_kp");
    nh_local_.deleteParam("spin_and_dock_start_dist");

    // diff drive
    nh_local_.deleteParam("straight_margin_rad");
    nh_local_.deleteParam("deviate_margin_rad");

    vel_pub_.shutdown();
    pose_sub_.shutdown();
    diff_radius_pub_.shutdown();
    goal_reached_pub_.shutdown();
    emergency_stop_sub_.shutdown();
    local_goal_pub_.shutdown();
}

void PathExecutor::initialize() {
    // is_local_goal_final_reached_ = false;
    // is_global_path_switched_ = false;
    // is_goal_blocked_ = false;
    timer_ = nh_.createTimer(ros::Duration(1.0 / control_frequency_), &PathExecutor::Timer_Callback, this, false);

    working_mode_ = MODE::IDLE;
    working_mode_pre_ = MODE::IDLE;

    t_bef_ = ros::Time::now();
    t_now_ = ros::Time::now();
    
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    path_pub_ = nh_.advertise<nav_msgs::Path>("followed_path", 1);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("obstacle_marker", 1);
    local_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("local_goal", 10);
    if (pose_type_ == 0) {
        pose_sub_ = nh_.subscribe("odom", 50, &PathExecutor::Pose_Callback, this);
    }
    else if (pose_type_ == 1) {
        pose_sub_ = nh_.subscribe("ekf_pose_in_odom", 50, &PathExecutor::Pose_Callback, this);
    }
    obst_sub_ = nh_.subscribe("obstacles", 50, &PathExecutor::Obst_Callback, this);

    emergency_stop_sub_=nh_.subscribe("emergency_stop",50,&PathExecutor::Emergency_Stop_Callback,this);
    diff_radius_pub_ = nh_.advertise<std_msgs::Float64>("diff_radius", 1);
    // goal_reached_pub_ = nh_.advertise<std_msgs::Char>("path_exec_status", 1);
    // goal_sub_ = nh_.subscribe("path_exec_goal", 50, &PathExecutor::Goal_Callback, this);
    replan_cnt_ = 0;
    cur_path_idx_ = 0;
    has_start_dacc_x_ = 0;
    has_start_dacc_y_ = 0;
    replan_idx_ = 0;
}

bool PathExecutor::initializeParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
    // load parameter
    nh_local_.param<std::string>("frame", frame_, "map");
    nh_local_.param<std::string>("robot_type", robot_type_, "holonomic");
    nh_local_.param<double>("control_frequency", control_frequency_, 20);
    nh_local_.param<double>("xy_tolerance", xy_tolerance_, 0.01);
    nh_local_.param<double>("theta_tolerance", theta_tolerance_, 0.03);
    nh_local_.param<double>("lookahead_d", lookahead_d_, 0.3);
    nh_local_.param<double>("can_reverse_drive", can_reverse_drive_, 1);
    nh_local_.param<int>("replan_threshold", replan_threshold_, 3);
    nh_local_.param<int>("pose_type", pose_type_, 0);
    nh_local_.param<int>("replan_frequency_divider", replan_frequency_divider_, 5);

    // linear
    nh_local_.param<double>("linear_max_vel", linear_max_vel_, 0.9);
    nh_local_.param<double>("linear_acceleration", linear_acceleration_, 0.5);
    nh_local_.param<double>("linear_kp", linear_kp_, 2.5);
    nh_local_.param<double>("linear_brake_distance", linear_brake_distance_, 0.3);
    nh_local_.param<double>("linear_margin", linear_margin_, 0.01);
    // nh_local_.param<double>("linear_max_vel", linear_max_vel_, 0.5);

    // angular
    nh_local_.param<double>("angular_max_vel", angular_max_vel_, 0.5);
    nh_local_.param<double>("angular_acceleration", angular_acceleration_, 0.2);
    nh_local_.param<double>("angular_kp", angular_kp_, 2.8);
    nh_local_.param<double>("angular_brake_distance", angular_brake_distance_, 0.5);
    nh_local_.param<double>("angular_margin", angular_margin_, 0.01);

    // spin and dock param
    nh_local_.param<double>("spin_kp", spin_kp_, 5.5);
    nh_local_.param<double>("dock_kp", dock_kp_, 1.2);
    nh_local_.param<double>("spin_and_dock_start_dist", spin_and_dock_start_dist_, 0.03);

    // diff drive
    nh_local_.param<double>("straight_margin_rad", straight_margin_rad_, (double)(3.0 * M_PI / 180.0));
    nh_local_.param<double>("deviate_margin_rad", deviate_margin_rad_, (double)(10.0 * M_PI / 180.0));

    // teb, g2o param
    // nh_local_.param<double>("weight_obstacle", weight_obstacle_, 50);
    nh_local_.param<double>("min_obst_dist", min_obst_dist_, 0.5);
    
    // nh_local_.param<bool>("active", p_active_, true);

    // if (p_active_ != prev_active) {
    //     if (p_active_) {
    //         if (odom_callback_type_ == ODOM_CALLBACK_TYPE::nav_msgs_Odometry) {
    //             pose_sub_ = nh_.subscribe(odom_topic_name_, 50, &PathTracker::Pose_type0_Callback, this);
    //         } else {
    //             pose_sub_ = nh_.subscribe(odom_topic_name_, 50, &PathTracker::Pose_type1_Callback, this);
    //         }
    //         goal_sub_ = nh_.subscribe("nav_goal", 50, &PathTracker::Goal_Callback, this);
    //         vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    //         local_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("local_goal", 10);
    //         pose_array_pub_ = nh_.advertise<geometry_msgs::PoseArray>("orientation", 10);
    //         goal_reached_pub_ = nh_.advertise<std_msgs::Char>("finishornot", 1);
    //     } else {
    //         pose_sub_.shutdown();
    //         goal_sub_.shutdown();
    //         vel_pub_.shutdown();
    //         local_goal_pub_.shutdown();
    //         pose_array_pub_.shutdown();
    //         goal_reached_pub_.shutdown();
    //     }
    // }
    goal_reached_pub_ = nh_.advertise<std_msgs::Char>("finishornot", 1);
    goal_sub_ = nh_.subscribe("move_base_simple/goal", 50, &PathExecutor::Goal_Callback, this);
    ROS_INFO_STREAM("[Path Executor]: set param ok");
    return true;
}

void PathExecutor::Goal_Callback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg) {
    has_start_dacc_x_ = 0;
    has_start_dacc_y_ = 0;
    t_bef_ = ros::Time::now();

    goal_pose_.x_ = pose_msg->pose.position.x;
    goal_pose_.y_ = pose_msg->pose.position.y;

    tf2::Quaternion q;
    tf2::fromMsg(pose_msg->pose.orientation, q);
    tf2::Matrix3x3 qt(q);
    double _, yaw;
    qt.getRPY(_, _, yaw);

    goal_pose_.theta_ = yaw;
    if (goal_pose_.x_ == -1 && goal_pose_.y_ == -1) {
        ROS_INFO("[Path Executor]: Mission Abort!");
        Switch_Mode(MODE::IDLE);
        return ;
    }
    ROS_INFO("Goal received ! (%f, %f, %f)", goal_pose_.x_, goal_pose_.y_, goal_pose_.theta_);

    replan_cnt_ = 0;

    // global_path_past_ = global_path_;
    if (!Get_Global_Path(cur_pose_, goal_pose_)) {
        ROS_WARN_STREAM("[Path Executor]" << __LINE__ << ": Get global path failed");
        is_goal_blocked_ = true;

        // publish /finishornot
        goal_reached_.data = 2;
        goal_reached_pub_.publish(goal_reached_);

        return;
    }
    is_goal_blocked_ = false;

    // publish /finishornot
    goal_reached_.data = 0;
    goal_reached_pub_.publish(goal_reached_);

    // linear_brake_distance_ = linear_brake_distance_ratio_ * cur_pose_.distanceTo(goal_pose_);
    // if (linear_brake_distance_ < linear_min_brake_distance_)
    //     linear_brake_distance_ = linear_min_brake_distance_;

    // is_local_goal_final_reached_ = false;
    // is_global_path_switched_ = false;
    Switch_Mode(MODE::START_SPIN);
    run_time_ = ros::Time::now();
        // new_goal = true;
    linear_integration_ = 0;
}


bool PathExecutor::Get_Global_Path(RobotState cur_pos, RobotState goal_pos) {
    cur_path_idx_ = -1;

    geometry_msgs::PoseStamped cur;
    cur.header.frame_id = frame_;
    cur.pose.position.x = cur_pos.x_;
    cur.pose.position.y = cur_pos.y_;
    cur.pose.position.z = 0;

    tf2::Quaternion q;
    q.setRPY(0, 0, cur_pos.theta_);
    cur.pose.orientation.x = q.x();
    cur.pose.orientation.y = q.y();
    cur.pose.orientation.z = q.z();
    cur.pose.orientation.w = q.w();

    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = frame_;
    goal.pose.position.x = goal_pos.x_;
    goal.pose.position.y = goal_pos.y_;
    goal.pose.position.z = 0;

    // tf2::Quaternion q;
    q.setRPY(0, 0, goal_pos.theta_);
    goal.pose.orientation.x = q.x();
    goal.pose.orientation.y = q.y();
    goal.pose.orientation.z = q.z();
    goal.pose.orientation.w = q.w();

    // ros::ServiceClient client = nh_.serviceClient<nav_msgs::GetPlan>("move_base/PathPlanner/make_plan");
    ros::ServiceClient client = nh_.serviceClient<nav_msgs::GetPlan>("/make_plan");
    nav_msgs::GetPlan srv;
    srv.request.start = cur;
    srv.request.goal = goal;

    std::vector<geometry_msgs::PoseStamped> path_msg;

    // ROS_WARN("[path executor] x, y, z: %f, %f, %f", cur.pose.position.x, cur.pose.position.y, cur.pose.position.z);
    // ROS_WARN("[path executor] x, y, z: %f, %f, %f", goal.pose.position.x, goal.pose.position.y, goal.pose.position.z);
    // ROS_WARN("[path executor] frame_: %s", frame_.c_str());
    if (client.call(srv)) {
        path_pub_.publish(srv.response.plan);
        // new_goal = false;
        // ROS_INFO_STREAM(__FILE__ << " " << __LINE__ << " got " << srv.response.plan.poses.size() << " of poses");
        if (srv.response.plan.poses.empty()) {
            ROS_WARN_STREAM("[PathExecutor] : Got empty plan");
            Switch_Mode(MODE::IDLE);
            return false;
        }

        nav_msgs::Path path_msg;
        path_msg.poses = srv.response.plan.poses;

        global_path_.clear();

        for (const auto& point : path_msg.poses) {
            RobotState pose;
            pose.x_ = point.pose.position.x;
            pose.y_ = point.pose.position.y;
            tf2::Quaternion q;
            tf2::fromMsg(point.pose.orientation, q);
            tf2::Matrix3x3 qt(q);
            double _, yaw;
            qt.getRPY(_, _, yaw);
            pose.theta_ = yaw;
            global_path_.push_back(pose);
        }
        if (global_path_.empty()) return false;
        global_path_ = Orientation_Filter(global_path_);
        return true;
    } else {
        ROS_ERROR_STREAM("[PathTracker] : Failed to call service make_plan");
        return false;
    }
}

void PathExecutor::Timer_Callback(const ros::TimerEvent& e) {
    // ROS_INFO_STREAM("[Path Executor]: in timercallback");
    if (working_mode_ == MODE::TRACKING) {
        if (is_XY_Reached(cur_pose_, goal_pose_) && is_Theta_Reached(cur_pose_, goal_pose_) && !is_goal_blocked_) {
            ROS_INFO_STREAM("[Path Executor]: GOAL REACHED !" << fabs(goal_pose_.y_ - cur_pose_.y_));
            ROS_INFO_STREAM("[pathExecutor]" << " RUNTIME " << (ros::Time::now() - run_time_).toSec() << "s");
            Switch_Mode(MODE::IDLE);
            velocity_state_.x_ = 0;
            velocity_state_.y_ = 0;
            velocity_state_.theta_ = 0;
            Velocity_Publish();

            is_goal_blocked_ = false;
            
            // publish /finishornot
            goal_reached_.data = 1;
            goal_reached_pub_.publish(goal_reached_);
        }
        else if(is_goal_blocked_) {
            ROS_INFO("[PathTracker]: Goal is blocked ... Local goal reached !");
            Switch_Mode(MODE::IDLE);
            velocity_state_.x_ = 0;
            velocity_state_.y_ = 0;
            velocity_state_.theta_ = 0;
            Velocity_Publish();

            is_goal_blocked_ = false;

            // publish /finishornot
            goal_reached_.data = 2;
            goal_reached_pub_.publish(goal_reached_);
            return ;
        }
        else if (is_XY_Reached(cur_pose_, goal_pose_) && !is_Theta_Reached(cur_pose_, goal_pose_)) {
            Switch_Mode(MODE::END_SPIN);
        }
        else {
            // cur_path_idx_ = Get_Next_Local_Goal();
            RobotState local_goal = Rolling_Window(cur_pose_, global_path_, lookahead_d_); 
            // ROS_INFO_STREAM("[Path Executor]: in timercallback local_goal: " << local_goal.x_ << " " << local_goal.y_ << " " << local_goal.theta_)
            if (robot_type_ == "holonomic" ) {
                Omni_Controller(local_goal);
            }
            else if (robot_type_ == "differential") {
                Diff_Controller(local_goal);
            }
            else {
                ROS_ERROR("[Path Executor]: WRONG BASE TYPE!");
                return;
            }      
        }
        if (!Get_Global_Path(cur_pose_, goal_pose_)) {
            ROS_WARN_STREAM("[Path Executor]" << __LINE__ << ": Get global path failed");
            is_goal_blocked_ = true;

            // publish /finishornot
            goal_reached_.data = 2;
            goal_reached_pub_.publish(goal_reached_);
            Switch_Mode(MODE::IDLE);
            return;

        }
    }
    else if (working_mode_ == MODE::IDLE) {
        velocity_state_.x_ = 0;
        velocity_state_.y_ = 0;
        velocity_state_.theta_ = 0;
        Velocity_Publish();
    }
    else if (working_mode_ == MODE::START_SPIN) {
        RobotState local_goal = Rolling_Window(cur_pose_, global_path_, lookahead_d_);
        SpinAndDock(local_goal, 0);
        if (!Get_Global_Path(cur_pose_, goal_pose_)) {
            ROS_WARN_STREAM("[Path Executor]" << __LINE__ << ": Get global path failed");
            is_goal_blocked_ = true;

            // publish /finishornot
            goal_reached_.data = 2;
            goal_reached_pub_.publish(goal_reached_);
            Switch_Mode(MODE::IDLE);
            return;

        }
    }
    else if (working_mode_ == MODE::END_SPIN) {
        if (is_Theta_Reached(cur_pose_, goal_pose_)) {
            ROS_INFO_STREAM("[Path Executor]: GOAL REACHED !" << fabs(goal_pose_.y_ - cur_pose_.y_));
            ROS_INFO_STREAM("[pathExecutor]" << " RUNTIME " << (ros::Time::now() - run_time_).toSec() << "s");
            Switch_Mode(MODE::IDLE);
            velocity_state_.x_ = 0;
            velocity_state_.y_ = 0;
            velocity_state_.theta_ = 0;
            Velocity_Publish();

            is_goal_blocked_ = false;
            
            // publish /finishornot
            goal_reached_.data = 1;
            goal_reached_pub_.publish(goal_reached_);
            Switch_Mode(MODE::IDLE);
            return ;
        }
        SpinAndDock(goal_pose_, 1);
    }
    else {
        ROS_ERROR("[Path Executor]: WRONG MODE!");
    }
}

void PathExecutor::Switch_Mode(MODE next_mode) {
    working_mode_pre_ = working_mode_;
    working_mode_ = next_mode;
    if (working_mode_ == MODE::IDLE) {
        ROS_INFO_STREAM("[Path Executor]: switch mode to idle");
    }
    else if (working_mode_ == MODE::TRACKING) {
        ROS_INFO_STREAM("[Path Executor]: switch mode to tracking");
    }
    else if (working_mode_ == MODE::START_SPIN) {
        ROS_INFO_STREAM("[Path Executor]: switch mode to start spin");
    }
    else if (working_mode_ == MODE::END_SPIN) {
        ROS_INFO_STREAM("[Path Executor]: switch mode to end spin");
    }
    else {
        ROS_ERROR("[Path Executor]: WRONG MODE!");
    }
}

bool PathExecutor::is_XY_Reached(RobotState cur_pos, RobotState goal_pos) {
    if (cur_pos.distanceTo(goal_pos) < xy_tolerance_) {
    // if (fabs(cur_pos.x_ - goal_pos.x_) < xy_tolerance_) {
        return true;
    } else {
        return false;
    }
}

bool PathExecutor::is_Theta_Reached(RobotState cur_pos, RobotState goal_pos) {
    double theta_err = 0;
    Eigen::Vector2d cur_vec;
    Eigen::Vector2d goal_vec;
    cur_vec << cos(cur_pos.theta_), sin(cur_pos.theta_);
    goal_vec << cos(goal_pos.theta_), sin(goal_pos.theta_);
    theta_err = cur_vec.dot(goal_vec);

    theta_err = std::min(fabs(goal_pos.theta_ - cur_pos.theta_), fabs(Angle_Mod(goal_pos.theta_ - cur_pos.theta_)));
    if (fabs(theta_err) < theta_tolerance_) {
        return true;
    } else
        return false;
}

void PathExecutor::Velocity_Publish() {
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = velocity_state_.x_;
    vel_msg.linear.y = velocity_state_.y_;
    vel_msg.linear.z = 0;
    vel_msg.angular.x = 0;
    vel_msg.angular.y = 0;
    // vel_msg.angular.z = velocity_state_.x_*(-0.185) + velocity_state_.theta_;
    vel_msg.angular.z = velocity_state_.theta_;
    vel_pub_.publish(vel_msg);
}

double PathExecutor::Angle_Mod(double theta) {
    return fmod(theta + 2 * M_PI, 2 * M_PI);
}

void PathExecutor::Pose_Callback(const nav_msgs::Odometry::ConstPtr& pose_msg) {
    cur_pose_.x_ = pose_msg->pose.pose.position.x;
    cur_pose_.y_ = pose_msg->pose.pose.position.y;
    tf2::Quaternion q;
    tf2::fromMsg(pose_msg->pose.pose.orientation, q);
    tf2::Matrix3x3 qt(q);
    double _, yaw;
    qt.getRPY(_, _, yaw);
    cur_pose_.theta_ = yaw;
    // ROS_INFO("Current at (%f, %f, %f)", cur_pose_.x_, cur_pose_.y_, cur_pose_.theta_);
}

void PathExecutor::Emergency_Stop_Callback(const std_msgs::Bool::ConstPtr& emergency_stop_msg){
    if(emergency_stop_msg){
        emergency_stop = true;
    }
}

void PathExecutor::Obst_Callback(const costmap_converter::ObstacleArrayMsg::ConstPtr& obstacle_msg) {
    obstacles_.clear();
    for (costmap_converter::ObstacleMsg obst : obstacle_msg->obstacles) {
        ObstState obst_state;
        obst_state.x_ = obst.polygon.points[0].x;
        obst_state.y_ = obst.polygon.points[0].y;
        obst_state.vel_x_ = obst.velocities.twist.linear.x;
        obst_state.vel_y_ = obst.velocities.twist.linear.y;
        obstacles_.push_back(obst_state);
    }
    // ROS_INFO("[path executor] obstacle size: %ld", obstacles_.size());
}

std::vector<RobotState> PathExecutor::Orientation_Filter(std::vector<RobotState> origin_path) {
    std::vector<RobotState> path;
    double init_theta = cur_pose_.theta_;
    double goal_theta = goal_pose_.theta_;
    double theta_err = 0;
    double d_theta = 0;
    Eigen::Vector3d init;
    Eigen::Vector3d goal;
    // calculate rotate direction
    init << cos(init_theta), sin(init_theta), 0;
    goal << cos(goal_theta), sin(goal_theta), 0;
    
    double rotate_direction;
    if (init.cross(goal)(2) >= 0)
        rotate_direction_ = 1;
    else
        rotate_direction_ = -1;

    // theta_err = acos(init(0)*goal(0)+init(1)*goal(1));
    theta_err = fabs(Angle_Limit_Checking(goal_theta - init_theta));
    d_theta = rotate_direction_ * theta_err / (origin_path.size() - 1);

    RobotState point(origin_path.at(0).x_, origin_path.at(0).y_, init_theta);
    path.push_back(point);

    for (int i = 0; i < origin_path.size(); i++) {
        if (i != 0) {
            double theta;
            theta = Angle_Limit_Checking(path.at(i - 1).theta_ + d_theta);
            // cout << "theta = " << theta << endl;
            RobotState point(origin_path.at(i).x_, origin_path.at(i).y_, theta);
            path.push_back(point);
        }
    }

    // Rviz visualize processed path
    // geometry_msgs::PoseArray arr_msg;
    // arr_msg.header.frame_id = frame_;
    // arr_msg.header.stamp = ros::Time::now();
    // std::vector<geometry_msgs::Pose> poses;

    // for (int i = 0; i < path.size(); i++) {
    //     geometry_msgs::Pose pose;
    //     pose.position.x = path.at(i).x_;
    //     pose.position.y = path.at(i).y_;
    //     tf2::Quaternion q;
    //     q.setRPY(0, 0, path.at(i).theta_);
    //     pose.orientation.x = q.x();
    //     pose.orientation.y = q.y();
    //     pose.orientation.z = q.z();
    //     pose.orientation.w = q.w();
    //     poses.push_back(pose);
    // }
    // arr_msg.poses = poses;
    // pose_array_pub_.publish(arr_msg);

    return path;
}


int PathExecutor::Get_Next_Local_Goal() {
    int i = cur_path_idx_;
    if (i == -1) i = 0;
    double cur_dist_to_goal = cur_pose_.distanceTo(goal_pose_);
    while (1) {
        if (i == global_path_.size() - 1) {
            return i;
        }
        else {
            double now_pursue = global_path_[cur_path_idx_].distanceTo(goal_pose_);
            ROS_INFO_STREAM("line: " << __LINE__ << "now_pursue: " << now_pursue << " cur_dist_to_goal: " << cur_dist_to_goal);
            if (now_pursue > cur_dist_to_goal) {
                ++i;
                ROS_INFO("go this way");
            }
            else {
                return i;
            }
        }
    }
}

RobotState PathExecutor::Rolling_Window(RobotState cur_pos, std::vector<RobotState> path, double L_d) {
    if (path.empty()) {
        ROS_WARN_STREAM("pathExecutor Rolling_Window " << __LINE__ << " : path is empty");
        return cur_pos;
    }
    int k = 1;
    int last_k = 0;
    int d_k = 0;
    RobotState a;
    int a_idx = 0;
    RobotState b;
    int b_idx = 0;
    RobotState local_goal;
    bool if_b_asigned = false;
    double r = L_d;

    // ROS_INFO("%ld", path.size());
    for (int i = 0; i < path.size(); i++) {
        if (i == 1)
            last_k = 0;
        last_k = k;
        if (cur_pos.distanceTo(path.at(i)) >= r)
            k = 1;
        else
            k = 0;

        d_k = k - last_k;

        if (d_k == 1) {
            b = path.at(i);
            if_b_asigned = true;
            b_idx = i;
            a_idx = i - 1;
            break;
        }
    }

    if (!if_b_asigned) {
        double min = 1000000;
        for (int i = 0; i < path.size(); i++) {
            if (cur_pos.distanceTo(path.at(i)) < min) {
                min = cur_pos.distanceTo(path.at(i));
                b_idx = i;
                a_idx = i - 1;
                b = path.at(i);
            }
        }
    }
    if (a_idx == -1) {
        local_goal = path.at(b_idx);
    } else {
        a = path.at(a_idx);
        double d_ca = cur_pos.distanceTo(a);
        double d_cb = cur_pos.distanceTo(b);
        local_goal.x_ = a.x_ + (b.x_ - a.x_) * (r - d_ca) / (d_cb - d_ca);
        local_goal.y_ = a.y_ + (b.y_ - a.y_) * (r - d_ca) / (d_cb - d_ca);
        local_goal.theta_ = a.theta_;
    }

    // if (is_local_goal_final_reached_) {
    //     // cout << "local goal set to path.back()" << endl;
    //     local_goal = path.back();
    // }

    if (cur_pos.distanceTo(path.back()) < r + 0.01)
        local_goal = path.back();

    if (local_goal.distanceTo(path.back()) < 0.005) {
        local_goal = path.back();
        // is_local_goal_final_reached_ = true;
    }

    // for rviz visualization
    geometry_msgs::PoseStamped pos_msg;
    pos_msg.header.frame_id = frame_;
    pos_msg.header.stamp = ros::Time::now();
    pos_msg.pose.position.x = local_goal.x_;
    pos_msg.pose.position.y = local_goal.y_;

    tf2::Quaternion q;
    q.setRPY(0, 0, local_goal.theta_);
    pos_msg.pose.orientation.x = q.x();
    pos_msg.pose.orientation.y = q.y();
    pos_msg.pose.orientation.z = q.z();
    pos_msg.pose.orientation.w = q.w();
    local_goal_pub_.publish(pos_msg);

    return local_goal;
}

double PathExecutor::Angle_Limit_Checking(double theta) {
    return fmod(theta + 2 * M_PI, 2 * M_PI);
}

void PathExecutor::Omni_Controller(RobotState local_goal) {
    // RobotState local_goal = global_path_[cur_path_idx_];
    if (working_mode_ == MODE::IDLE) {
        velocity_state_.x_ = 0;
        velocity_state_.y_ = 0;
        velocity_state_.theta_ = 0;
        Velocity_Publish();
        return;
    }
    else if (working_mode_ == MODE::TRACKING) {
        double linear_velocity = 0.0;
        double angular_velocity = 0.0;

        int rotate_direction = 0; 1.544863;
        Eigen::Vector3d goal_vec(goal_pose_.x_, goal_pose_.y_, goal_pose_.theta_);
        Eigen::Vector3d cur_vec(cur_pose_.x_, cur_pose_.y_, cur_pose_.theta_);

        if (cur_vec.cross(goal_vec)(2) >= 0)
            rotate_direction = 1;
        else
            rotate_direction = -1;

        // transform local_goal to base_footprint frame
        Eigen::Vector2d goal_base_vec;
        Eigen::Vector2d local_goal_bf;
        Eigen::Matrix2d rot;
        goal_base_vec << (local_goal.x_ - cur_pose_.x_), (local_goal.y_ - cur_pose_.y_);
        rot << cos(-cur_pose_.theta_), -sin(-cur_pose_.theta_), sin(-cur_pose_.theta_), cos(-cur_pose_.theta_);
        local_goal_bf = rot * goal_base_vec;

        // local_goal.x_ = local_goal_bf(0);
        // local_goal.y_ = local_goal_bf(1);
        
        t_now_ = ros::Time::now();

        dt_ = (t_now_ - t_bef_).toSec();
        // ROS_INFO("dt: %f", dt_);

        if (is_XY_Reached(cur_pose_, goal_pose_)) {
            velocity_state_.x_ = 0;
            velocity_state_.y_ = 0;
        } else {
            linear_velocity = Extract_Velocity(VELOCITY::LINEAR, goal_pose_, linear_acceleration_);
            double direction = atan2(local_goal_bf(1), local_goal_bf(0));
            velocity_state_.x_ = linear_velocity * cos(direction);
            velocity_state_.y_ = linear_velocity * sin(direction);
        }

        if (is_Theta_Reached(cur_pose_, goal_pose_)) {
            velocity_state_.theta_ = 0;
        } else {
            angular_velocity = Extract_Velocity(VELOCITY::ANGULAR, goal_pose_, rotate_direction_ * angular_acceleration_);
            velocity_state_.theta_ = angular_velocity;
        }
    }

    if(emergency_stop){
        velocity_state_.x_ = 0;
        velocity_state_.y_ = 0;
        velocity_state_.theta_ = 0;
        ROS_ERROR("[Path Executor]: EMERGENCY STOP !!!!!!!!!!!!!!!");
    }

    Velocity_Publish();

    t_bef_ = t_now_;
}
void PathExecutor::Diff_Controller(RobotState local_goal) {
    if (working_mode_ == MODE::IDLE) {
        velocity_state_.x_ = 0;
        velocity_state_.y_ = 0;
        velocity_state_.theta_ = 0;
        Velocity_Publish();
        return;
    }
        // RobotState local_goal = global_path_[cur_path_idx_];
    double linear_velocity = 0.0;
    double angular_velocity;

    int rotate_direction = 0;
    // only for diff drive, dd = diff drive

    // double relative_rotate_first = 0;
    // double relative_w_rate = 1;
    
    double w_compress_ratio = 1;

    double relative_lpf_w[3] = {0.0};
    double relative_beta = 0.98;
    
    std_msgs::Float64 diff_radius;

  
    CalculateSteer(local_goal);
    // to avoid the trajetory deviate the path too much, make it rotate while the angle to large
    // if(((relative_steer_angle_ >= 0.5&& relative_steer_angle_ <= 2.64 ) || (relative_steer_angle_ >= 3.64 && relative_steer_angle_ <= 5.78 )))
    //     relative_rotate_first = 1;
    // else
    //     relative_rotate_first = 0;

    
    t_now_ = ros::Time::now();

    dt_ = (t_now_ - t_bef_).toSec();
    // ROS_INFO("dt: %f", dt_);
    
    if (is_XY_Reached(cur_pose_, goal_pose_)) {
        velocity_state_.x_ = 0;
        velocity_state_.theta_ = 0;
        keep_x = 0;
    }
    // else if(cur_pose_.distanceTo(goal_pose_) < 0.05){
    //     keep_x = 1;
    //     velocity_state_.theta_ = 0;
    //     linear_velocity = relative_drive_straight * Extract_Velocity(VELOCITY::LINEAR, goal_pose_, linear_acceleration_) * 0.8;
    //     velocity_state_.x_ = linear_velocity;
    // } 
    else {
        linear_velocity = relative_drive_straight_ * Extract_Velocity(VELOCITY::LINEAR, goal_pose_, linear_acceleration_);
        if(!keep_x){
            if (relative_radius_ == -1) {
                angular_velocity = 0;
            }
            else {
                angular_velocity = relative_is_couterclockwise_ * linear_velocity/(relative_radius_);
            }

            // if (cur_pose_.distanceTo(goal_pose_) < angular_brake_distance_ &&  abs(angular_velocity) > angular_max_vel_) {
            //     double shrink_rate = angular_max_vel_ / abs(angular_velocity);
            //     angular_velocity = angular_velocity * shrink_rate;
            //     linear_velocity = linear_velocity * shrink_rate;
            //     ROS_INFO_STREAM("pathExecutor " << "shirnk_rate:" << shrink_rate);
            // }
            
            // angu vel depressed by angu acc
            // if(angular_velocity > w_before + angular_acceleration_*dt_){
            //     angular_velocity = w_before + angular_acceleration_ *dt_;
            // }
            // else if(angular_velocity < w_before - angular_acceleration_*dt_){
            //     angular_velocity = w_before - angular_acceleration_*dt_;
            // }

            // w velocity lpf
            // relative_lpf_w[1] = velocity_state_.theta_;
            // relative_lpf_w[2] = relative_beta*relative_lpf_w[1] + (1-relative_beta)*relative_lpf_w[0];
            // relative_lpf_w[0] = relative_lpf_w[2];
            // velocity_state_.theta_ = relative_lpf_w[2]; 

            // to limit the angular accceleration 
            // if(abs(angular_velocity) < abs(abs(w_before)+angular_acceleration_*dt_) ){
            //     keep_x = 1;
            // }
            // else if(angular_velocity > w_before + angular_acceleration_ * dt_ ){
            //     angular_velocity = w_before + angular_acceleration_ * dt_;
            //     keep_x = 0;
            // }
            // else if(angular_velocity < w_before - angular_acceleration_ * dt_){
            //     angular_velocity = w_before - angular_acceleration_ * dt_;
            //     keep_x = 0;
            // }     
            // if (relative_rotate_first == 1)
            //     angular_velocity = angular_velocity / abs(angular_velocity) * angular_max_vel_;

            velocity_state_.theta_ = angular_velocity;
            velocity_state_.x_ = linear_velocity;
            // if (angular_velocity == 0) {
            //     velocity_state_.x_ = linear_velocity;
            // }
            // else {
            //     velocity_state_.x_ = relative_radius_ * angular_velocity * relative_is_couterclockwise_;
            // }
            w_before = velocity_state_.theta_;

        }
    
        diff_radius.data = relative_radius_;
        diff_radius_pub_.publish(diff_radius);
    }

    //remote emergency stop
    if(emergency_stop){
        velocity_state_.x_ = 0;
        velocity_state_.y_ = 0;
        velocity_state_.theta_ = 0;
        ROS_ERROR("[Path Executor]: EMERGENCY STOP !!!!!!!!!!!!!!!");
    }
    
    Velocity_Publish();
    

    t_bef_ = t_now_;
    keep_x = 0;
}

double PathExecutor::Extract_Velocity(VELOCITY vel_type, RobotState goal_pos, double acceleration) {
    // ROS_INFO("%lf %lf %lf, current local goal:%f (%lf %lf %lf)", cur_pose_.x_, cur_pose_.y_, cur_pose_.theta_, cur_path_idx_, global_path_[cur_path_idx_].x_, global_path_[cur_path_idx_].y_, global_path_[cur_path_idx_].theta_);
    double output_vel = 0;
    if (working_mode_ == MODE::TRACKING) {
        if (vel_type == VELOCITY::LINEAR) {
            RobotState origin(0, 0, 0);
            double last_vel = velocity_state_.distanceTo(origin);

            // acceleration
            double d_vel = acceleration * dt_;
            double xy_err = cur_pose_.distanceTo(goal_pose_);
            linear_integration_ += xy_err;

            output_vel = (xy_err + linear_margin_) * linear_kp_ ;


            if (output_vel > std::min(linear_max_vel_, last_vel + d_vel)) {
                output_vel = std::min(linear_max_vel_, last_vel + d_vel);
            }

            // if (output_vel < last_vel - d_vel) {
            //     output_vel = last_vel - d_vel;
            // }

            // if (xy_err > 0.4 && output_vel < last_vel) {
            //     output_vel = last_vel;
            // }
            // if (xy_err > 0.4 && output_vel < last_vel) {
            //     output_vel = last_vel;
            // }
            // if (output_vel < 0.01) {
            //     output_vel = 0;
            // }
        }

        else if (vel_type == VELOCITY::ANGULAR) {
            double d_vel = acceleration * dt_;
            output_vel = velocity_state_.theta_ + d_vel;
            double theta_err = (Angle_Limit_Checking(goal_pos.theta_ - cur_pose_.theta_));

            // double angular_brake_distance = theta_err / acceleration;
            // if (fabs(theta_err) < angular_brake_distance) {
            //     output_vel = velocity_state_.theta_ - d_vel;
            //     // output_vel = theta_err * angular_kp_;
            // }
            double last_vel = velocity_state_.theta_;
            if (fabs(theta_err) < angular_brake_distance_) {
                // output_vel = theta_err * angular_kp_ * (last_vel) * 1.3;
                output_vel = theta_err * angular_kp_;
            }

            // Saturation
            // if (output_vel > angular_max_vel_)
            //     output_vel = angular_max_vel_;
            // if (output_vel < -angular_max_vel_)
            //     output_vel = -angular_max_vel_;
        }
    }
    return output_vel;
}

void PathExecutor::SpinAndDock(RobotState local_goal, bool opt) {
    ROS_INFO_STREAM("[Path Executor]: in spin and dock");
    if (opt == 0) {
        CalculateSteer(local_goal);
        if (relative_drive_straight_ == -1) {
            relative_steer_angle_ = M_PI - relative_steer_angle_;
        }
        // ROS_INFO_STREAM("[Path Executor]: relative_steer_angle_ " << relative_steer_angle_ << " " << deviate_margin_rad_);
        if (relative_steer_angle_ > deviate_margin_rad_) {
            double angular_velocity = relative_steer_angle_ * spin_kp_ * relative_is_couterclockwise_ * relative_drive_straight_;
            // ROS_INFO_STREAM("[path exec]" << spin_kp_ << " " << relative_is_couterclockwise_ << " " << relative_drive_straight_);
            if (angular_velocity > angular_max_vel_) angular_velocity = angular_max_vel_;
            if (angular_velocity < -angular_max_vel_) angular_velocity = -angular_max_vel_;
            velocity_state_.x_ = 0;
            velocity_state_.theta_ = angular_velocity;
            Velocity_Publish();
        }
        else {
            Switch_Mode(MODE::TRACKING);
            velocity_state_.x_ = 0;
            velocity_state_.theta_ = 0;
            Velocity_Publish();
            t_bef_ = ros::Time::now();
        }
    }
    else {
        double theta_err = local_goal.theta_ - cur_pose_.theta_;
        while (theta_err > M_PI) theta_err = theta_err - 2 * M_PI;
        while (theta_err < -M_PI) theta_err = theta_err + 2 * M_PI;
        // ROS_INFO_STREAM("[Path Executor]: ENDSPIN " << theta_err << " " << (fabs(theta_err) < theta_tolerance_));

        double angular_velocity = theta_err * spin_kp_;

        double dt_ = (ros::Time::now() - t_bef_).toSec();
        double last_angular_vel = velocity_state_.theta_;
        ROS_INFO_STREAM("pathExec ENDSPIN " << dt_ << " " << last_angular_vel);
        // if (angular_velocity > std::min(angular_max_vel_, last_angular_vel + angular_acceleration_ * dt_)) angular_velocity = std::min(angular_max_vel_, last_angular_vel + angular_acceleration_ * dt_);
        // if (angular_velocity < std::max(-angular_max_vel_, last_angular_vel - angular_acceleration_ * dt_)) angular_velocity = std::max(-angular_max_vel_, last_angular_vel - angular_acceleration_ * dt_);
        if (angular_velocity > angular_max_vel_) angular_velocity = angular_max_vel_;
        if (angular_velocity < -angular_max_vel_) angular_velocity = -angular_max_vel_;

        velocity_state_.x_ = 0;
        velocity_state_.theta_ = angular_velocity;
        Velocity_Publish();
        t_bef_ = ros::Time::now();
    }
}

void PathExecutor::CalculateSteer(RobotState local_goal) {
    relative_distance_ = sqrt(pow(local_goal.x_-cur_pose_.x_,2)+pow(local_goal.y_-cur_pose_.y_,2));
    // ROS_INFO_STREAM("path executor: relative_distance: " << relative_distance_);
 
    // a*b=|a||b|*cos(theta) ==> theta = acos(a*b/|a||b|)
    relative_steer_angle_ = acos((cos(cur_pose_.theta_)*(local_goal.x_ - cur_pose_.x_)+sin(cur_pose_.theta_)*(local_goal.y_ - cur_pose_.y_))/(sqrt(pow(local_goal.x_-cur_pose_.x_,2)+pow(local_goal.y_-cur_pose_.y_,2))));

    // sin(theta)= a X b to define whether the clockwise is <=0 clockwise ; >0 counterclockwise 
    if((cos(cur_pose_.theta_)*(local_goal.y_ - cur_pose_.y_)-sin(cur_pose_.theta_)*(local_goal.x_ - cur_pose_.x_))<=0)
        relative_is_couterclockwise_ = -1;
    else
        relative_is_couterclockwise_ = 1;

     
    // if angle > 90 degree , change to reverse driving
    // ROS_INFO_STREAM("path executor: relative_steer_angle: " << relative_steer_angle);
    if(relative_steer_angle_ <= M_PI/2) {
        if (relative_steer_angle_ <= straight_margin_rad_) {
            relative_radius_ = -1;
        }
        else {
            relative_radius_ = relative_distance_/(2*(sin(relative_steer_angle_)));
        }
        relative_drive_straight_ = 1;
    }
    else {
        if(can_reverse_drive_== 0) {
            if (relative_steer_angle_-M_PI/2 <= straight_margin_rad_) {
                relative_radius_ = -1;
            }
            else {
                relative_radius_ = relative_distance_/(2*(sin(relative_steer_angle_-M_PI/2)));
            }
            relative_drive_straight_ = 1;
        }else{
            if (M_PI-relative_steer_angle_ <= straight_margin_rad_) {
                relative_radius_ = -1;
            }
            else {
                relative_radius_ = relative_distance_/(2*(sin(M_PI-relative_steer_angle_)));
            }
            relative_drive_straight_ = -1;
        }
    }
    // ROS_INFO_STREAM("path executor: relative_radius: " << relative_radius << " " << relative_steer_angle);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "PathExecutor");
    ros::NodeHandle nh("");
    ros::NodeHandle nh_local("~");
    PathExecutor PathExecutor_int(nh, nh_local);

    ros::spin();
}