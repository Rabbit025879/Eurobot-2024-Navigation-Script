#include <ros/console.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

#include <eigen3/Eigen/Dense>
#include <cmath>

// msgs
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Char.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/GetPlan.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

// srvs
#include <std_srvs/Empty.h>

// teb and g2o
// #include <teb_local_planner/obstacles.h>
// #include <g2o_types/edge_obstacle.h>
// #include <g2o_types/edge_dynamic_obstacle.h>

// obstacles
#include <costmap_converter/ObstacleArrayMsg.h>

/** process of pathTracker when each event happen **/

// (1) normal mode
//     goal received                                    reached
// IDLE ----------> GLOBAL_PATH_RECEIVED --> TRACKING ----------> IDLE
// 
// (2) goal changed when tracking
//    goal received                                  goal received              reached
// IDLE ----------> GLOBAL_PATH_RECEIVED --> TRACKING ---------->  TRANSITION ----------> IDLE
// 
// (3) goal is so closed to rival, stopped by nav_main. 
//    goal received                               rival near send -1
// IDLE ----------> GLOBAL_PATH_RECEIVED --> TRACKING ---------->  IDLE
// 
// (4) goal is blocked. fail to get plan
//    cannot get plan 
// IDLE ----------> IDLE
// 
// (5) goal is blocked when tracking
//    goal received                                  something block goal, set local goal to goal
// IDLE ----------> GLOBAL_PATH_RECEIVED --> TRACKING ---------------------------------------->  IDLE

enum class MODE {
    IDLE = 0,
    TRACKING,
    START_SPIN,
    END_SPIN
};

enum class VELOCITY {
    LINEAR = 0,
    ANGULAR
};

// sim -> nav_msgs, real -> geometry_msgs
enum class ODOM_CALLBACK_TYPE {
    nav_msgs_Odometry = 0,
    geometry_msgs_PoseWithCovarianceStamped
};

class RobotState {
   public:
    RobotState(double x, double y, double theta);
    RobotState() {}
    double x_;
    double y_;
    double theta_;
    // Eigen::Vector3d getVector();
    double distanceTo(RobotState);
    // Eigen::Vector2d orientationUnitVec() const { return Eigen::Vector2d(cos(theta_), sin(theta_)); }
    void operator=(const RobotState& rhs) {
        this->x_ = rhs.x_;
        this->y_ = rhs.y_;
        this->theta_ = rhs.theta_;
    }
};

class ObstState {
   public:
    ObstState(double x, double y, double vel_x, double vel_y);
    ObstState() {}
    double x_;
    double y_;
    double vel_x_;
    double vel_y_;
};

class PathExecutor {
   public:
    PathExecutor(ros::NodeHandle& nh, ros::NodeHandle& nh_local);
    ~PathExecutor();

    bool initializeParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
    void initialize();
// 
   private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_local_;
    ros::ServiceServer params_srv_;

    // subscriber
    ros::Subscriber pose_sub_;
    ros::Subscriber goal_sub_;
    ros::Subscriber obst_sub_;
    ros::Subscriber emergency_stop_sub_;  //use topic to stop the real robot
    // ros::Subscriber action_sub_;
    int pose_type_;
    void Pose_Callback(const nav_msgs::Odometry::ConstPtr& pose_msg);
    // void Pose_type0_Callback(const nav_msgs::Odometry::ConstPtr& pose_msg);
    // void Pose_type1_Callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg);
    void Goal_Callback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg);
    void Obst_Callback(const costmap_converter::ObstacleArrayMsg::ConstPtr& obstacle_msg);
    void Emergency_Stop_Callback(const std_msgs::Bool::ConstPtr& emergency_stop_msg);

    // void actionCallback(const std_msgs::Bool::ConstPtr& action_msg);

    // geometry_msgs::PoseArray obstacle_pose_;
    // geometry_msgs::PoseArray rival_pose_;
    // ODOM_CALLBACK_TYPE odom_callback_type_;

    // Publisher
    ros::Publisher vel_pub_;
    ros::Publisher path_pub_; // for debug
    ros::Publisher marker_pub_; // for obstacles debug
    ros::Publisher diff_radius_pub_; // for diff drive radius debug
    ros::Publisher local_goal_pub_;
    // ros::Publisher pose_array_pub_;
    ros::Publisher goal_reached_pub_;

    std_msgs::Char goal_reached_;

    void Velocity_Publish();

    // Client
    bool Get_Global_Path(RobotState, RobotState);

    RobotState goal_pose_;
    RobotState cur_pose_;
    RobotState velocity_state_;

    bool is_local_goal_final_reached_;
    // bool is_reached_target_range_;

    bool is_XY_Reached(RobotState cur_pose, RobotState goal_pose);
    bool is_Theta_Reached(RobotState cur_pose, RobotState goal_pose);

    // // Goal request from Main and Path received from global planner
    std::vector<RobotState> global_path_;
    // std::vector<RobotState> global_path_past_;
    double cur_path_idx_;

    // // timer setup
    ros::Timer timer_;
    void Timer_Callback(const ros::TimerEvent& e);

    MODE working_mode_;
    // MODE working_mode_pre_;

    void Switch_Mode(MODE next_mode);
    // bool is_global_path_switched_;

    // // controller parameter
    std::string frame_;
    std::string robot_type_;
    // bool p_active_;
    double control_frequency_;
    double lookahead_d_;
    // double blocked_lookahead_d_;
    // double waiting_timeout_;

    double linear_kp_;
    double linear_max_vel_;
    double linear_acceleration_;
    double linear_brake_distance_;
    double linear_margin_;
    // double linear_transition_vel_;
    // double linear_transition_acc_;
    // // double linear_tracking_vel_;
    double xy_tolerance_;
    // double linear_min_brake_distance_;
    // double linear_brake_distance_ratio_;
    // // velocity profile type : linear, smooth_step
    // std::string linear_acceleration_profile_;
    // std::string linear_deceleration_profile_;
    double can_reverse_drive_;
    bool emergency_stop = 0;
    double w_before = 0;
    bool keep_x = 0 ;
    double x_before ;

    double angular_kp_;
    double angular_max_vel_;
    double angular_acceleration_;
    double angular_brake_distance_;
    double angular_margin_;
    // double angular_transition_vel_;
    // double angular_transition_acc_;
    double theta_tolerance_;
    // double theta_err;
    // // velocity profile type : p_control, linear, smooth_step
    // std::string angular_acceleration_profile_e
    // std::string angular_deceleration_profile_;
    double linear_integration_;

    double Angle_Mod(double theta);
    // double Velocity_Profile(VELOCITY, RobotState cur_pos, RobotState goal_pos, RobotState velocity_state, double acceleration_sign);
    // double max_linear_vel_reached_;

    // // Path post process
    std::vector<RobotState> Orientation_Filter(std::vector<RobotState>);
    int rotate_direction_;

    // // rollingWindow method flag
    // RobotState Rolling_Window(RobotState cur_pos, std::vector<RobotState> global_path, double R);

    // void Omni_Controller(RobotState local_goal, RobotState cur_pos);

    ros::Time t_bef_;
    ros::Time t_now_;
    ros::Time run_time_;
    double dt_;

    ros::Time dacc_clock_x_, dacc_clock_y_;
    double has_start_dacc_x_, has_start_dacc_y_;

    // // check if goal is blocked after goal received
    // bool new_goal;
    bool is_goal_blocked_;

    // obstacle
    std::vector<ObstState> obstacles_;
    int replan_cnt_;

    // optimize path part(using g2o from teb_local_planner)
    // double weight_obstacle_;
    double min_obst_dist_;
    int replan_threshold_;
    int replan_frequency_divider_;
    int replan_idx_;

    // SpinAdnDock Param
    double spin_kp_;
    double dock_kp_;
    double sd_start_dist_;
    bool start_final_spin_;
    double relative_drive_straight_;
    double spin_and_dock_start_dist_;

    int Get_Next_Local_Goal();   
    void Omni_Controller(RobotState local_goal);
    void Diff_Controller(RobotState local_goal);
    double Extract_Velocity(VELOCITY vel_type, RobotState goal_pos, double acceleration);
    double Angle_Limit_Checking(double theta);
    void SetDrivingArg(bool &relative_drive_straight, bool &relative_is_couterclockwise, double &relative_steer_angle, double &relative_radius, double &relative_distance, RobotState local_goal);
    void SpinAndDock(RobotState local_goal, bool need_forward);

    RobotState Rolling_Window(RobotState cur_pos, std::vector<RobotState> path, double L_d);
};