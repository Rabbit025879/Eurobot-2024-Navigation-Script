#include "path_solver.h"

//DEBUG
double buffer[10] = {0.0};
bool print_once[10] = {0};
//Obstacles position
std::vector<Point>  obs_pose_sim;
std::vector<Point>  obs_pose_real;
std::vector<Point>  obs_pose;
//Goal reached?
bool is_goal_reached = 0;
//Target goal
Point goal;
//Robot pose
Point pose_sim;
Point pose_ekf;
Point pose;
//Convert
nav_msgs::Odometry OS;
geometry_msgs::PoseWithCovarianceStamped OR;
geometry_msgs::PoseStamped G;

void DEBUG(double data, int i){
    if(data != buffer[i]) print_once[i] = 0;  
    buffer[i] = data;        
    if(print_once[i] == 0){
        ROS_FATAL("path_solver -> Data[%d] = %lf", i, buffer[i]);
        print_once[i] = 1;
    }
}

double dis_point_to_point(Point a, Point b){
    double distance = 0.0;
    distance = sqrt(pow((a.x-b.x),2)+pow((a.y-b.y),2));
    return(distance);
}
void Line_tan::line_param_insert_pc(Point point_on_line, Point point_outside, double radius){
    Line_tan::point_on_line = point_on_line;
    Line_tan::point_outside = point_outside;
    //The parameter of the equation which aims to find the slope of tangent line
    Line_tan::m2 = pow((point_outside.x-point_on_line.x),2)-pow(radius,2);
    Line_tan::m1 = 2*((point_outside.x-point_on_line.x)*(point_on_line.y-point_outside.y));
    Line_tan::m0 = pow((point_outside.y-point_on_line.y),2)-pow(radius,2);
    //Paramerter calculate
    Line_tan::tan.I = (-m1+sqrt(pow(m1,2)-(4*m2*m0)))/(2*m2);
    Line_tan::tan.II = (-m1-sqrt(pow(m1,2)-(4*m2*m0)))/(2*m2);
    // ROS_WARN("goal tangent slope : %lf,%lf", Line_tan::tan.I, Line_tan::tan.II);
    //The parameter of the equation of the two tangent line
    //ax+by+c = 0
    Line_tan::a1 = Line_tan::tan.I;
    Line_tan::b1 = -1;
    Line_tan::c1 = point_on_line.y-Line_tan::tan.I*point_on_line.x;
    Line_tan::a2 = Line_tan::tan.II;
    Line_tan::b2 = -1;
    Line_tan::c2 = point_on_line.y-Line_tan::tan.II*point_on_line.x;
    // ROS_WARN("1:(%lf,%lf,%lf), 2:(%lf,%lf,%lf)", a1,b1,c1,a2,b2,c2);
 }
 void Line_tan::line_param_insert_cc(Point obs1, Point obs2, double radius){
    ROS_INFO("obs1 -> (%lf, %lf), obs2 -> (%lf, %lf)", obs1.x, obs1.y, obs2.x, obs2.y);
    double delta_x = obs1.x-obs2.x;
    double delta_y = obs1.y-obs2.y;   
    ROS_INFO("delta -> (%lf, %lf)", delta_x, delta_y);
    //The parameter of the equation of the four tangent line
    //Parameter calculate
    Line_tan::tan.I = delta_y/delta_x;
    Line_tan::tan.II = ((2*delta_x*delta_y)+sqrt(pow((2*delta_x*delta_y),2)-4*(pow(delta_x,2)-pow(2*radius,2))*(pow(delta_y,2)-pow(2*radius,2))))/(2*(pow(delta_x,2)-pow(2*radius,2)));
    Line_tan::tan.III = ((2*delta_x*delta_y)-sqrt(pow((2*delta_x*delta_y),2)-4*(pow(delta_x,2)-pow(2*radius,2))*(pow(delta_y,2)-pow(2*radius,2))))/(2*(pow(delta_x,2)-pow(2*radius,2)));
    ROS_INFO("tan.I -> %lf", Line_tan::tan.I);
    ROS_INFO("tan.II -> %lf", Line_tan::tan.II);
    ROS_INFO("tan.III -> %lf", Line_tan::tan.III);
    // ROS_INFO("tan.II(sqrt) -> %lf", pow((2*delta_x*delta_y),2)-4*(pow(delta_x,2)-pow(2*radius,2))*(pow(delta_y,2)-pow(2*radius,2)));
    //ax+by+c = 0
    Line_tan::a1 = Line_tan::tan.I;
    Line_tan::b1 = -1;
    Line_tan::c1 = (sqrt((a1*a1)+1)*radius)-(a1*obs1.x)+obs1.y;
    Line_tan::a2 = Line_tan::tan.I;
    Line_tan::b2 = -1;
    Line_tan::c2 = -(sqrt((a2*a2)+1)*radius)-(a2*obs1.x)+obs1.y;
    Line_tan::a3 = Line_tan::tan.II;
    Line_tan::b3 = -1;
    Line_tan::c3 = -(sqrt((a3*a3)+1)*radius)-(a3*obs1.x)+obs1.y;
    Line_tan::a4 = Line_tan::tan.III;
    Line_tan::b4 = -1;
    Line_tan::c4 = (sqrt((a4*a4)+1)*radius)-(a4*obs1.x)+obs1.y;
    ROS_WARN("(c3, c4) -> (%lf, %lf)", c3, c4);
 }
Point Line_tan::line_intersection_22(Line_tan t, Line_tan u, int n){
    Point intersection;
    double delta, delta_x, delta_y;
    if(n == 11){
        delta = (t.a1*u.b1)-(t.b1*u.a1);
        delta_x = -(t.c1*u.b1)+(t.b1*u.c1);
        delta_y = -(t.a1*u.c1)+(t.c1*u.a1);
    }
    if(n == 12){
        delta = (t.a1*u.b2)-(t.b1*u.a2);
        delta_x = -(t.c1*u.b2)+(t.b1*u.c2);
        delta_y = -(t.a1*u.c2)+(t.c1*u.a2);
    }
    if(n == 21){
        delta = (t.a2*u.b1)-(t.b2*u.a1);
        delta_x = -(t.c2*u.b1)+(t.b2*u.c1);
        delta_y = -(t.a2*u.c1)+(t.c2*u.a1);
    }
    if(n == 22){
        delta = (t.a2*u.b2)-(t.b2*u.a2);
        delta_x = -(t.c2*u.b2)+(t.b2*u.c2);
        delta_y = -(t.a2*u.c2)+(t.c2*u.a2);
    } 
    intersection.x = delta_x/delta;
    intersection.y = delta_y/delta;
    // ROS_INFO("intersection : (%lf, %lf)", intersection.x, intersection.y);
    return(intersection);
}
Point Line_tan::line_intersection_14(Line_tan t, Line_tan u, int n, Point comfirm_point){
    // ROS_INFO("<-------- Intersection Calculating -------->");
    Point intersection;
    double delta, delta_x, delta_y;

    // ROS_WARN("comfirm_point = (%lf, %lf)", comfirm_point.x, comfirm_point.y);
    // ROS_WARN("1 : (%lf, %lf, %lf)", t.a1, t.b1, t.c1);
    // ROS_WARN("2 : (%lf, %lf, %lf)", t.a2, t.b2, t.c2);   
    if((t.a1*comfirm_point.x + t.b1*comfirm_point.y + t.c1) <= (t.a2*comfirm_point.x + t.b2*comfirm_point.y + t.c2)){
        // ROS_INFO("a1");
        if(n == 11){
            delta = (t.a1*u.b1)-(t.b1*u.a1);
            delta_x = -(t.c1*u.b1)+(t.b1*u.c1);
            delta_y = -(t.a1*u.c1)+(t.c1*u.a1);
        }
        if(n == 12){
            delta = (t.a1*u.b2)-(t.b1*u.a2);
            delta_x = -(t.c1*u.b2)+(t.b1*u.c2);
            delta_y = -(t.a1*u.c2)+(t.c1*u.a2);
        }
        if(n == 13){
            // delta = (t.a1*u.b3)-(t.b1*u.a3);
            // delta_x = -(t.c1*u.b3)+(t.b1*u.c3);
            // delta_y = -(t.a1*u.c3)+(t.c1*u.a3);
            delta = 1;
            delta_x = -1;
            delta_y = -1;
        }
        if(n == 14){
            // delta = (t.a1*u.b4)-(t.b1*u.a4);
            // delta_x = -(t.c1*u.b4)+(t.b1*u.c4);
            // delta_y = -(t.a1*u.c4)+(t.c1*u.a4);
            delta = 1;
            delta_x = -1;
            delta_y = -1;
        }
    }
    else if((t.a1*comfirm_point.x + t.b1*comfirm_point.y + t.c1) > (t.a2*comfirm_point.x + t.b2*comfirm_point.y + t.c2)){
        // ROS_INFO("a2");
        if(n == 11){
            delta = (t.a2*u.b1)-(t.b2*u.a1);
            delta_x = -(t.c2*u.b1)+(t.b2*u.c1);
            delta_y = -(t.a2*u.c1)+(t.c2*u.a1);
        }
        if(n == 12){
            delta = (t.a2*u.b2)-(t.b2*u.a2);
            delta_x = -(t.c2*u.b2)+(t.b2*u.c2);
            delta_y = -(t.a2*u.c2)+(t.c2*u.a2);
        }
        if(n == 13){
            // delta = (t.a2*u.b3)-(t.b2*u.a3);
            // delta_x = -(t.c2*u.b3)+(t.b2*u.c3);
            // delta_y = -(t.a2*u.c3)+(t.c2*u.a3);
            delta = 1;
            delta_x = -1;
            delta_y = -1;
        }
        if(n == 14){
            // delta = (t.a2*u.b4)-(t.b2*u.a4);
            // delta_x = -(t.c2*u.b4)+(t.b2*u.c4);
            // delta_y = -(t.a2*u.c4)+(t.c2*u.a4);
            delta = 1;
            delta_x = -1;
            delta_y = -1;
        }
    }
    else    ROS_ERROR("the point is not on the robot_line !!");

    intersection.x = delta_x/delta;
    intersection.y = delta_y/delta;
    ROS_INFO("intersection_cc : (%lf, %lf)", intersection.x, intersection.y);
    // ROS_INFO("<------------------------------------------>");
    return(intersection);
}
Point Line_tan::line_intersection_12(Line_tan t, Line_tan u, Point comfirm_point){
    Point intersection;
    Point sol_1;
    Point sol_2;
    double dis_1 = 0.0;
    double dis_2 = 0.0;
    double delta, delta_x, delta_y;
    // ROS_FATAL("comfirm point -> (%lf, %lf)", comfirm_point.x, comfirm_point.y);
    if((t.a1*comfirm_point.x + t.b1*comfirm_point.y + t.c1) <= (t.a2*comfirm_point.x + t.b2*comfirm_point.y + t.c2)){
        // solution 1
        delta = (t.a1*u.b1)-(t.b1*u.a1);
        delta_x = -(t.c1*u.b1)+(t.b1*u.c1);
        delta_y = -(t.a1*u.c1)+(t.c1*u.a1);
        sol_1.x = delta_x/delta;
        sol_1.y = delta_y/delta;
        // solution 2
        delta = (t.a1*u.b2)-(t.b1*u.a2);
        delta_x = -(t.c1*u.b2)+(t.b1*u.c2);
        delta_y = -(t.a1*u.c2)+(t.c1*u.a2);
        sol_2.x = delta_x/delta;
        sol_2.y = delta_y/delta;
    }
    else if((t.a1*comfirm_point.x + t.b1*comfirm_point.y + t.c1) > (t.a2*comfirm_point.x + t.b2*comfirm_point.y + t.c2)){
        // solution 1
        delta = (t.a2*u.b1)-(t.b2*u.a1);
        delta_x = -(t.c2*u.b1)+(t.b2*u.c1);
        delta_y = -(t.a2*u.c1)+(t.c2*u.a1);
        sol_1.x = delta_x/delta;
        sol_1.y = delta_y/delta;
        // solution 2
        delta = (t.a2*u.b2)-(t.b2*u.a2);
        delta_x = -(t.c2*u.b2)+(t.b2*u.c2);
        delta_y = -(t.a2*u.c2)+(t.c2*u.a2);
        sol_2.x = delta_x/delta;
        sol_2.y = delta_y/delta;
    }
    
    dis_1 = dis_point_to_point(comfirm_point, sol_1) + dis_point_to_point(sol_1, goal);
    dis_2 = dis_point_to_point(comfirm_point, sol_2) + dis_point_to_point(sol_2, goal);

    if(dis_1 <= dis_2)  intersection = sol_1;
    else if(dis_2 < dis_1)  intersection = sol_2;

    ROS_INFO("intersection_12 : (%lf, %lf)", intersection.x, intersection.y);
    return(intersection);
}

//Data convert into Point
Point Point_convert(double x, double y){
    Point data;
    data.x = x;
    data.y = y;
    return(data);
}
double dis_line_to_point(Point line_a, Point line_b, Point point){
    double slope = 0.0;
    double distance = 0.0;
    slope = (line_b.y - line_a.y)/(line_b.x - line_a.x);
    distance = fabs(slope*point.x-point.y+(line_a.y - (slope*line_a.x)))/sqrt(pow(slope,2)+1);
    return(distance);
}
//Bubble sort for array
void sort(double master[2], double slave[2]){
    // double buffer = 0.0;
    int tmp = 0;
    int i = 0;
    int j = 0;
    for(i = 2-1; i > 0; i--){
        for(j = 0; j <= i-1; j++){
            if(master[j] > master[j+1]){
                // buffer = master[j];
                // tmp = slave[j];
                // master[j] = master[j+1];
                // slave[j] = slave[j+1];
                // master[j+1] = buffer;
                // slave[j+1] = tmp;
                std::swap(slave[j], slave[j+1]);
            }
        }
    }
}
//Bubble sort for vector
void sort(std::vector<double> master, std::vector<Point> slave){
    // double buffer = 0.0;
    int tmp = 0;
    int i = 0;
    int j = 0;
    for(i = 2-1; i > 0; i--){
        for(j = 0; j <= i-1; j++){
            if(master[j] > master[j+1]){
                // buffer = master[j];
                // tmp = slave[j];
                // master[j] = master[j+1];
                // slave[j] = slave[j+1];
                // master[j+1] = buffer;
                // slave[j+1] = tmp;
                std::swap(slave[j], slave[j+1]);
            }
        }
    }
}
//Bubble sort for cost
std::vector<std::vector<Point>> sort_path(std::vector<double> cost, std::vector<std::vector<Point>> multipath){
    double buffer = 0.0;
    // std::vector<Point> tmp;
    int i = 0;
    int j = 0;
    for(i = cost.size()-1; i > 0; i--){
        for(j = 0; j <= i-1; j++){
            if(cost[j] > cost[j+1]){
                buffer = cost[j];
                cost[j] = cost[j+1];
                cost[j+1] = buffer;
                multipath[j].swap(multipath[j+1]);
            }
        }
    }
    // tmp.clear();
    ROS_INFO("Smallest cost -> %lf", cost[0]);
    return(multipath);
}
//Path selection
Point point_select(Point a, Point b, Point c, Point d, int rank){
    Point Point;
    int point = 0;
    bool cut = 0;
    double path_dis[4] = {0.0};
    int sequence[4] = {0};
    double sequence_double[4] = {0.0,1.0,2.0,3.0};
    bool valid[4] = {false};
    ROS_INFO("Point_select -> (%lf,%lf)", a.x, a.y);
    ROS_INFO("Point_select -> (%lf,%lf)", b.x, b.y);
    ROS_INFO("Point_select -> (%lf,%lf)", c.x, c.y);
    ROS_INFO("Point_select -> (%lf,%lf)", d.x, d.y);
    // double distance[4] = {0.0};
    //Determine is the point valid
    if(a.x >= 0.2 && a.x <= 2.8 && a.y >= 0.2 && a.y <= 1.8)  valid[0] = true;
    if(b.x >= 0.2 && b.x <= 2.8 && b.y >= 0.2 && b.y <= 1.8)  valid[1] = true;
    if(c.x >= 0.2 && c.x <= 2.8 && c.y >= 0.2 && c.y <= 1.8)  valid[2] = true;    
    if(d.x >= 0.2 && d.x <= 2.8 && d.y >= 0.2 && d.y <= 1.8)  valid[3] = true;   
    // distance[0] = dis_line_to_point(a, goal, Point_convert(obs_pose_x[0], obs_pose_y[0]));
    // distance[1] = dis_line_to_point(b, goal, Point_convert(obs_pose_x[0], obs_pose_y[0]));
    // distance[2] = dis_line_to_point(c, goal, Point_convert(obs_pose_x[0], obs_pose_y[0]));
    // distance[3] = dis_line_to_point(d, goal, Point_convert(obs_pose_x[0], obs_pose_y[0]));
    // if(distance[0] < r) valid[0] = false;
    // if(distance[1] < r) valid[1] = false;
    // if(distance[2] < r) valid[2] = false;
    // if(distance[3] < r) valid[3] = false;
    //Determine the priority
    path_dis[0] = dis_point_to_point(pose,a)+dis_point_to_point(goal,a);
    path_dis[1] = dis_point_to_point(pose,b)+dis_point_to_point(goal,b);
    path_dis[2] = dis_point_to_point(pose,c)+dis_point_to_point(goal,c);
    path_dis[3] = dis_point_to_point(pose,d)+dis_point_to_point(goal,d);
    sort(path_dis,sequence_double);
    for(int j=0; j<4; j++)  sequence[j] = int(sequence_double[j]);
    //Determine the point base on priority & valid
    //find the nearest point
    if(rank == 0){
        if(valid[sequence[0]])  point = sequence[0];
        else if(valid[sequence[1]]) point = sequence[1];
        else if(valid[sequence[2]]) point = sequence[2];
        else if(valid[sequence[3]]) point = sequence[3];
        else    point = 4;
    }
    //find the second nearest point
    else if(rank == 1){
        for(int k=0; k<4; k++){
            if(cut == 1)    break;
            if(valid[sequence[k]]){
                for(int g=k+1; g<4; g++){
                    if(valid[sequence[g]]){
                        point = sequence[g];
                        cut = 1;
                        break;
                    }   
                }
            }
        }
        if(cut == 1)    cut = 0;
        else    point = 4;
    }
    else    ROS_WARN("invalid rank");

    //Convert sequence into point
    if(point == 0)  Point = a;
    if(point == 1)  Point = b;
    if(point == 2)  Point = c;
    if(point == 3)  Point = d;
    if(point == 4)  ROS_ERROR("No Valid Point !!!");

    // ROS_INFO("rank : %d", rank);
    ROS_INFO("priority : %d,%d,%d,%d", sequence[0],sequence[1],sequence[2],sequence[3]);
    ROS_INFO("valid : %d,%d,%d,%d",valid[0],valid[1],valid[2],valid[3]);
    // ROS_INFO("distance : %lf,%lf,%lf,%lf",distance[0],distance[1],distance[2],distance[3]);
    // ROS_INFO("path %d selected!",point+1);
    // ROS_INFO("Point_select -> (%lf,%lf)", Point.x, Point.y);

    return(Point);
}
//Get target goal for simulation
void getobs_sim(const geometry_msgs::PoseArray& obs_detector_sim){
    int i = 0;
    obs_pose_sim.clear();
    obs_pose_sim.reserve(obs_detector_sim.poses.size());
    for(i=0; i<obs_detector_sim.poses.size(); i++){
        obs_pose_sim.push_back(Point_convert(obs_detector_sim.poses[i].position.x, obs_detector_sim.poses[i].position.y));
        // ROS_INFO("obs %d : (%lf,%lf)",i, obs_pose[i].x, obs_pose[i].y);
    }
}
//Get target goal for real
void getobs_real(const obstacle_detector::Obstacles& obs_detector_real){
    int i = 0;
    obs_pose_real.clear();
    obs_pose_real.reserve(obs_detector_real.circles.size());
    for(i=0; i<obs_detector_real.circles.size(); i++){
        obs_pose_real.push_back(Point_convert(obs_detector_real.circles[i].center.x, obs_detector_real.circles[i].center.y));
        // ROS_INFO("obs %d : (%lf,%lf)",i, obs_pose[i].x, obs_pose[i].y);
    }
}
//Get target goal
void getgoal(const geometry_msgs::PoseStamped& goal_pose){
    goal.x = goal_pose.pose.position.x;
    goal.y = goal_pose.pose.position.y;
    G = goal_pose;
}
//Get robot pose for simulation
void getodom_sim(const nav_msgs::Odometry& robot_sim_pose){
    pose_sim.x = robot_sim_pose.pose.pose.position.x;
    pose_sim.y = robot_sim_pose.pose.pose.position.y;
        // ROS_FATAL("pose.sim : (%lf,%lf)", pose.x, pose.y);
        // OS = robot_sim_pose;
}

//Get robot pose for real
void getodom_ekf(const geometry_msgs::PoseWithCovarianceStamped& robot_ekf_pose){
    pose_ekf.x = robot_ekf_pose.pose.pose.position.x;
    pose_ekf.y = robot_ekf_pose.pose.pose.position.y;
    // ROS_FATAL("pose.real : (%lf,%lf)", pose.x, pose.y);
        // OR = robot_real_pose;
}

//Goal_reached?
void goal_reached(const std_msgs::Bool& goal_reached){
    is_goal_reached = goal_reached.data;
}

geometry_msgs::PolygonStamped obs_point;
geometry_msgs::PoseStamped point;
// geometry_msgs::PoseArray camara_obs;
// geometry_msgs::PoseStamped obs_sim;
// obstacle_detector::Obstacles obs_real;

int main(int argc, char** argv){
    ros::init(argc, argv, "path_solver");
    ros::NodeHandle nh;
    ros::NodeHandle simple_nh("move_base_simple");
    ros::Publisher point_pub = nh.advertise<geometry_msgs::PoseStamped>("point",1);
    ros::Subscriber obs_sub_sim = nh.subscribe("obstacle_position_array", 1000, getobs_sim);
    ros::Subscriber obs_sub_real = nh.subscribe("obstacle_array", 1000, getobs_real);
    ros::Subscriber goal_sub = simple_nh.subscribe("goal", 1, getgoal);
    ros::Subscriber pose_sim_sub = nh.subscribe("odom",1,getodom_sim);
    ros::Subscriber pose_ekf_sub = nh.subscribe("ekf_pose",1,getodom_ekf);
    ros::Subscriber reached_sub = nh.subscribe("goal_reached",1,goal_reached);
    ros::Publisher obs_pub_Point = nh.advertise<geometry_msgs::PolygonStamped>("obstacle_position_rviz",1000);

    Line_tan robot_line;
    Line_tan goal_line;
    Line_tan obs_line;
    Step path_solving_process = Step::Checking;

    // Is simulation ?
    bool is_sim = 1;
    nh.param("is_sim_param", is_sim);
    // Is lidar on ?
    bool lidar_on = 0;
    nh.param("is_ekf_param", lidar_on);

    bool new_goal = 0;

    int which_path = 0;  //how many path has been found
    std::vector<int> binary = {0};
    int level = 0;
    std::vector<std::vector<Point>> multipath;
    std::vector<double> cost;
    std::vector<Point> final_path;

    int i_point = 0; //which point is going to be publish
    int node = 1;   //how many point is going to be publish
    std::vector<Point> path_point;
    // double point_w[] = {0.0};
    // double point_z[] = {0.0};

    int i_obs = 0;
    int which_obs = 0;
    int obs_enc = 0;

    Point goal_ed;

    Point last_obs;

    //end point will always be the target goal
    Point begin_point;

    Point intersection_1, intersection_2, intersection_3, intersection_4;
    double path_dis[4];

    bool is_path_clear = 0;
    double begin_border = 0.0;
    double goal_border = 0.0;
    int all_clear = 0;

    double slope_robot_to_goal = 0.0;

    double dis_obs_to_path[2] = {0.0};
    std::vector<double> dis_obs_to_robot;

    bool once = false;

    // bool is_goal_valid = true;
    // int safe_obs = 0;

    std::vector <geometry_msgs::Point32>  rviz_obs;
    geometry_msgs::Point32  point32;
    int N = 16;
    double angle = 0.0;

    double cost_buffer = 0.0;

    while(ros::ok()){
        // ros::Duration(0.5).sleep();

        // Callback
        ros::spinOnce();
        obs_pose.clear();
        obs_pose.reserve(obs_pose_sim.size() + obs_pose_real.size());
        for(int S=0; S<obs_pose_sim.size(); S++)    obs_pose.push_back(obs_pose_sim[S]);
        for(int R=0; R<obs_pose_real.size(); R++)   obs_pose.push_back(obs_pose_real[R]);    
        // for(int M=0; M<obs_pose.size(); M++)    ROS_INFO("obs_position -> (%lf, %lf)", obs_pose[M].x, obs_pose[M].y);
        if(is_sim == true){
            pose = pose_sim;
        }
        else{
            if(lidar_on == true)    pose = pose_ekf;
            else    pose = pose_sim;
        }
        // ROS_INFO("pose -> (%lf, %lf)", pose.x, pose.y);
        //RVIZ visualization
        for(int i=0; i<obs_pose.size(); i++){

            for (int j = 0; j < N; ++j){
                angle = j * 2 * M_PI / N;
                point32.x = obs_pose[i].x+(cos(angle) * (r+stdev_inflation));
                point32.y = obs_pose[i].y+(sin(angle) * (r+stdev_inflation));
                // ROS_FATAL("obs_rviz -> (%lf, %lf)", point32.x, point32.y);
                rviz_obs.push_back(point32);
            }
            // for (int j = 0; j < N; ++j){
            //     angle = j * 2 * M_PI / N;
            //     point32.x = obs_pose[i].x+(cos(angle) * (r));
            //     point32.y = obs_pose[i].y+(sin(angle) * (r));
            //     // ROS_FATAL("obs_rviz -> (%lf, %lf)", point32.x, point32.y);
            //     rviz_obs.push_back(point32);
            // }
            obs_point.header.frame_id = "/robot1/map";
            obs_point.header.stamp = ros::Time::now();
            obs_point.polygon.points = rviz_obs;
            
            obs_pub_Point.publish(obs_point);
            
            rviz_obs.clear();
        }
        
        //Determination of new goal
        if(goal.x != goal_ed.x && goal.y != goal_ed.y){
            // Is the goal valid ?
            for(i_obs=0; i_obs<obs_pose.size(); i_obs++){
                if(goal.x > 3-r || goal.x < 0+r || goal.y > 2-r || goal.y < 0+r){
                    new_goal = 0;
                    ROS_FATAL("|---------------- Unvalid Goal ----------------|");
                    break;
                }
                else if(dis_point_to_point(obs_pose[i_obs], goal) < r){
                    new_goal = 0;
                    ROS_FATAL("|---------------- Unvalid Goal ----------------|");
                    break;
                } 
                else    new_goal = 1;
            }
            //----------------------
            //Initialize the process
            //----------------------                
            path_solving_process = Step::Checking;     
            //path simulation point
            begin_point.x = pose.x;
            begin_point.y = pose.y;
            //path point
            path_point.push_back(pose);
            if(new_goal)    ROS_WARN("|---------------- New goal recieved ----------------|");
            ROS_INFO("Goal -> (%lf, %lf)", goal.x, goal.y);
            //binary tree
            binary.clear();
            binary.push_back(0);
            level = 0;
            //multipath
            which_path = 0;
        }
        //Path solving process -> Step 1 & 2 is going to repeat again and again until the point-to-point path to the target goal is clear
        //If new goal received
        if(new_goal){
            //Step 1: First, we check if the point-to-point path is clear, if true -> keep on finding the next path, if not -> go to Step 2 & node++
            //if all path has been found -> jump out of the cycle & move to step 3
            if(path_solving_process == Step::Checking){
                ROS_FATAL("Checking");
                //Sorting the obstacles according to the distance between the obstacles and robot pose  (near -> far)
                // First, we sort the obstacles by its distance with the begin point      
                dis_obs_to_robot.clear();              
                for(i_obs=0; i_obs<obs_pose.size(); i_obs++)  dis_obs_to_robot.push_back(dis_point_to_point(begin_point, obs_pose[i_obs]));
                sort(dis_obs_to_robot, obs_pose);
                // Then, we culculate the distance of the obstacles and robot path(straight to goal) 
                slope_robot_to_goal = (goal.y - begin_point.y)/(goal.x - begin_point.x);
                for(i_obs=0; i_obs<obs_pose.size(); i_obs++)  dis_obs_to_path[i_obs] = fabs(slope_robot_to_goal*obs_pose[i_obs].x-obs_pose[i_obs].y+(begin_point.y - (slope_robot_to_goal*begin_point.x)))/sqrt(pow(slope_robot_to_goal,2)+1);
                for(i_obs=0; i_obs<obs_pose.size(); i_obs++)  ROS_WARN("distance %d: %lf", i_obs, dis_obs_to_path[i_obs]);
                // Determine is there any obstacles on the way & which one we will first encounter(because we have sort the obstacles)

                // Reset all_clear
                all_clear = 0;

                for(i_obs=0; i_obs<obs_pose.size(); i_obs++){
                    // We shall check from the nearest obstacle
                    begin_border = obs_pose[i_obs].x+slope_robot_to_goal*obs_pose[i_obs].y-(slope_robot_to_goal*begin_point.y+begin_point.x);
                    goal_border = obs_pose[i_obs].x+slope_robot_to_goal*obs_pose[i_obs].y-(slope_robot_to_goal*goal.y+goal.x);
                    if(dis_obs_to_path[i_obs] >= r) is_path_clear = true;
                    else{
                        // Border consideration
                        ROS_INFO("begin_border -> %lf, goal_border -> %lf", begin_border, goal_border);
                        if(goal.x >= pose.x && goal.y >= pose.y){
                            if(begin_border > 0 && goal_border < 0)    is_path_clear = false;
                            else    is_path_clear = true;
                        }
                        else if(goal.x <= pose.x && goal.y > pose.y){
                            if(begin_border > 0 && goal_border < 0)    is_path_clear = false;
                            else    is_path_clear = true;
                        }
                        else if(goal.x < pose.x && goal.y <= pose.y){   
                            if(begin_border < 0 && goal_border > 0)    is_path_clear = false;
                            else    is_path_clear = true;
                        }
                        else if(goal.x > pose.x && goal.y < pose.y){
                            if(begin_border < 0 && goal_border > 0)    is_path_clear = false;
                            else    is_path_clear = true;
                        }
                        else    ROS_WARN("Undefined Circumstances");
                        // is_path_clear = false;
                    }
                    // If the path is clear, it means that the path has been found
                    if(is_path_clear){
                        all_clear++;
                        if(all_clear == obs_pose.size()){
                            // Push the end point of the path(goal point) into path_point
                            path_point.push_back(goal);
                            // Push the path_point into multipath
                            multipath.push_back(path_point);
                            // Reset path_point
                            path_point.clear(); // Clear the vector
                            path_point.push_back(pose); // Initialize the begin point of the path(robot pose)
                            
                            // Reset the begin_point for simulation
                            begin_point.x = pose.x;
                            begin_point.y = pose.y;
                            
                            // Try out other branches
                            binary[level]++;

                            // If the level has been fully searched -> back to the last level
                            ROS_INFO("binary initial[%d] -> %d", level, binary[level]);
                            for(int i = level; i>=0; i--){
                                if(binary[i] > 1){
                                    // ROS_INFO("binary over -> %d", binary[level]);
                                    binary.pop_back();
                                    level--;
                                    // ROS_INFO("binary pop -> %d", binary[level]);
                                    binary[level]++;
                                    // ROS_INFO("binary plus -> %d", binary[level]);
                                }
                                ROS_INFO("binary final[%d] -> %d", level, binary[level]);
                            }
                            
                            // If the tree has been fully searched -> evaluate the path & select the best path
                            // If not -> keep searching
                            if(level == 0 && binary[level] == 1){
                                path_solving_process = Step::Selecting;
                                binary.clear();
                            }
                            
                            which_path++;
                            
                            // Search from the beginning of the tree
                            level = 0;
                            obs_enc = 0;

                            // Reset nodes
                            node = 1;

                            ROS_WARN("|----------------- path %d finished -----------------|", which_path);
                        }   
                    }
                    else{
                        obs_enc++;
                        if(obs_enc >= 2){
                            ROS_WARN("|----------------- Replacing Process Started -----------------|");
                            // ros::Duration(0.3).sleep();
                            //Calculate the tangent line of the two obstacles(which_obs -> old data, i_obs -> new data)
                            obs_line.line_param_insert_cc(last_obs, obs_pose[i_obs], r+stdev_inflation);
                            robot_line.line_param_insert_pc(path_point[path_point.size()-2],last_obs,r+stdev_inflation);
                            //Find the intersection of obs_tan & robot_tan
                            intersection_1 = obs_line.line_intersection_14(robot_line, obs_line, 11, path_point[path_point.size()-1]);   //robot line 1 or 2 -> obs line 1
                            intersection_2 = obs_line.line_intersection_14(robot_line, obs_line, 12, path_point[path_point.size()-1]);   //robot line 1 or 2 -> obs line 2
                            intersection_3 = obs_line.line_intersection_14(robot_line, obs_line, 13, path_point[path_point.size()-1]);   //robot line 1 or 2 -> obs line 3
                            intersection_4 = obs_line.line_intersection_14(robot_line, obs_line, 14, path_point[path_point.size()-1]);   //robot line 1 or 2 -> obs line 4
                            //Correct the path_point with the result above
                            ROS_WARN("last path point -> (%lf, %lf)", path_point[path_point.size()-1].x, path_point[path_point.size()-1].y);
                            path_point.pop_back();
                            ROS_WARN("last path point -> (%lf, %lf)", path_point[path_point.size()-1].x, path_point[path_point.size()-1].y);
                            path_point.push_back(point_select(intersection_1, intersection_2, intersection_3, intersection_4, binary[level-1]));
                            ROS_WARN("last path point -> (%lf, %lf)", path_point[path_point.size()-1].x, path_point[path_point.size()-1].y);
                            //Correct the simlulate begin pointf
                            begin_point = path_point[path_point.size()-1];

                            ROS_INFO("point replace -> (%lf, %lf)", path_point[path_point.size()-1].x, path_point[path_point.size()-1].y);
                            ROS_WARN("|----------------- Replacing Process Finished -----------------|");
                        }

                        last_obs = obs_pose[i_obs];
                        which_obs = i_obs;
                        level++;
                        if(level > binary.size()-1)   binary.push_back(0);
                        // ROS_INFO("%ld", binary.size());
                        path_solving_process = Step::Planning;
                        break;
                    }
                }       
                ROS_FATAL("is_path_clear : %d", is_path_clear);
            }
            //Step 2: Then, we find out the tangent line between the obstacle and our robot/goal, find out the turning point, when it's done -> back to step 1
            if(path_solving_process == Step::Planning){
                ROS_FATAL("Planning");
                ROS_INFO("obstacle detected -> (%lf, %lf)", obs_pose[which_obs].x, obs_pose[which_obs].y);
                // ros::Duration(0.22).sleep();***********************************
                node++;
                
                ROS_INFO("begin point -> (%lf, %lf)", begin_point.x, begin_point.y);

                goal_line.line_param_insert_pc(goal,obs_pose[which_obs],r+stdev_inflation);
                robot_line.line_param_insert_pc(begin_point,last_obs,r+stdev_inflation);

                intersection_1 = robot_line.line_intersection_22(robot_line, goal_line, 11);   //robot line 1 -> goal line 1
                intersection_2 = robot_line.line_intersection_22(robot_line, goal_line, 12);   //robot line 1 -> goal line 2
                intersection_3 = robot_line.line_intersection_22(robot_line, goal_line, 21);   //robot line 2 -> goal line 1
                intersection_4 = robot_line.line_intersection_22(robot_line, goal_line, 22);   //robot line 2 -> goal line 2
                // ROS_FATAL("(1.x,1.y):(%lf,%lf)",intersection_1.x,intersection_1.y);
                // ROS_FATAL("(2.x,2.y):(%lf,%lf)",intersection_2.x,intersection_2.y);
                // ROS_FATAL("(3.x,3.y):(%lf,%lf)",intersection_3.x,intersection_3.y);
                // ROS_FATAL("(4.x,4.y):(%lf,%lf)",intersection_4.x,intersection_4.y);

                //point select
                ROS_INFO("binary[%d] = %d", level, binary[level]);
                if(obs_enc == 1)    path_point.push_back(point_select(intersection_1, intersection_2, intersection_3, intersection_4, binary[level]));
                else if(obs_enc >= 2)   path_point.push_back(obs_line.line_intersection_12(obs_line, goal_line, begin_point));
                ROS_INFO("point raw -> (%lf, %lf)", path_point[path_point.size()-1].x, path_point[path_point.size()-1].y);
                //Simulate begin point
                begin_point = path_point[path_point.size()-1];

                ROS_WARN("obs_enc -> %d", obs_enc);

                path_solving_process = Step::Checking;   
            }
            //Step 3: Select the shortest path
            if(path_solving_process == Step::Selecting){
                ROS_FATAL("Selecting");  
                ROS_WARN("<------ Evaluation started ------>");
                for(int m = 0; m < which_path; m++){
                    cost_buffer = 0.0;
                    for(int s = 1; s < multipath[m].size(); s++){
                        // if((which_path - m) == multipath.size())   cost[m] = 0;
                        cost_buffer += dis_point_to_point(multipath[m][s],multipath[m][s-1]);
                        // ROS_INFO("cost_buffer -> %lf", cost_buffer);
                        // ROS_INFO("dis_point_to_point -> %lf", dis_point_to_point(multipath[m][s],multipath[m][s-1]));         
                        // ROS_INFO("path_points[(%lf,%lf),(%lf,%lf)]", path_point[s].x, path_point[s].y, path_point[s-1].x, path_point[s-1].y);
                        // ROS_WARN("(%lf,%lf) & (%lf,%lf)", path_point[s].x, path_point[s].y, path_point[s-1].x, path_point[s-1].y);
                    }

                    cost.push_back(cost_buffer);
                    // ROS_INFO("distance_cost[%d], nodes_cost[%d] -> %lf, %lf", m, m, cost[m], pow(nodes_cost_param, path_point.size()));
                    cost[m] *= pow(nodes_cost_param, path_point.size());
                    ROS_INFO("path_cost[%d] -> %lf", m, cost[m]);
                    path_point.clear();
                }
                
                // final_path = sort_path(cost, multipath)[0];
                final_path = multipath[0];
                cost.clear();
                multipath.clear();
                ROS_WARN("<------ Evaluation finished ------>");
                path_solving_process = Step::Publishing;
            }
            //Step 4: Finally, we publish the path
            if(path_solving_process == Step::Publishing){
                // ROS_INFO("is goal reached -> %d", is_goal_reached);
                if(is_goal_reached == false){
                    if(once == false){
                        i_point++;    
                        once = true;
                        if(i_point <= final_path.size()-1){
                            point.pose.position.x = final_path[i_point].x;
                            point.pose.position.y = final_path[i_point].y;
                            ROS_INFO("publish point -> (%lf,%lf)", final_path[i_point].x, final_path[i_point].y);
                            if(i_point == final_path.size()-1){
                                point.pose.orientation.w = G.pose.orientation.w;
                                point.pose.orientation.z = G.pose.orientation.z;   
                            }   
                            // ros::Duration(0.3).sleep();*****************************************************************
                            point_pub.publish(point);                        
                        }
                    }
                }
                else{
                    if(once == true)    ROS_FATAL("Point Reached !!");
                    once = false;
                }
                //If all data publish finished, reset the process
                if(i_point > final_path.size()-1){
                    once = false;
                    i_point = 0;
                    new_goal = 0;
                    final_path.clear();
                    ROS_FATAL("Goal Reached !!");
                }    
            }        
        }
        //-----------------------------------
        //              DEBUG
        //-----------------------------------
        // DEBUG(double(i_point), 1);
        // DEBUG(double(node), 2);
        // DEBUG(double(is_goal_reached), 3);
        // DEBUG(double(once), 4);
        // DEBUG(double(goal.x), 5);
        // DEBUG(double(goal.y), 6);
        // DEBUG(double(new_goal), 7);
        // DEBUG(double(path_solving_process), 8);
        // DEBUG(double(point_x[2]), 9);
        // DEBUG(double(point_x[3]), 0);

        goal_ed.x = goal.x;
        goal_ed.y = goal.y;
    }
}

// #include "path_solver.h"

// //DEBUG
// double buffer[10] = {0.0};
// bool print_once[10] = {0};
// //Obstacles position
// double obs_pose_x[] = {0};
// double obs_pose_y[] = {0};
// //How many obstacles?
// int i_obs = 0;
// //Goal reached?
// bool is_goal_reached = 0;
// //Target goal
// Point goal;
// //Robot pose
// Point pose;
// //Convert
// nav_msgs::Odometry O;
// geometry_msgs::PoseStamped G;

// void DEBUG(double data, int i){
//     if(data != buffer[i]) print_once[i] = 0;  
//     buffer[i] = data;        
//     if(print_once[i] == 0){
//         ROS_FATAL("path_solver -> Data[%d] = %lf", i, buffer[i]);
//         print_once[i] = 1;
//     }
// }

// void Line_tan::line_param_insert(Point point_on_line, Point point_outside, double radius){
//     Line_tan::point_on_line = point_on_line;
//     Line_tan::point_outside = point_outside;
//     Line_tan::radius = radius;
//     //Paramerter calculate
//     Line_tan::tan.I = (-m1+sqrt(pow(m1,2)-(4*m2*m0)))/(2*m2);
//     Line_tan::tan.II = (-m1-sqrt(pow(m1,2)-(4*m2*m0)))/(2*m2);
//     ROS_WARN("goal tangent slope : %lf,%lf", Line_tan::tan.I, Line_tan::tan.II);
//     //The parameter of the equation of the two tangent line
//     Line_tan::a1 = Line_tan::tan.I;
//     Line_tan::b1 = -1;
//     Line_tan::c1 = (point_on_line.y-Line_tan::tan.I*point_on_line.x);
//     Line_tan::a2 = Line_tan::tan.II;
//     Line_tan::b2 = -1;
//     Line_tan::c2 = (point_on_line.y-Line_tan::tan.II*point_on_line.x);
//     ROS_WARN("1:(%lf,%lf,%lf), 2:(%lf,%lf,%lf)", a1,b1,c1,a2,b2,c2);
//     //The parameter of the equation which aims to find the slope of tangent line
//     Line_tan::m2 = pow((point_outside.x-point_on_line.x),2)-pow(r,2);
//     Line_tan::m1 = 2*((point_outside.x-point_on_line.x)*(point_on_line.y-point_outside.y));
//     Line_tan::m0 = pow((point_outside.y-point_on_line.y),2)-pow(r,2);
//  }
// Point Line_tan::line_intersection(Line_tan t, Line_tan u, int n){
//     Point intersection;
//     double delta, delta_x, delta_y;
//     if(n == 11){
//         delta = (t.a1*u.b1)-(t.b1*u.a1);
//         delta_x = -(t.c1*u.b1)+(t.b1*u.c1);
//         delta_y = -(t.a1*u.c1)+(t.c1*u.a1);
//     }
//     if(n == 12){
//         delta = (t.a1*u.b2)-(t.b1*u.a2);
//         delta_x = -(t.c1*u.b2)+(t.b1*u.c2);
//         delta_y = -(t.a1*u.c2)+(t.c1*u.a2);
//     }
//     if(n == 21){
//         delta = (t.a2*u.b1)-(t.b2*u.a1);
//         delta_x = -(t.c2*u.b1)+(t.b2*u.c1);
//         delta_y = -(t.a2*u.c1)+(t.c2*u.a1);
//     }
//     if(n == 22){
//         delta = (t.a2*u.b2)-(t.b2*u.a2);
//         delta_x = -(t.c2*u.b2)+(t.b2*u.c2);
//         delta_y = -(t.a2*u.c2)+(t.c2*u.a2);
//     } 
//     intersection.x = delta_x/delta;
//     intersection.y = delta_y/delta;
//     return(intersection);
// }
// //Data convert into Point
// Point Point_convert(double x, double y){
//     Point data;
//     data.x = x;
//     data.y = y;
//     return(data);
// }
// double dis_point_to_point(Point a, Point b){
//     double distance = 0.0;
//     distance = sqrt(pow((a.x-b.x),2)+pow((a.y-b.y),2));
//     return(distance);
// }
// double dis_line_to_point(Point line_a, Point line_b, Point point){
//     double slope = 0.0;
//     double distance = 0.0;
//     slope = (line_b.y - line_a.y)/(line_b.x - line_a.x);
//     distance = fabs(slope*point.x-point.y+(line_a.y - (slope*line_a.x)))/sqrt(pow(slope,2)+1);
//     return(distance);
// }
// //Bubble sort
// void sort(double master[4], int slave[4]){
//     double buffer = 0.0;
//     int tmp = 0;
//     int i = 0;
//     int j = 0;
//     for(i = 4-1; i > 0; i--){
//         for(j = 0; j <= i-1; j++){
//             if( master[j] > master[j+1]){
//                 buffer = master[j];
//                 tmp = slave[j];
//                 master[j] = master[j+1];
//                 slave[j] = slave[j+1];
//                 master[j+1] = buffer;
//                 slave[j+1] = tmp;
//             }
//         }
//     }
// }
// //Path selection
// Point point_select(Point a, Point b, Point c, Point d){
//     Point Point;
//     int point = 0;
//     double path_dis[4] = {0.0};
//     int sequence[4] = {0,1,2,3};
//     bool valid[4] = {false};
//     // double distance[4] = {0.0};
//     //Determine is the point valid
//     if(a.x >= 0.2 && a.x <= 2.8 && a.y >= 0.2 && a.y <= 1.8)  valid[0] = true;
//     if(b.x >= 0.2 && b.x <= 2.8 && b.y >= 0.2 && b.y <= 1.8)  valid[1] = true;
//     if(c.x >= 0.2 && c.x <= 2.8 && c.y >= 0.2 && c.y <= 1.8)  valid[2] = true;    
//     if(d.x >= 0.2 && d.x <= 2.8 && d.y >= 0.2 && d.y <= 1.8)  valid[3] = true;   
//     // distance[0] = dis_line_to_point(a, goal, Point_convert(obs_pose_x[0],obs_pose_y[0]));
//     // distance[1] = dis_line_to_point(b, goal, Point_convert(obs_pose_x[0],obs_pose_y[0]));
//     // distance[2] = dis_line_to_point(c, goal, Point_convert(obs_pose_x[0],obs_pose_y[0]));
//     // distance[3] = dis_line_to_point(d, goal, Point_convert(obs_pose_x[0],obs_pose_y[0]));
//     // if(distance[0] < r) valid[0] = false;
//     // if(distance[1] < r) valid[1] = false;
//     // if(distance[2] < r) valid[2] = false;
//     // if(distance[3] < r) valid[3] = false;
//     //Determine the priority
//     path_dis[0] = dis_point_to_point(pose,a)+dis_point_to_point(goal,a);
//     path_dis[1] = dis_point_to_point(pose,b)+dis_point_to_point(goal,b);
//     path_dis[2] = dis_point_to_point(pose,c)+dis_point_to_point(goal,c);
//     path_dis[3] = dis_point_to_point(pose,d)+dis_point_to_point(goal,d);
//     sort(path_dis,sequence);
//     //Determine the point base on priority & valid
//     if(valid[sequence[0]])  point = sequence[0];
//     else if(valid[sequence[1]]) point = sequence[1];
//     else if(valid[sequence[2]]) point = sequence[2];
//     else if(valid[sequence[3]]) point = sequence[3];
//     else    point = 4;
//     //Convert sequence into point
//     if(point == 0)  Point = a;
//     if(point == 1)  Point = b;
//     if(point == 2)  Point = c;
//     if(point == 3)  Point = d;
//     if(point == 4)  ROS_ERROR("No Valid Point!!!");

//     ROS_INFO("priority : %d,%d,%d,%d", sequence[0],sequence[1],sequence[2],sequence[3]);
//     ROS_INFO("valid : %d,%d,%d,%d",valid[0],valid[1],valid[2],valid[3]);
//     // ROS_INFO("distance : %lf,%lf,%lf,%lf",distance[0],distance[1],distance[2],distance[3]);
//     ROS_INFO("path %d selected!",point+1);
//     ROS_INFO("Point(%lf,%lf)", Point.x, Point.y);

//     return(Point);
// }
// //Get target goal
// void getobs(const geometry_msgs::PoseArray& camara_obs){
//     int i = 0;
//     i_obs = 9;
//     for(i=0; i<9; i++){
//         obs_pose_x[i] = camara_obs.poses[i].position.x;
//         obs_pose_y[i] = camara_obs.poses[i].position.y;
//         ROS_FATAL("obstacla pose[%d] : (%lf,%lf)", i, obs_pose_x[i], obs_pose_y[i]);
//     }
// }
// //Get target goal
// void getgoal(const geometry_msgs::PoseStamped& goal_pose){
//     goal.x = goal_pose.pose.position.x;
//     goal.y = goal_pose.pose.position.y;
//     G = goal_pose;
// }
// //Get robot pose
// void getodom(const nav_msgs::Odometry& robot_pose){
//     pose.x = robot_pose.pose.pose.position.x;
//     pose.y = robot_pose.pose.pose.position.y;
//     O = robot_pose;
// }
// //Goal_reached?
// void goal_reached(const std_msgs::Bool& goal_reached){
//     is_goal_reached = goal_reached.data;
// }

// geometry_msgs::PoseArray camara_obs;
// geometry_msgs::PoseStamped point;
// geometry_msgs::PoseStamped obs;

// int main(int argc, char** argv){
//     ros::init(argc, argv, "path_solver");
//     ros::NodeHandle nh;
//     ros::NodeHandle simple_nh("move_base_simple");
//     ros::Publisher point_pub = nh.advertise<geometry_msgs::PoseStamped>("point",1);
//     ros::Subscriber obs_sub = nh.subscribe("obstacle_position_array", 1000, getobs);
//     ros::Subscriber goal_sub = simple_nh.subscribe("goal", 1, getgoal);
//     ros::Subscriber pose_sub = nh.subscribe("odom",1,getodom);
//     ros::Subscriber reached_sub = nh.subscribe("goal_reached",1,goal_reached);

//     Line_tan robot_line;
//     Line_tan goal_line;
//     Step path_solving_process = Step::Checking;

//     bool new_goal = 0;

//     int i_point = 0; //which point is going to be publish
//     int node = 1;   //how many point is going to be publish
//     double point_x[] = {0.0};
//     double point_y[] = {0.0};
//     // double point_w[] = {0.0};
//     // double point_z[] = {0.0};

//     int which_obs = 0;

//     Point goal_ed;

//     //end point will always be the target goal
//     Point begin_point;

//     Point intersection_1, intersection_2, intersection_3, intersection_4;
//     double path_dis[4];

//     bool is_path_clear = 0;

//     double slope_robot_to_goal = 0.0;

//     double dis_obs_to_path[] = {0.0};

//     bool once = false;

//     while(ros::ok()){
//         //Callback
//         ros::spinOnce();

//         //The equation of the circle which represents the obstacles 
//         //(x-obs_pose_x)^2 + (y-obs_pose_y)^2 = r^2
//         //The equation of the line which represent the direct point-to-point path from robot to target
//         //(y - point_y) = slope*(x - point_x) -> slope*x - y + (point_y - (slope*point_x)) = 0 - a = slope, b = -1, c = (point_y - (slope*point_x)

//         //Determination of new goal
//         if(goal.x != goal_ed.x && goal.y != goal_ed.y){
//             new_goal = 1;
//             //----------------------
//             //Initialize the process
//             //----------------------
//             path_solving_process = Step::Checking;     
//             //path simulation point
//             begin_point.x = pose.x;
//             begin_point.y = pose.y;
//             //path point
//             point_x[0] = pose.x;
//             point_y[0] = pose.y;
//         }
//         //Path solving process -> Step 1 & 2 is going to repeat again and again until the point-to-point path to the target goal is clear
//         //If new goal received
//         if(new_goal){
//             //Step 1: First, we check if the point-to-point path is clear, if it's clear -> jump out of the cycle & move to step 3, if it's not -> node++ & move on to step 2
//             if(path_solving_process == Step::Checking){
//                 slope_robot_to_goal = (goal.y - begin_point.y)/(goal.x - begin_point.x);
//                 dis_obs_to_path[0] = fabs(slope_robot_to_goal*obs_pose_x[0]-obs_pose_y[0]+(begin_point.y - (slope_robot_to_goal*begin_point.x)))/sqrt(pow(slope_robot_to_goal,2)+1);
//                 if(dis_obs_to_path[0] >= r) is_path_clear = true;
//                 else{
//                     if(goal.x >= pose.x && goal.y >= pose.y){
//                         if((obs_pose_x[0] > pose.x && obs_pose_x[0] < goal.x) || (obs_pose_y[0] > pose.y && obs_pose_y[0] < goal.y))    is_path_clear = false;
//                         else    is_path_clear = true;
//                     }
//                     else if(goal.x >= pose.x && goal.y < pose.y){
//                         if((obs_pose_x[0] > pose.x && obs_pose_x[0] < goal.x) || (obs_pose_y[0] < pose.y && obs_pose_y[0] > goal.y))    is_path_clear = false;
//                         else    is_path_clear = true;
//                     }
//                     else if(goal.x < pose.x && goal.y >= pose.y){   
//                         if((obs_pose_x[0] < pose.x && obs_pose_x[0] > goal.x) || (obs_pose_y[0] > pose.y && obs_pose_y[0] < goal.y))    is_path_clear = false;
//                         else    is_path_clear = true;
//                     }
//                     else if(goal.x < pose.x && goal.y < pose.y){
//                         if((obs_pose_x[0] < pose.x && obs_pose_x[0] > goal.x) || (obs_pose_y[0] < pose.y && obs_pose_y[0] > goal.y))    is_path_clear = false;
//                         else    is_path_clear = true;
//                     }
//                 }
//                 if(is_path_clear)   path_solving_process = Step::Publishing;
//                 else    path_solving_process = Step::Planning;
//                 // ROS_FATAL("is_path_clear : %d", is_path_clear);
//                 // ROS_FATAL("Checking");
//             }
//             //Step 2: Then, we find out the tangent line between the obstacle and our robot/goal, find out the turning point, and determine which way is the shorter one, when it's done -> back to step 1
//             if(path_solving_process == Step::Planning){
//                 node++;
//                 goal_line.line_param_insert(goal,Point_convert(obs_pose_x[0],obs_pose_y[0]),r);
//                 robot_line.line_param_insert(pose,Point_convert(obs_pose_x[0],obs_pose_y[0]),r);
//                 goal_line.line_param_insert(goal,Point_convert(obs_pose_x[0],obs_pose_y[0]),r);
//                 robot_line.line_param_insert(pose,Point_convert(obs_pose_x[0],obs_pose_y[0]),r);
//                 intersection_1 = robot_line.line_intersection(robot_line, goal_line, 11);   //robot line 1 -> goal line 1
//                 intersection_2 = robot_line.line_intersection(robot_line, goal_line, 12);   //robot line 1 -> goal line 2
//                 intersection_3 = robot_line.line_intersection(robot_line, goal_line, 21);   //robot line 2 -> goal line 1
//                 intersection_4 = robot_line.line_intersection(robot_line, goal_line, 22);   //robot line 2 -> goal line 2
//                 ROS_FATAL("(1.x,1.y):(%lf,%lf)",intersection_1.x,intersection_1.y);
//                 ROS_FATAL("(2.x,2.y):(%lf,%lf)",intersection_2.x,intersection_2.y);
//                 ROS_FATAL("(3.x,3.y):(%lf,%lf)",intersection_3.x,intersection_3.y);
//                 ROS_FATAL("(4.x,4.y):(%lf,%lf)",intersection_4.x,intersection_4.y);

//                 //point select
//                 point_x[1] = point_select(intersection_1, intersection_2, intersection_3, intersection_4).x;
//                 point_y[1] = point_select(intersection_1, intersection_2, intersection_3, intersection_4).y;

//                 begin_point.x = point_x[1];
//                 begin_point.y = point_y[1];

//                 path_solving_process = Step::Checking;
//                 // ROS_FATAL("Planning");
//             }
//             //Step 3: Finally, we publish the path
//             if(path_solving_process == Step::Publishing){
//                 point_x[node] = goal.x;
//                 point_y[node] = goal.y;
//                 if(is_goal_reached == false){
//                     if(once == false){
//                         i_point++;    
//                         once = true;
//                         if(i_point <= node){
//                             while(point_x[i_point] == 0 && point_y[i_point] == 0)   i_point++;  
//                             point.pose.position.x = point_x[i_point];
//                             point.pose.position.y = point_y[i_point];
//                             point.pose.orientation.w = G.pose.orientation.w;
//                             point.pose.orientation.z = G.pose.orientation.z;          
//                             ros::Duration(0.3).sleep();
//                             point_pub.publish(point);                        
//                         }
//                     }
//                 }
//                 else    once = false;
//                 //If all data publish finished, reset the process
//                 if(i_point > node){
//                     once = false;
//                     i_point = 0;
//                     node = 1;
//                     new_goal = 0;
//                 }    
//             }        
//         }
//         //DEBUG
//         // DEBUG(double(i_point), 1);
//         // DEBUG(double(node), 2);
//         // DEBUG(double(is_goal_reached), 3);
//         // DEBUG(double(once), 4);
//         // DEBUG(double(goal.x), 5);
//         // DEBUG(double(goal.y), 6);
//         // DEBUG(double(new_goal), 7);
//         // DEBUG(double(point_x[1]), 8);
//         // DEBUG(double(point_x[2]), 9);
//         // DEBUG(double(point_x[3]), 0);
//         // ROS_FATAL("is_goal_reached? %d", is_goal_reached);
//         // if(double(i_point) != buffer[1]) print_once[1] = 0;  
//         // buffer[1] = double(i_point);        
//         // if(print_once == 0){
//         //     ROS_WARN("path_solver -> Data = %lf", buffer[1]);
//         //     print_once[1] = 1;
//         // }
//         // if(double(path_solving_process) != buffer[2]) print_once[2] = 0;  
//         // buffer[2] = double(path_solving_process);        
//         // if(print_once == 0){
//         //     ROS_WARN("path_solver -> Data = %lf", buffer[2]);
//         //     print_once[2] = 1;
//         // }

//         goal_ed.x = goal.x;
//         goal_ed.y = goal.y;

//     }
// }