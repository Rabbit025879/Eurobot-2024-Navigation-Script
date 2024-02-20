//----------------------------------------------- Header Files  -----------------------------------------------------
#include "path_solver.h" 

//----------------------------------------------- Global Variables --------------------------------------------------
//  *Team Blue or Yellow
std::string team("Blue");
//  *Obstacles position
std::vector<Point>  obs_pose_static;    //  *Static obstacles from obstacle simulation -> map
std::vector<Point>  obs_pose_pot;       //  *Static obstacles from sensors -> pot & plant
Point  obs_pose_rival;                  //  *Dynamic obstacles from sensors -> rival
//  *Obstacle radius
std::vector<double> obs_size_static = {0.08};
std::vector<double> obs_size_pot = {0.03};
double obs_size_rival = 0.14;
//  *Obstacle inflations
double safety_inflation = peace_radius;
//  *Replan counter
int cnt_replan = 2;
Point Target_goal;
//  *Begin point
Point begin_point;   //  *Begin point of the simulation
//  *Previous path
bool valid_guard = true;
std::vector<geometry_msgs::PoseStamped> prev_path;

//----------------------------------------------- Custom Functions --------------------------------------------------
// *Make sure that the point is inside static boundaries
bool static_boundaries_check(Point point);
// *Path Solving Process
std::vector<geometry_msgs::PoseStamped> Path_Solving_Process(Point Begin_Point, Point pose, Point goal, std::vector<Point> obs_pose, std::vector<Point> obs_size);
//  *Merge all types of obstacles
std::vector<Point> Merge_obstacles(int n);
//  *Debug function
void DEBUG(double data, int i);
//  *Calculate point to point distance
double dis_point_to_point(Point a, Point b);
//  *Calculate point to line distance
double dis_line_to_point(Point line_a, Point line_b, Point point);
//  *Raw data convert into Point
Point Point_convert(double x, double y);
//  *Bubble sort for array
void sort_seq_with_path_dis(double master[2], double slave[2]);
//  *Bubble sort for vector
std::vector<Point> sort_obs_with_robot_dis(std::vector<double> dis, std::vector<Point> pose, std::vector<Point> size, int n);
//  *ubble sort for cost
std::vector<std::vector<Point>> sort_path(std::vector<double> cost, std::vector<std::vector<Point>> multipath);
//  *Intersection filters
Point Intersection_filter_22(Point a, Point b, Point c, Point d, int rank, Point pose, Point goal);
Point Intersection_filter_14(Point a, Point b, Point c, Point d, int rank, Point old_point);

//---------------------------------------------- Subscriber Callback ------------------------------------------------

// *Get obstacle position for simulation
void getobs_static(const geometry_msgs::PoseArray& obs_detector_static){
    // double size = 0.25;
    double size = 0.1;
    obs_pose_static.clear();
    obs_pose_static.reserve(obs_detector_static.poses.size());
    obs_size_static.clear();
    obs_size_static.reserve(obs_detector_static.poses.size());
    for(int i_obs = 0; i_obs < obs_detector_static.poses.size(); i_obs++){
        obs_pose_static.push_back(Point_convert(obs_detector_static.poses[i_obs].position.x, obs_detector_static.poses[i_obs].position.y));
        // obs_size_static.push_back(size+normal_inflation_pot);
        obs_size_static[0] = 0.1+normal_inflation_pot;
        obs_size_static[1] = 0.03+normal_inflation_pot;
        // obs_size_static[2] = 0.05+normal_inflation_pot;
        // - ROS_INFO("obs %d : (%lf,%lf)",i, obs_pose[i].x, obs_pose[i].y);
    }
}
//  *Get static obstacle position for real
void getobs_pot(const obstacle_detector::Obstacles& obs_detector_pot){
    obs_pose_pot.clear();
    obs_pose_pot.reserve(obs_detector_pot.circles.size());
    obs_size_pot.clear();
    obs_size_pot.reserve(obs_detector_pot.circles.size());
    for(int i_obs = 0; i_obs < obs_detector_pot.circles.size(); i_obs++){
        obs_pose_pot.push_back(Point_convert(obs_detector_pot.circles[i_obs].center.x, obs_detector_pot.circles[i_obs].center.y));
        obs_size_pot.push_back(pot_size+normal_inflation_plant);
        // - ROS_INFO("obs %d : (%lf,%lf)",i, obs_pose[i].x, obs_pose[i].y);
    }
}
// TODO Get rival obstacle position for real
// void getobs_rival(const obstacle_detector::Obstacles& obs_detector_rival){
//    if(obs_detector_rival.circles.size() == 1){
//         obs_pose_rival.x = obs_detector_rival.circles[0].center.x;
//         obs_pose_rival.y = obs_detector_rival.circles[0].center.y;
//         obs_size_rival = rival_size+normal_inflation_rival;
//     }
//     else    ROS_FATAL("Lidar found obstacles other than rival -> please confirm the error");
// }

//------------------------------------------------ Service Callback -------------------------------------------------
// *GetPlan Server
bool Make_plan_server(nav_msgs::GetPlan::Request &request, nav_msgs::GetPlan::Response &responce){
    // *Target goal
    Point goal;          // *Target goal from service request
    // *Robot pose   
    Point pose;          // *Start point from service request
    // *Obstacle position
    std::vector<Point> obs_pose;
    std::vector<Point> obs_size;
    obs_pose = Merge_obstacles(1);
    obs_size = Merge_obstacles(2);
    // *Divide freq for total replan
    int divide_freq = 3;
    
    ROS_WARN("|------------------------------- Make plan ----------------------------|");

    // *sRequest
    goal.x = request.goal.pose.position.x;
    goal.y = request.goal.pose.position.y;
    pose.x = request.start.pose.position.x;
    pose.y = request.start.pose.position.y;

    // *Reset frequency devider & previous path
    if((Target_goal.x != goal.x) && (Target_goal.y != goal.y)){
        cnt_replan = 1;
        prev_path.clear();
    }
    // *Replan count -> Frequency divider
    cnt_replan++;
    // for(int i_obs = 0; i_obs < obs_pose.size(); i_obs++){
    //     if((dis_point_to_point(obs_pose[i_obs], pose)) < 0.3)   divide_freq = 5;
    //     if((dis_point_to_point(obs_pose[i_obs], pose)) < 0.2)   divide_freq = 3;
    // }
    if(cnt_replan % divide_freq == 2){
        begin_point.x = pose.x;
        begin_point.y = pose.y;
    }
    ROS_INFO("Divide freq -> %d", divide_freq);
    // - ROS_FATAL("begin_point -> (%lf, %lf)", begin_point.x, begin_point.y);

    // *Responce
    responce.plan.header.frame_id = "/robot1/map";
    responce.plan.header.stamp = ros::Time::now();
    responce.plan.poses = Path_Solving_Process(begin_point, pose, goal, obs_pose, obs_size);

    // - ROS_FATAL("cnt_replan -> %d", cnt_replan);

    // *Record goal
    Target_goal = goal;

    return true;
}
//------------------------------------------------ Main Function ----------------------------------------------------
int main(int argc, char** argv){
//---------------------------------------------- ROS Initialization -------------------------------------------------
    ros::init(argc, argv, "path_solver");
    ros::NodeHandle nh;

//----------------------------------------------------- Service -----------------------------------------------------
    ros::ServiceServer server = nh.advertiseService("/make_plan", Make_plan_server);                        // *GetPlan server

//----------------------------------------------------- Topics ------------------------------------------------------
    // *Obstacles
    ros::Subscriber obs_sub_static = nh.subscribe("obstacle_position_array", 1, getobs_static);              // *Static obstacles received
    ros::Subscriber obs_sub_pot = nh.subscribe("obstacle_array", 1, getobs_pot);                             // *Pot/Plant position received
    // TODO rival scan
    // ros::Subscriber obs_sub_rival = nh.subscribe("obstacle_array", 1, getobs_rival);                      // *Rival position received
    ros::Publisher obs_pub_Rviz = nh.advertise<geometry_msgs::PolygonStamped>("obstacle_position_rviz",1);   // *Rviz visualization for obstacles

//----------------------------------------------------- Variables ---------------------------------------------------
    // *Rviz visualization for obstacles
    int N = 16;                                         // *Transform point to polygon with 16 dots
    double angle = 0.0;                                 // *Transform point to polygon with 16 dots
    geometry_msgs::Point32  point32;                    // *Point if polygon to push back
    std::vector <geometry_msgs::Point32>  rviz_obs;     // *Vector of polygon point to publish 
    std::vector <Point> total_obs;                      // *Obstacle buffer to get Merge_obstacles
    std::vector <Point> total_size;                     // *Obstacle buffer to get Merge_obstacles
    geometry_msgs::PolygonStamped obs_point;            // *Obstacle output for rviz visualization

//----------------------------------------------------- Loop --------------------------------------------------------
    while(ros::ok()){
        // *RVIZ visualization for obstacles
        total_obs = Merge_obstacles(1);
        total_size = Merge_obstacles(2);
        for(int i = 0; i < total_obs.size(); i++){
            for (int j = 0; j < N; ++j){
                angle = j * 2 * M_PI / N;
                point32.x = total_obs[i].x+(cos(angle) * total_size[i].x);
                point32.y = total_obs[i].y+(sin(angle) * total_size[i].x);
                // - ROS_FATAL("obs_rviz -> (%lf, %lf)", point32.x, point32.y);
                rviz_obs.push_back(point32);
            }
            for (int j = 0; j < N; ++j){
                angle = j * 2 * M_PI / N;
                point32.x = total_obs[i].x+(cos(angle) * (total_size[i].x+robot_size+safety_inflation));
                point32.y = total_obs[i].y+(sin(angle) * (total_size[i].x+robot_size+safety_inflation));
                // - ROS_FATAL("obs_rviz -> (%lf, %lf)", point32.x, point32.y);
                rviz_obs.push_back(point32);
            }
            for (int j = 0; j < N; ++j){
                angle = j * 2 * M_PI / N;
                point32.x = total_obs[i].x+(cos(angle) * (total_size[i].x+robot_size));
                point32.y = total_obs[i].y+(sin(angle) * (total_size[i].x+robot_size));
                // - ROS_FATAL("obs_rviz -> (%lf, %lf)", point32.x, point32.y);
                rviz_obs.push_back(point32);
            }
            for (int j = 0; j < N; ++j){
                angle = j * 2 * M_PI / N;
                point32.x = total_obs[i].x+(cos(angle) * (total_size[i].x+robot_size+0.015));
                point32.y = total_obs[i].y+(sin(angle) * (total_size[i].x+robot_size+0.015));
                // - ROS_FATAL("obs_rviz -> (%lf, %lf)", point32.x, point32.y);
                rviz_obs.push_back(point32);
            }
            obs_point.header.frame_id = "/robot1/map";
            obs_point.header.stamp = ros::Time::now();
            obs_point.polygon.points = rviz_obs;
            
            obs_pub_Rviz.publish(obs_point);
            
            rviz_obs.clear();
        }

        // *Callback
        ros::spinOnce();
    }
}

//------------------------------------------------ Function practices -----------------------------------------------
// *Make sure that the point is inside static boundaries
bool static_boundaries_check(Point point){
    bool valid = 1;
    
    if(point.x < 0+(robot_size+normal_inflation_wall) || point.x > 3-(robot_size+normal_inflation_wall) || point.y < 0+(robot_size+normal_inflation_wall) || point.y > 2-(robot_size+normal_inflation_wall))  valid = 0;               // *Playground
    if(point.x > 1.05-(robot_size+normal_inflation_wall) && point.x < 1.95+(robot_size+normal_inflation_wall) && point.y > 1.85-(robot_size+normal_inflation_wall) && point.y < 2+(robot_size+normal_inflation_wall))   valid = 0;     // *Bugs' area
    if(team == "Blue"){
        if(point.x > 2.55-(robot_size+normal_inflation_wall) && point.x < 3+(robot_size+normal_inflation_wall) && point.y > 1.55-(robot_size+normal_inflation_wall) && point.y < 2+(robot_size+normal_inflation_wall))  valid = 0;     // *Blue team's restricted area
    }
    if(team == "Yellow"){
        if(point.x > 0-(robot_size+normal_inflation_wall) && point.x < 0.45+(robot_size+normal_inflation_wall) && point.y > 1.55-(robot_size+normal_inflation_wall) && point.y < 2+(robot_size+normal_inflation_wall))  valid = 0;     // *Yellow team's restricted area
    }

    return valid;
}
// *Path Solving Process
std::vector<geometry_msgs::PoseStamped> Path_Solving_Process(Point Begin_Point, Point pose, Point goal, std::vector<Point> obs_pose, std::vector<Point> obs_size){
//----------------------------------------------------- Lines -------------------------------------------------------
    Line_tan robot_line;    // *The tangent line of robot and obstacle
    Line_tan goal_line;     // *The tangent line of goal and obstacle    
    Line_tan obs_line;      // *The tangent line of obstacles

//------------------------------------------ Path solving process state ---------------------------------------------
    Step path_solving_process = Step::Checking; // *Checking <-> Planning -> Selecting -> Publishing

//----------------------------------------------------- Variables ---------------------------------------------------
    // *Entry of the "path solving process"
    Point goal_ed;          // *Previous goal  
    bool new_goal = 0;      // *Is new goal received ?

    // *Paths
    int level = 0;                                                  // *Level for binary counter
    std::vector<int> binary = {0};                                  // *Binary counter for paths
    std::vector<Point> path_point;                                  // *Path buffer to push back
    std::vector<std::vector<Point>> multipath;                      // *All path found
    double cost_buffer = 0.0;                                       // *Cost buffer to push back 
    std::vector<double> cost;                                       // *Cost for each path
    std::vector<Point> final_path;                                  // *Final path we select
    geometry_msgs::PoseStamped Final_path_buffer;                   // *Path buffer for service respond
    std::vector<geometry_msgs::PoseStamped> Final_path_responce;    // *Path output for service respond

    // *Obstacles
    int i_obs = 0;                          // *Counter of obstacles 
    int which_obs = 0;                      // *Which obstacles we need to avoid 
    int obs_enc = 0;                        // *How many obstacles we have encountered 
    Point last_obs_pose;                    // *Previous obstacle that we ecountered
    double last_obs_avoid_radius;           // *Avoid radius of the previous obstacle

    // *Path simulation 
    Point begin_point;                                                      // *Simulation begin point, end point will always be the target goal
    Point intersection_1, intersection_2, intersection_3, intersection_4;   // *Tangent line intersections     
    Point old_point;                                                        // *The buffer of the point that pop_back                                  

    // *Obstacle detections
    double begin_border = 0.0;              // *Begin border to determine which obstacles is inbetween the begin_point and the goal  
    double goal_border = 0.0;               // *Goal border to determine which obstacles is inbetween the begin_point and the goal
    bool is_path_clear = 0;                 // *Is the path clear ? -> check one by one
    int all_clear = 0;                      // *Is the path clear ? -> all obstacles
    double slope_robot_to_goal = 0.0;       // *The slope of robot - goal line
    std::vector<double> dis_obs_to_robot;   // *The distance between obstacle and robot
    std::vector<double> dis_obs_to_path;    // *The distance between obstacle and path

    // *Timeout
    int timeout = 0;            // *Timeout if the process is stucked !!
    bool reset_process = false; // ! Reset the process -> Doesn't work really well
    bool pose_in_obs = false;
    bool goal_in_obs = false;
    bool switch_once = false;

//----------------------------------------------------- Loop started ------------------------------------------------
    // *Repeat until process finished
    while(path_solving_process != Step::Finishing){
        // spinOnce
        ros::spinOnce();
        // *Timeout
        timeout++;
        if(timeout>=100){
            path_solving_process = Step::Finishing;
            ROS_FATAL("Path solving timeout -> Process has stopped !!");
        }
        if(path_solving_process == Step::Finishing) break;
        // *Inflations
        // ? If we can't make a valid plan -> consider more aggressive path(shrink inflations)
        if(goal_in_obs || pose_in_obs){
            safety_inflation = aggressive_radius;
            ROS_WARN("Aggressive path !!");
        }
        else    safety_inflation = peace_radius;
        // *Determination of new goal
        if((goal.x != goal_ed.x && goal.y != goal_ed.y) || reset_process){
            // *Is the goal valid ?
            // *Static boundaries
            if(!static_boundaries_check(goal)){
                path_solving_process = Step::Finishing;
                ROS_ERROR("!---------------- Unvalid Goal -> static boundaries ----------------!");
            }
            else    new_goal = 1;
            // *Obstacles check
            if(obs_pose.size() != 0){
                for(i_obs=0; i_obs<obs_pose.size(); i_obs++){
                    if(dis_point_to_point(obs_pose[i_obs], goal) <= (obs_size[i_obs].x+robot_size+safety_inflation)){
                        if(dis_point_to_point(obs_pose[i_obs], goal) <= (obs_size[i_obs].x+robot_size)){
                            path_solving_process = Step::Finishing;
                            ROS_ERROR("!---------------- Unvalid Goal -> obstacles ----------------!");
                        }
                        else{
                            safety_inflation = aggressive_radius;
                            goal_in_obs = true;
                            ROS_WARN("Potential collision may occur !!");
                        } 
                        break;
                    }
                    else    new_goal = 1;
                }
            }
            if(path_solving_process == Step::Finishing) break;
            // * Reset obs_enc
            obs_enc = 0;
            // *Record the last goal
            goal_ed.x = goal.x;
            goal_ed.y = goal.y;
            // *Initialize the process             
            path_solving_process = Step::Checking;   
            reset_process = false;  
            // *Path simulation point
            begin_point = Begin_Point;
            // *Clear vector
            path_point.clear();
            // *Initialize the Path point
            path_point.push_back(pose);
            if(new_goal)    ROS_INFO("!-------------------------- New goal received -------------------------!");
            ROS_INFO("[Path Solver] Start point -> (%lf, %lf)", begin_point.x, begin_point.y);
            ROS_INFO("[Path Solver] Target Goal -> (%lf, %lf)", goal.x, goal.y);
            // *Binary tree
            binary.clear();
            binary.push_back(0);
            level = 0;
        }

        // *Path solving process -> Step 1 & 2 is going to repeat again and again until the point-to-point path to the target goal is clear & all path have been searched
        // *If new goal received
        if(new_goal){
            // *If all path has been found -> jump out of the cycle & move to step 3
            //------------------------------------------------ Checking ---------------------------------------------
            if(path_solving_process == Step::Checking){
                // *Step 1: First, we check if the point-to-point path is clear, if true -> keep on finding the next path, if not -> go to Step 2 & node++
                ROS_WARN("<---------- Checking ----------->");
                // *Execptions for 0 obstacles -> no need to run path solving process
                if(obs_pose.size() == 0){
                    Final_path_responce.push_back(Final_path_buffer);
                    Final_path_responce.push_back(Final_path_buffer);
                    Final_path_responce[0].header.frame_id = "/robot1/map";              // *Path frame_id
                    Final_path_responce[0].header.stamp = ros::Time::now();              // *Path stamp
                    Final_path_responce[0].pose.position.x = pose.x;
                    Final_path_responce[0].pose.position.y = pose.y;
                    Final_path_responce[1].header.frame_id = "/robot1/map";              // *Path frame_id
                    Final_path_responce[1].header.stamp = ros::Time::now();              // *Path stamp
                    Final_path_responce[1].pose.position.x = goal.x;
                    Final_path_responce[1].pose.position.y = goal.y;
                    ROS_INFO("No obstacles exist");
                    path_solving_process = Step::Finishing;
                }
                else{
                    // ? Guard -> Stablize the path in case of the robot went into the obstacle inflations by accident 
                    for(int i_obs = 0; i_obs < obs_pose.size(); i_obs++){
                        // ! recently changed: pose -> begin_point 
                        if(dis_point_to_point(obs_pose[i_obs], Begin_Point) <= (obs_size[i_obs].x+robot_size)){
                            ROS_FATAL("Emergency stopped !!");
                            ROS_FATAL("Potential crash has happened !!");
                            path_solving_process = Step::Finishing;
                        }
                        else if(dis_point_to_point(obs_pose[i_obs], Begin_Point) <= (obs_size[i_obs].x+robot_size+safety_inflation)){
                            // ** On the move
                            if(!prev_path.empty()){
                                Final_path_responce = prev_path;
                                path_solving_process = Step::Finishing;
                            }
                            // ** Prepare to move
                            else{
                                pose_in_obs = true;
                                safety_inflation = aggressive_radius;
                            }
                            ROS_WARN("Potential collision may occur !!");
                            ROS_WARN("safety_inflations -> %lf", safety_inflation);
                        }
                        else{
                            if( ! goal_in_obs && ! pose_in_obs){
                                pose_in_obs = false;
                                safety_inflation = peace_radius;
                            }
                        }
                        break;
                    }
                }
                if(path_solving_process == Step::Finishing) break;
                // *Sorting the obstacles according to the distance between the obstacles and robot pose  (near -> far)
                // *First, we sort the obstacles by its distance with the begin point      
                dis_obs_to_robot.clear();              
                dis_obs_to_robot.reserve(obs_pose.size());
                for(i_obs = 0; i_obs < obs_pose.size(); i_obs++)  dis_obs_to_robot.push_back(dis_point_to_point(begin_point, obs_pose[i_obs]));
                obs_pose = sort_obs_with_robot_dis(dis_obs_to_robot, obs_pose, obs_size, 1);  
                obs_size = sort_obs_with_robot_dis(dis_obs_to_robot, obs_pose, obs_size, 2);              
                // *Second, we culculate the distance of the obstacles and robot path(straight to goal) 
                dis_obs_to_path.clear();
                dis_obs_to_path.reserve(obs_pose.size());
                for(i_obs=0; i_obs<obs_pose.size(); i_obs++){
                    if(goal.x != begin_point.x) dis_obs_to_path.push_back(dis_line_to_point(goal, begin_point, obs_pose[i_obs]));
                    else dis_obs_to_path.push_back(fabs(obs_pose[i_obs].x - goal.x));
                }

                // *Reset all_clear
                all_clear = 0;
                
                // *Third, we determine is there any obstacles on the way & which one we will first encounter(because we have already sort the obstacles)
                for(i_obs=0; i_obs<obs_pose.size(); i_obs++){
                    // *We shall check from the nearest obstacle
                    if(goal.x != begin_point.x){
                        slope_robot_to_goal = (goal.y - begin_point.y)/(goal.x - begin_point.x);
                        begin_border = obs_pose[i_obs].x+slope_robot_to_goal*obs_pose[i_obs].y-(slope_robot_to_goal*begin_point.y+begin_point.x);
                        goal_border = obs_pose[i_obs].x+slope_robot_to_goal*obs_pose[i_obs].y-(slope_robot_to_goal*goal.y+goal.x);
                    }
                    else{
                        ROS_WARN("border slope is vertical !!");
                        if(obs_pose[i_obs].x > begin_point.x)    begin_border = 1;
                        else if(obs_pose[i_obs].x < begin_point.x)   begin_border = -1;
                        else    begin_border = 0;     
                        if(obs_pose[i_obs].x > goal.x)    goal_border = 1;
                        else if(obs_pose[i_obs].x < goal.x)   goal_border = -1;
                        else    goal_border = 0;   
                    }
                    if(dis_obs_to_path[i_obs] >= (obs_size[i_obs].x+robot_size)) is_path_clear = true;
                    else{
                        // *Border consideration
                        if(goal.x >= begin_point.x && goal.y >= begin_point.y){
                            if(begin_border > 0 && goal_border < 0)    is_path_clear = false;
                            else    is_path_clear = true;
                        }
                        else if(goal.x <= begin_point.x && goal.y > begin_point.y){
                            if(begin_border < 0 && goal_border > 0)    is_path_clear = false;
                            else    is_path_clear = true;
                        }
                        else if(goal.x < begin_point.x && goal.y <= begin_point.y){   
                            if(begin_border < 0 && goal_border > 0)    is_path_clear = false;
                            else    is_path_clear = true;
                        }
                        else if(goal.x > begin_point.x && goal.y < begin_point.y){
                            if(begin_border > 0 && goal_border < 0)    is_path_clear = false;
                            else    is_path_clear = true;                            
                        }
                        else    ROS_WARN("Undefined Circumstances");
                    }
                    // *If the path is clear, it means that the path has been found
                    if(is_path_clear){
                        all_clear++;
                        if(all_clear == obs_pose.size()){
                            // *Push the end point of the path(goal point) into path_point
                            path_point.push_back(goal);
                            // *Push the path_point into multipath
                            multipath.push_back(path_point);
                            // *Reset path_point
                            path_point.clear(); // *Clear the vector
                            path_point.push_back(pose); // *Initialize the begin point of the path(robot pose)
                            // *Reset the begin_point for simulation
                            begin_point = Begin_Point;
                            // *Try out other branches
                            binary.back()++;
                            // *If the level has been fully searched -> back to the last level
                            for(int i = binary.size()-1; i>0; i--){
                                if(binary[i] > 1){
                                    // *Binary counter of the path carry
                                    binary.pop_back();  
                                    binary[i-1]++; 
                                }
                            }
                            // *If the tree has been fully searched -> evaluate the path & select the best path
                            // *If not -> keep searching
                            if(binary[0]){
                                path_solving_process = Step::Selecting;
                                binary.clear();
                            }
                            
                            // *Search from the beginning of the tree
                            level = 0;
                            obs_enc = 0;

                            ROS_INFO("|-------------------------- path %ld finished --------------------------|", multipath.size());
                        }   
                    }
                    else{
                        obs_enc++;
                        level++;                                            // *Obstacle encountered -> update the binary counter 
                        if(level > binary.size()-1)   binary.push_back(0);  // *Obstacle encountered -> update the binary counter
                        ROS_WARN("obs_enc -> %d", obs_enc);
                        // *If more than 2 obstacles -> replace the point with the correct one
                        if(obs_enc >= 2){
                            ROS_INFO("|----------------- Replacing Process Started -----------------|");
                            // *Calculate the tangent line of the two obstacles(which_obs -> old data, i_obs -> new data)
                            ROS_INFO("robot line confirm point -> %lf,%lf", path_point[path_point.size()-2].x,path_point[path_point.size()-2].y);
                            obs_line.line_param_insert_cc(last_obs_pose, obs_pose[i_obs], last_obs_avoid_radius, (obs_size[i_obs].x+robot_size+safety_inflation));
                            if((path_point.size()-2))    robot_line.line_param_insert_pc(path_point[path_point.size()-2], last_obs_pose, last_obs_avoid_radius);
                            else    robot_line.line_param_insert_pc(Begin_Point, last_obs_pose, last_obs_avoid_radius);
                            // *Find the intersection of obs_tan & robot_tan
                            intersection_1 = obs_line.line_intersection_14(robot_line, obs_line, 11, path_point.back());   // *Robot line 1 or 2 -> obs line 1
                            intersection_2 = obs_line.line_intersection_14(robot_line, obs_line, 12, path_point.back());   // *Robot line 1 or 2 -> obs line 2
                            intersection_3 = obs_line.line_intersection_14(robot_line, obs_line, 13, path_point.back());   // *Robot line 1 or 2 -> obs line 3
                            intersection_4 = obs_line.line_intersection_14(robot_line, obs_line, 14, path_point.back());   // *Robot line 1 or 2 -> obs line 4
                            // *Correct the path_point with the result above
                            old_point = path_point.back();                                                                          // *Record the last path point which is going to be replaced
                            path_point.pop_back();                                                                                                                    // *Discard the last point 
                            path_point.push_back(Intersection_filter_14(intersection_1, intersection_2, intersection_3, intersection_4, binary[level], old_point));   // *Replace the last point
                            // *Correct the simlulate begin point
                            begin_point = path_point.back();

                            ROS_INFO("point replace -> (%lf, %lf)", path_point.back().x, path_point.back().y);
                            ROS_INFO("|----------------- Replacing Process Finished -----------------|");
                        }

                        last_obs_pose = obs_pose[i_obs];                                                // *Record the last obstacle we try to avoid
                        last_obs_avoid_radius = (obs_size[i_obs].x+robot_size+safety_inflation);        // *Record the avoid radius of the last obstacle     
                        which_obs = i_obs;                                                              // *Record which obstacle we try to avoid
                        path_solving_process = Step::Planning;                                          // *Go on to the next process
                        // - ROS_INFO("%ld", binary.size());
                        break;
                    }
                }       
            }
            // *Step 2: Then, we find out the tangent line between the obstacle and our robot/goal, find out the turning point, when it's done -> back to step 1
            //------------------------------------------------ Planning ---------------------------------------------
            if(path_solving_process == Step::Planning){
                ROS_WARN("<----------- Planning ----------->");
                ROS_INFO("obstacle detected -> (%lf, %lf)", obs_pose[which_obs].x, obs_pose[which_obs].y);
                ROS_INFO("begin point -> (%lf, %lf)", begin_point.x, begin_point.y);
                // TODO: If the robot is already inside the safety inflation the radius should shrink !!
                goal_line.line_param_insert_pc(goal, obs_pose[which_obs], (obs_size[which_obs].x+robot_size+safety_inflation));
                robot_line.line_param_insert_pc(begin_point, obs_pose[which_obs], (obs_size[which_obs].x+robot_size+safety_inflation));

                intersection_1 = robot_line.line_intersection_22(robot_line, goal_line, 11);   // *Robot line 1 -> Goal line 1
                intersection_2 = robot_line.line_intersection_22(robot_line, goal_line, 12);   // *Robot line 1 -> Goal line 2
                intersection_3 = robot_line.line_intersection_22(robot_line, goal_line, 21);   // *Robot line 2 -> Goal line 1
                intersection_4 = robot_line.line_intersection_22(robot_line, goal_line, 22);   // *Robot line 2 -> Goal line 2
                // - ROS_FATAL("(1.x,1.y):(%lf,%lf)",intersection_1.x,intersection_1.y);
                // - ROS_FATAL("(2.x,2.y):(%lf,%lf)",intersection_2.x,intersection_2.y);
                // - ROS_FATAL("(3.x,3.y):(%lf,%lf)",intersection_3.x,intersection_3.y);
                // - ROS_FATAL("(4.x,4.y):(%lf,%lf)",intersection_4.x,intersection_4.y);

                // *Point select
                // - ROS_INFO("binary[%d] = %d", level, binary[level]);
                if(obs_enc == 1)    path_point.push_back(Intersection_filter_22(intersection_1, intersection_2, intersection_3, intersection_4, binary[level], begin_point, goal));
                else if(obs_enc >= 2)   path_point.push_back(obs_line.line_intersection_12(obs_line, goal_line, begin_point, goal));
                ROS_INFO("point raw -> (%lf, %lf)", path_point.back().x, path_point.back().y);
                // *Simulate begin point
                begin_point = path_point.back();

                path_solving_process = Step::Checking;   
            }
            // *Step 3: Select the shortest path
            //------------------------------------------------ Selecting --------------------------------------------
            if(path_solving_process == Step::Selecting){
                ROS_WARN("<---------- Selecting ---------->");
                for(int m = 0; m < multipath.size(); m++){
                    for(int s = 1; s < multipath[m].size(); s++){
                        // *Cost regarding path's distance
                        if(s == 1)   cost_buffer = dis_point_to_point(multipath[m][s],pose);
                        else    cost_buffer += dis_point_to_point(multipath[m][s],multipath[m][s-1]);
                        // *Cost regarding previous path
                        if((s < multipath[m].size()-1) && !prev_path.empty()){
                            if(multipath[m].size()-1 != (prev_path.size()/divied_path)){
                                // - ROS_INFO("past factor"); 
                                // - ROS_INFO("(%lf, %lf) & (%lf,%lf) -> %lf", multipath[m][s].x, multipath[m][s].y, prev_path[(s+1)*divied_path].pose.position.x, prev_path[(s+1)*divied_path].pose.position.y, dis_point_to_point(multipath[m][s], Point_convert(prev_path[(s+1)*divied_path].pose.position.x, prev_path[(s+1)*divied_path].pose.position.y)));
                                if(dis_point_to_point(multipath[m][s], Point_convert(prev_path[(s+1)*divied_path].pose.position.x, prev_path[(s+1)*divied_path].pose.position.y)) > 0.05)    cost_buffer += 1.2;
                            }
                            else{
                                // - ROS_INFO("past factor"); 
                                // - ROS_INFO("(%lf, %lf) & (%lf,%lf) -> %lf", multipath[m][s].x, multipath[m][s].y, prev_path[s*divied_path].pose.position.x, prev_path[s*divied_path].pose.position.y, dis_point_to_point(multipath[m][s], Point_convert(prev_path[s*divied_path].pose.position.x, prev_path[s*divied_path].pose.position.y)));
                                if(dis_point_to_point(multipath[m][s], Point_convert(prev_path[s*divied_path].pose.position.x, prev_path[s*divied_path].pose.position.y)) > 0.05)    cost_buffer += 1.2;
                            }
                        }
                        // *Avoid path went into obstacles
                        for(int i_obs = 0; i_obs < obs_pose.size(); i_obs++){
                            if(dis_point_to_point(multipath[m][s], obs_pose[i_obs]) <= (obs_size[i_obs].x+robot_size))    cost_buffer += 66;
                        }
                        // *Avoid path went outside the map
                        if(!static_boundaries_check(multipath[m][s]))   cost_buffer += 666;
                        // - ROS_INFO("cost_buffer -> %lf", cost_buffer);
                        // - ROS_INFO("dis_point_to_point -> %lf", dis_point_to_point(multipath[m][s],multipath[m][s-1]));         
                        // - ROS_INFO("path_points[(%lf,%lf),(%lf,%lf)]", path_point[s].x, path_point[s].y, path_point[s-1].x, path_point[s-1].y);
                        // - ROS_WARN("(%lf,%lf) & (%lf,%lf)", path_point[s].x, path_point[s].y, path_point[s-1].x, path_point[s-1].y);
                    }

                    cost.push_back(cost_buffer);
                    ROS_INFO("distance_cost[%d], nodes_cost[%d] -> %lf, %lf", m, m, cost[m], pow(nodes_cost_param, multipath[m].size()-1));
                    // *Cost regarding path's nodes
                    cost[m] *= pow(nodes_cost_param, multipath[m].size()-1);
                    ROS_INFO("path_cost[%d] -> %lf", m, cost[m]);
                }
                for(int m = 0; m < multipath.size(); m++){
                    for(int i_path = 0; i_path < multipath[m].size(); i_path++) ROS_FATAL("path[%d] -> (%lf,%lf)", m, multipath[m][i_path].x, multipath[m][i_path].y);
                }
                final_path = sort_path(cost, multipath)[0];
                for(int i_path = 0; i_path < final_path.size(); i_path++) ROS_FATAL("<--------------------------------  final path -> (%lf,%lf)  -------------------------------->", final_path[i_path].x, final_path[i_path].y);
                cost.clear();
                multipath.clear();
                path_solving_process = Step::Publishing;
            }
            // *Step 4: Finally, we publish the path
            //----------------------------------------------- Publishing --------------------------------------------
            if(path_solving_process == Step::Publishing){
                ROS_WARN("<--------- Publishing ---------->");
                Final_path_responce.clear();
                Final_path_responce.reserve(final_path.size());
                // *Undevide path
                // for(int p=0; p<final_path.size(); p++){
                //     Final_path_buffer.header.frame_id = "/robot1/map";              // Path frame_id
                //     Final_path_buffer.header.stamp = ros::Time::now();              // Path stamp
                //     Final_path_buffer.pose.position.x = final_path[p].x; 
                //     Final_path_buffer.pose.position.y = final_path[p].y; 
                //     ROS_INFO("Path -> (%lf, %lf)", Final_path_buffer.pose.position.x, Final_path_buffer.pose.position.y);
                //     Final_path_responce.push_back(Final_path_buffer);
                // }

                // ? If There are no valid path, we will use the previous path 
                if(!valid_guard){
                    path_solving_process = Step::Finishing;
                    ROS_WARN("No valid path !!");
                }
                else{
                    // *Divide path
                    for(int p=0; p<final_path.size()-1; p++){
                        Final_path_buffer.header.frame_id = "/robot1/map";              // *Path frame_id
                        Final_path_buffer.header.stamp = ros::Time::now();              // *Path stamp
                        for(double c=0.0; c<divied_path; c++){
                            Final_path_buffer.pose.position.x = final_path[p].x+((final_path[p+1].x-final_path[p].x)*(c/divied_path)); 
                            Final_path_buffer.pose.position.y = final_path[p].y+((final_path[p+1].y-final_path[p].y)*(c/divied_path)); 
                            // - ROS_INFO("Path -> (%lf, %lf)", Final_path_buffer.pose.position.x, Final_path_buffer.pose.position.y);
                            Final_path_responce.push_back(Final_path_buffer);
                        }  
                    }
                }
                // *Prevent path roll back
                for(int i_path = 1; i_path < final_path.size(); i_path++){
                    if((fabs(final_path[i_path].x - pose.x) <= 0.08) && (fabs(final_path[i_path].y - pose.y) <= 0.08))    cnt_replan = 0;
                }
                path_solving_process = Step::Finishing;
            }        
        }
    }
    //--------------------------------------------------- Finishing -------------------------------------------------
    ROS_WARN("<--------- Finishing ---------->");
    // *Reset timeout
    timeout = 0;
    // *Record the path
    prev_path = Final_path_responce; 
    // - ROS_FATAL("size %ld", prev_path.size());
    // Return Path responce
    return Final_path_responce;
}
// *Merge obstacles -> 1 for pose, 2 for size
std::vector<Point> Merge_obstacles(int n){
    // *Obstacles
    std::vector<Point>  obs_pose;       // *The pose of obstacles 
    std::vector<double> obs_size;       // *The size of obstacles
    std::vector<Point>  return_value;   // *The final return value for this function
    // *Push in obstacles
    obs_pose.clear();                                                       // *Clear the vector first
    obs_pose.reserve(obs_pose_static.size() + obs_pose_pot.size() + 1);     // *Reserve memories for the vector
    obs_size.clear();                                                       // *Clear the vector first
    obs_size.reserve(obs_pose_static.size() + obs_pose_pot.size() + 1);     // *Reserve memories for the vector
    for(int S = 0; S < obs_pose_static.size(); S++){
        obs_pose.push_back(obs_pose_static[S]);                             // *Push back the pose of static obstacles from obstacle simulation                                 
        obs_size.push_back(obs_size_static[S]);                             // *Push back the size of static obstacles from obstacle simulation
    }
    for(int P = 0; P < obs_pose_pot.size(); P++){
        obs_pose.push_back(obs_pose_pot[P]);                                // *Push back the pose of static obstacles from sensors
        obs_size.push_back(obs_size_pot[P]);                                // *Push back the size of static obstacles from sensors
    }
    // TODO Rival
    // obs_pose.push_back(obs_pose_rival);                                     // Push back the pose of dynamic obstacle from sencors
    // obs_pose.push_back(obs_pose_rival);                                     // Push back the size of dynamic obstacle from sencors

    return_value.clear();
    return_value.reserve(obs_pose.size());
    if(n == 1)  return_value = obs_pose;
    if(n == 2){
        for(int i_size = 0; i_size < obs_size.size(); i_size++) return_value.push_back(Point_convert(obs_size[i_size], obs_size[i_size]));
    }

    return return_value;
}
// *Raw data convert into Point
Point Point_convert(double x, double y){
    Point data;
    data.x = x;
    data.y = y;
    return(data);
}
// *Calculate the distance between point to point
double dis_point_to_point(Point a, Point b){
    double distance = 0.0;
    distance = sqrt(pow((a.x-b.x),2)+pow((a.y-b.y),2));
    return(distance);
}
// *Distance between point and line -> slope needs to have value !!
double dis_line_to_point(Point line_a, Point line_b, Point point){
    double slope = 0.0;
    double distance = 0.0;
    if(line_a.x != line_b.x){
        slope = (line_b.y - line_a.y)/(line_b.x - line_a.x);
        distance = fabs(slope*point.x-point.y+(line_a.y - (slope*line_a.x)))/sqrt(pow(slope,2)+1);
    }
    else    ROS_ERROR("The line is vertical !!");
    return(distance);
}
// *Insert & calculate the parameters of point - circle line
void Line_tan::line_param_insert_pc(Point point_on_line, Point point_outside, double radius){
    Line_tan::point_on_line = point_on_line;
    Line_tan::point_outside = point_outside;
    // *The parameter of the equation which aims to find the slope of tangent line
    Line_tan::m2 = pow((point_outside.x-point_on_line.x),2)-pow(radius,2);
    Line_tan::m1 = 2*((point_outside.x-point_on_line.x)*(point_on_line.y-point_outside.y));
    Line_tan::m0 = pow((point_outside.y-point_on_line.y),2)-pow(radius,2);
    // *Paramerter calculate
    Line_tan::tan.I = (-m1+sqrt(pow(m1,2)-(4*m2*m0)))/(2*m2);
    Line_tan::tan.II = (-m1-sqrt(pow(m1,2)-(4*m2*m0)))/(2*m2);
    // - ROS_WARN("goal tangent slope : %lf,%lf", Line_tan::tan.I, Line_tan::tan.II);
    // TODO: vertical line check
    // TODO: check for c1 abs !!
    // *The parameter of the equation of the two tangent line
    // *ax+by+c = 0
    // if( ! isnan(Line_tan::tan.I)){
    if(!m2){
        Line_tan::a1 = 0;
        Line_tan::b1 = 1;
        Line_tan::c1 = -(point_on_line.y);
        Line_tan::a2 = 0;
        Line_tan::b2 = 1;
        Line_tan::c2 = -(point_on_line.y);
    }    
    else if(!m0){
        Line_tan::a1 = 1;
        Line_tan::b1 = 0;
        Line_tan::c1 = -(point_on_line.x);
        Line_tan::a2 = 1;
        Line_tan::b2 = 0;
        Line_tan::c2 = -(point_on_line.x);
    }
    else{
        Line_tan::a1 = Line_tan::tan.I;
        Line_tan::b1 = -1;
        Line_tan::c1 = point_on_line.y-Line_tan::tan.I*point_on_line.x;
        Line_tan::a2 = Line_tan::tan.II;
        Line_tan::b2 = -1;
        Line_tan::c2 = point_on_line.y-Line_tan::tan.II*point_on_line.x;
    }
    // - ROS_WARN("point_on_line,point_outside -> (%lf,%lf),(%lf,%lf)",point_on_line.x,point_on_line.y,point_outside.x,point_outside.y);
    ROS_INFO("m -> (%lf,%lf,%lf)", m0,m1,m2);
    ROS_INFO("tan -> (%lf,%lf)", tan.I,tan.II);
    // - for(int i=0; i<2; i++)  ROS_FATAL("dis %lf",dis_point_to_point(obs_pose_static[i], point_on_line));
    ROS_INFO("1:(%lf,%lf,%lf), 2:(%lf,%lf,%lf)", a1,b1,c1,a2,b2,c2);
 }
// *Insert & calculate the parameters of circle - circle line
void Line_tan::line_param_insert_cc(Point obs1, Point obs2, double radius1, double radius2){
    // - ROS_INFO("obs1 -> (%lf, %lf), obs2 -> (%lf, %lf)", obs1.x, obs1.y, obs2.x, obs2.y);
    double delta_x = obs1.x-obs2.x;
    double delta_y = obs1.y-obs2.y;   
    double delta_r = radius1-radius2;
    double sigma_r = radius1+radius2; 
    // - ROS_INFO("delta -> (%lf, %lf)", delta_x, delta_y);
    // *The parameter of the equation of the four tangent line
    // *Parameter calculate
    Line_tan::tan.I = ( (delta_x*delta_y) + sqrt( (pow(delta_x,2)*pow(delta_y,2)) - ( (pow(delta_x,2)-pow(delta_r,2)) * (pow(delta_y,2)-pow(delta_r,2)) ) ) ) / (pow(delta_x,2)-pow(delta_r,2));        // *Outer
    Line_tan::tan.II = ( (delta_x*delta_y) - sqrt( (pow(delta_x,2)*pow(delta_y,2)) - ( (pow(delta_x,2)-pow(delta_r,2)) * (pow(delta_y,2)-pow(delta_r,2)) ) ) ) / (pow(delta_x,2)-pow(delta_r,2));       // *Outer
    Line_tan::tan.III = ( (delta_x*delta_y) + sqrt( (pow(delta_x,2)*pow(delta_y,2)) - ( (pow(delta_x,2)-pow(sigma_r,2)) * (pow(delta_y,2)-pow(sigma_r,2)) ) ) ) / (pow(delta_x,2)-pow(sigma_r,2));      // *Cross
    Line_tan::tan.IIII = ( (delta_x*delta_y) - sqrt( (pow(delta_x,2)*pow(delta_y,2)) - ( (pow(delta_x,2)-pow(sigma_r,2)) * (pow(delta_y,2)-pow(sigma_r,2)) ) ) ) / (pow(delta_x,2)-pow(sigma_r,2));     // *Cross
    ROS_INFO("tan.I -> %lf", Line_tan::tan.I);
    ROS_INFO("tan.II -> %lf", Line_tan::tan.II);
    ROS_INFO("tan.III -> %lf", Line_tan::tan.III);
    ROS_INFO("tan.IIII -> %lf", Line_tan::tan.IIII);
    // *ax+by+c = 0
    // *Outer
    // TODO: vertical line check
    // TODO: obs too close -> shrink the radius & the cost function should denied it 
    // if( ! isnan(Line_tan::tan.I)){
        Line_tan::a1 = Line_tan::tan.I;
        Line_tan::b1 = -1;
        Line_tan::c1 = -(sqrt((a1*a1)+1)*radius1)-(a1*obs1.x)+obs1.y;
        // ? Check for abs -> -
        // ROS_WARN("abs check -> %lf",a1*obs1.x-obs1.y+c1);
        // if((a1*obs1.x-obs1.y+c1) > 0)   c1 = (sqrt((a1*a1)+1)*radius1)-(a1*obs1.x)+obs1.y;
        // ROS_WARN("abs check -> %lf",a1*obs1.x-obs1.y+c1);
    // }
    // else{
    //     ROS_WARN("Vertical -> cc, outer");
    //     Line_tan::a1 = 1;
    //     Line_tan::b1 = 0;
    //     if(delta_r){
    //         if((obs1.x > obs2.x) && (radius1 > radius2))    Line_tan::c1 = -(obs1.x-radius1);
    //         if((obs1.x > obs2.x) && (radius1 < radius2))    Line_tan::c1 = -(obs1.x+radius1);
    //         if((obs1.x < obs2.x) && (radius1 > radius2))    Line_tan::c1 = -(obs1.x+radius1);
    //         if((obs1.x < obs2.x) && (radius1 < radius2))    Line_tan::c1 = -(obs1.x-radius1);
    //     }
    //     else    Line_tan::c1 = -(obs1.x+radius1);
    // }
    // if( ! isnan(Line_tan::tan.II)){
        Line_tan::a2 = Line_tan::tan.II;
        Line_tan::b2 = -1;
        Line_tan::c2 = (sqrt((a2*a2)+1)*radius1)-(a2*obs1.x)+obs1.y;
        // ? Check for abs -> +
        // ROS_WARN("abs check -> %lf",a2*obs1.x-obs1.y+c2);
        // if((a2*obs1.x-obs1.y+c2) < 0)   c2 = -(sqrt((a2*a2)+1)*radius1)-(a2*obs1.x)+obs1.y;
        // ROS_WARN("abs check -> %lf",a2*obs1.x-obs1.y+c2);
    // }
    // else{
    //     ROS_WARN("Vertical -> cc, outer");
    //     Line_tan::a2 = 1;
    //     Line_tan::b2 = 0;
    //     if(delta_r){
    //         if((obs1.x > obs2.x) && (radius1 > radius2))    Line_tan::c2 = -(obs1.x-radius1);
    //         if((obs1.x > obs2.x) && (radius1 < radius2))    Line_tan::c2 = -(obs1.x+radius1);
    //         if((obs1.x < obs2.x) && (radius1 > radius2))    Line_tan::c2 = -(obs1.x+radius1);
    //         if((obs1.x < obs2.x) && (radius1 < radius2))    Line_tan::c2 = -(obs1.x-radius1);
    //     }
    //     else    Line_tan::c2 = -(obs1.x-radius1);
    // }
    // *Cross
    // ! tan.III is strange -> (+ or -) ?
    // ! My math sucks -> It is fine for now
    // if( ! isnan(Line_tan::tan.III)){
        Line_tan::a3 = Line_tan::tan.III;
        Line_tan::b3 = -1;
        Line_tan::c3 = (sqrt((a3*a3)+1)*radius1)-(a3*obs1.x)+obs1.y;
        // ? Check for abs -> +
        // ROS_WARN("abs check -> %lf",a3*obs1.x-obs1.y+c3);
        // if((a3*obs1.x-obs1.y+c3) > 0)   c3 = -(sqrt((a3*a3)+1)*radius1)-(a3*obs1.x)+obs1.y;
        // ROS_WARN("abs check -> %lf",a3*obs1.x-obs1.y+c3);
    // }
    // else{
    //     ROS_WARN("Vertical -> cc, cross");
    //     Line_tan::a3 = 1;
    //     Line_tan::b3 = 0;
    //     if(obs1.x >= obs2.x)    Line_tan::c3 = -(obs1.x-radius1);
    //     else    Line_tan::c3 = -(obs1.x+radius1);
    // }
    // if( ! isnan(Line_tan::tan.IIII)){
        Line_tan::a4 = Line_tan::tan.IIII;
        Line_tan::b4 = -1;
        Line_tan::c4 = -(sqrt((a4*a4)+1)*radius1)-(a4*obs1.x)+obs1.y;
        // ? Check for abs -> -
        // ROS_WARN("abs check -> %lf",a4*obs1.x-obs1.y+c4);
        // if((a4*obs1.x-obs1.y+c4) < 0)   c4 = (sqrt((a4*a4)+1)*radius1)-(a4*obs1.x)+obs1.y;
        // ROS_WARN("abs check -> %lf",a4*obs1.x-obs1.y+c4);
    // }
    // else{
    //     ROS_WARN("Vertical -> cc, cross");
    //     Line_tan::a4 = 1;
    //     Line_tan::b4 = 0;
    //     if(obs1.x >= obs2.x)    Line_tan::c3 = -(obs1.x-radius1);
    //     else    Line_tan::c3 = -(obs1.x+radius1);
    // }
    ROS_WARN("(c1, c2, c3, c4) -> (%lf, %lf, %lf, %lf)", c1, c2, c3, c4);
}
// *Find out the intersections of 2 lines & 2 lines
Point Line_tan::line_intersection_22(Line_tan t, Line_tan u, int n){
    Point intersection;
    double delta, delta_x, delta_y;
    // *Calculate parameters
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
    // *Calculate intersections
    if(delta != 0){
        intersection.x = delta_x/delta;
        intersection.y = delta_y/delta;        
    }
    else{
        ROS_WARN("No intersections !!!");
        intersection.x = 0;
        intersection.y = 0;
    }
    // - ROS_FATAL("delta x,y,_ -> (%lf,%lf,%lf)", delta_x, delta_y, delta);
    return(intersection);
}
// *Find out the intersections of 1 line & 4 lines
Point Line_tan::line_intersection_14(Line_tan t, Line_tan u, int n, Point confirm_point){
    // - ROS_INFO("<-------- Intersection Calculating -------->");
    Point intersection;
    double delta, delta_x, delta_y, confirm_1, confirm_2;

    // - ROS_WARN("confirm_point for 2 robot line = (%lf, %lf)", confirm_point.x, confirm_point.y);
    ROS_WARN("intersection_14 -> 1 : (%lf, %lf, %lf)", t.a1, t.b1, t.c1);
    ROS_WARN("intersection_14 -> 2 : (%lf, %lf, %lf)", t.a2, t.b2, t.c2);   
    ROS_WARN("confirm 1 -> %lf", (t.a1*confirm_point.x + t.b1*confirm_point.y + t.c1));
    ROS_WARN("confirm 2 -> %lf", (t.a2*confirm_point.x + t.b2*confirm_point.y + t.c2));
    
    confirm_1 = fabs(t.a1*confirm_point.x + t.b1*confirm_point.y + t.c1);
    confirm_2 = fabs(t.a2*confirm_point.x + t.b2*confirm_point.y + t.c2);

    if(confirm_1 < confirm_2){
        // - ROS_INFO("a1");
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
            delta = (t.a1*u.b3)-(t.b1*u.a3);
            delta_x = -(t.c1*u.b3)+(t.b1*u.c3);
            delta_y = -(t.a1*u.c3)+(t.c1*u.a3);
        }
        if(n == 14){
            delta = (t.a1*u.b4)-(t.b1*u.a4);
            delta_x = -(t.c1*u.b4)+(t.b1*u.c4);
            delta_y = -(t.a1*u.c4)+(t.c1*u.a4);
        }
    }
    else if(confirm_2 < confirm_1){
        // - ROS_INFO("a2");
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
            delta = (t.a2*u.b3)-(t.b2*u.a3);
            delta_x = -(t.c2*u.b3)+(t.b2*u.c3);
            delta_y = -(t.a2*u.c3)+(t.c2*u.a3);
        }
        if(n == 14){
            delta = (t.a2*u.b4)-(t.b2*u.a4);
            delta_x = -(t.c2*u.b4)+(t.b2*u.c4);
            delta_y = -(t.a2*u.c4)+(t.c2*u.a4);
        }
    }
    else    ROS_ERROR("the point is not on the robot_line !!");

    intersection.x = delta_x/delta;
    intersection.y = delta_y/delta;
    ROS_INFO("intersection_cc : (%lf, %lf)", intersection.x, intersection.y);
    // - ROS_INFO("<------------------------------------------>");
    return(intersection);
}
// *Find out the intersections of 1 line & 2 lines 
Point Line_tan::line_intersection_12(Line_tan t, Line_tan u, Point confirm_point ,Point goal){
    Point intersection;
    Point sol_1;
    Point sol_2;
    double dis_1 = 0.0;
    double dis_2 = 0.0;
    double delta, delta_x, delta_y, confirm_1, confirm_2, confirm_3, confirm_4;
    // - ROS_FATAL("confirm point for 4 obs line -> (%lf, %lf)", confirm_point.x, confirm_point.y);
    ROS_WARN("confirm 1 -> %lf", (t.a1*confirm_point.x + t.b1*confirm_point.y + t.c1));
    ROS_WARN("confirm 2 -> %lf", (t.a2*confirm_point.x + t.b2*confirm_point.y + t.c2));
    ROS_WARN("confirm 3 -> %lf", (t.a3*confirm_point.x + t.b3*confirm_point.y + t.c3));
    ROS_WARN("confirm 4 -> %lf", (t.a4*confirm_point.x + t.b4*confirm_point.y + t.c4));

    confirm_1 = fabs(t.a1*confirm_point.x + t.b1*confirm_point.y + t.c1);
    confirm_2 = fabs(t.a2*confirm_point.x + t.b2*confirm_point.y + t.c2);
    confirm_3 = fabs(t.a3*confirm_point.x + t.b3*confirm_point.y + t.c3);
    confirm_4 = fabs(t.a4*confirm_point.x + t.b4*confirm_point.y + t.c4);

    if((confirm_1 < confirm_2) && (confirm_1 < confirm_3) && (confirm_1 < confirm_4)){
        // - ROS_INFO("a1");
        // *solution 1
        delta = (t.a1*u.b1)-(t.b1*u.a1);
        delta_x = -(t.c1*u.b1)+(t.b1*u.c1);
        delta_y = -(t.a1*u.c1)+(t.c1*u.a1);
        sol_1.x = delta_x/delta;
        sol_1.y = delta_y/delta;
        // *solution 2
        delta = (t.a1*u.b2)-(t.b1*u.a2);
        delta_x = -(t.c1*u.b2)+(t.b1*u.c2);
        delta_y = -(t.a1*u.c2)+(t.c1*u.a2);
        sol_2.x = delta_x/delta;
        sol_2.y = delta_y/delta;
    }
    else if((confirm_2 < confirm_1) && (confirm_2 < confirm_3) && (confirm_2 < confirm_4)){
        // - ROS_INFO("a2");
        // *solution 1
        delta = (t.a2*u.b1)-(t.b2*u.a1);
        delta_x = -(t.c2*u.b1)+(t.b2*u.c1);
        delta_y = -(t.a2*u.c1)+(t.c2*u.a1);
        sol_1.x = delta_x/delta;
        sol_1.y = delta_y/delta;
        // *solution 2
        delta = (t.a2*u.b2)-(t.b2*u.a2);
        delta_x = -(t.c2*u.b2)+(t.b2*u.c2);
        delta_y = -(t.a2*u.c2)+(t.c2*u.a2);
        sol_2.x = delta_x/delta;
        sol_2.y = delta_y/delta;
    }
    else if((confirm_3 < confirm_1) && (confirm_3 < confirm_2) && (confirm_3 < confirm_4)){
        // - ROS_INFO("a3");
        // *solution 1
        delta = (t.a3*u.b1)-(t.b3*u.a1);
        delta_x = -(t.c3*u.b1)+(t.b3*u.c1);
        delta_y = -(t.a3*u.c1)+(t.c3*u.a1);
        sol_1.x = delta_x/delta;
        sol_1.y = delta_y/delta;
        // *solution 2
        delta = (t.a3*u.b2)-(t.b3*u.a2);
        delta_x = -(t.c3*u.b2)+(t.b3*u.c2);
        delta_y = -(t.a3*u.c2)+(t.c3*u.a2);
        sol_2.x = delta_x/delta;
        sol_2.y = delta_y/delta;
    }
    else if((confirm_4 < confirm_1) && (confirm_4 < confirm_2) && (confirm_4 < confirm_3)){
        // - ROS_INFO("a4");
        // *solution 1
        delta = (t.a4*u.b1)-(t.b4*u.a1);
        delta_x = -(t.c4*u.b1)+(t.b4*u.c1);
        delta_y = -(t.a4*u.c1)+(t.c4*u.a1);
        sol_1.x = delta_x/delta;
        sol_1.y = delta_y/delta;
        // *solution 2
        delta = (t.a4*u.b2)-(t.b4*u.a2);
        delta_x = -(t.c4*u.b2)+(t.b4*u.c2);
        delta_y = -(t.a4*u.c2)+(t.c4*u.a2);
        sol_2.x = delta_x/delta;
        sol_2.y = delta_y/delta;
    }
    else    ROS_ERROR("the point is not on the obs_line !!");
    
    dis_1 = dis_point_to_point(confirm_point, sol_1) + dis_point_to_point(sol_1, goal);
    dis_2 = dis_point_to_point(confirm_point, sol_2) + dis_point_to_point(sol_2, goal);

    if(dis_1 <= dis_2)  intersection = sol_1;
    else if(dis_2 < dis_1)  intersection = sol_2;

    ROS_INFO("intersection_12 : (%lf, %lf)", intersection.x, intersection.y);
    return(intersection);
}
// *Bubble sort for intersection filter
void sort_seq_with_path_dis(double path_dis[4], int sequence[4]){
    double buffer = 0.0;
    int tmp = 0;

    for(int i = 0; i < 4-1; i++){
        for(int j = 0; j < 4-1-i; j++){
            if(path_dis[j] > path_dis[j+1]){
                buffer = path_dis[j];
                tmp = sequence[j];
                path_dis[j] = path_dis[j+1];
                sequence[j] = sequence[j+1];
                path_dis[j+1] = buffer;
                sequence[j+1] = tmp;
            }
        }
    }
}
// *Bubble sort for the pose & size of obstacles with distance between robot & obstacles -> 1 for pose, 2 for size
std::vector<Point> sort_obs_with_robot_dis(std::vector<double> dis, std::vector<Point> pose, std::vector<Point> size, int n){
    double buffer = 0.0;
    Point tmp;
    Point sub;
    std::vector<Point>  return_value;

    for(int i = 0; i < dis.size()-1; i++){
        for(int j = 0; j < dis.size()-1-i; j++){
            if(dis[j] > dis[j+1]){
                buffer = dis[j];
                tmp = pose[j];
                sub = size[j];
                dis[j] = dis[j+1];
                pose[j] = pose[j+1];
                size[j] = size[j+1];
                dis[j+1] = buffer;
                pose[j+1] = tmp;
                size[j+1] = sub;
            }
        }
    }

    if(n == 1)  return_value = pose;
    if(n == 2)  return_value = size;

    // - for(int i = 0; i < return_value.size(); i++)    ROS_WARN("pose/size -> (%lf,%lf)", return_value[i].x, return_value[i].y);

    return(return_value);
}
// *Bubble sort for cost
std::vector<std::vector<Point>> sort_path(std::vector<double> cost, std::vector<std::vector<Point>> multipath){
    double buffer = 0.0;
    std::vector<Point> tmp;

    for(int i = 0; i < cost.size()-1; i++){
        for(int j = 0; j < cost.size()-1-i; j++){
            if(cost[j] > cost[j+1]){
                buffer = cost[j];
                tmp = multipath[j];
                cost[j] = cost[j+1];
                multipath[j] = multipath[j+1];
                cost[j+1] = buffer;
                multipath[j+1] = tmp;
            }
        }
    }

    // ? If the smallest cost >= 66, it means that there are no path valid
    if(cost[0] >= 66)   valid_guard = false;
    else    valid_guard = true;
    ROS_INFO("Smallest cost -> %lf", cost[0]);
    return(multipath);
}
// *Intersection filter -> 2 robot line & 2 goal line
Point Intersection_filter_22(Point a, Point b, Point c, Point d, int rank, Point pose, Point goal){
    ROS_WARN("----------------------------------------- Intersection filter -----------------------------------------");
    
    // *Variables
    Point point;                                            // *Point to return
    double path_dis[4] = {0.0};                             // *Distance for the specific path
    int sequence[4] = {0,1,2,3};                            // *Sequence of the paths
    bool valid[4] = {false};                                // *Determine is the point valid
    double slope_robot_to_goal = 0.0;                       // *The slope of the robot - goal line
    double pose_border = 0.0;                               // *The border of the pose vertical line 
    double goal_border = 0.0;                               // *The border of the goal vertical line
    Point point_buffer;                                     // *Point buffer for border filter
    int point_sequence = 0;                                 // *The sequence of the point we want
    bool cut = 0;                                           // *If we have found the secondary point we want -> jump out of the loop
    // *Debug -> Input points
    // - ROS_INFO("Input -> (%lf,%lf)", a.x, a.y);
    // - ROS_INFO("Input -> (%lf,%lf)", b.x, b.y);
    // - ROS_INFO("Input -> (%lf,%lf)", c.x, c.y);
    // - ROS_INFO("Input -> (%lf,%lf)", d.x, d.y);
    // *Determine the priority of the sequence by the distance of robot to point to goal -> The first & second shortest should tell us the desired intersection point
    path_dis[0] = dis_point_to_point(pose,a)+dis_point_to_point(goal,a);
    path_dis[1] = dis_point_to_point(pose,b)+dis_point_to_point(goal,b);
    path_dis[2] = dis_point_to_point(pose,c)+dis_point_to_point(goal,c);
    path_dis[3] = dis_point_to_point(pose,d)+dis_point_to_point(goal,d);
    // - ROS_INFO("distance(unsort) : %lf,%lf,%lf,%lf",path_dis[0],path_dis[1],path_dis[2],path_dis[3]);
    sort_seq_with_path_dis(path_dis,sequence);
    // - ROS_INFO("distance(sorted) : %lf,%lf,%lf,%lf",path_dis[0],path_dis[1],path_dis[2],path_dis[3]);
    // - ROS_INFO("priority : %d,%d,%d,%d", sequence[0],sequence[1],sequence[2],sequence[3]);  
    // *Determine is the point valid -> Vertical line boundaries(point that is not inbetween robot & goal should be denied)
    for(int i_point=0; i_point<4; i_point++){
        // *Determine which point is going to be verify
        if(i_point == 0)    point_buffer = a;
        if(i_point == 1)    point_buffer = b;
        if(i_point == 2)    point_buffer = c;
        if(i_point == 3)    point_buffer = d;
        // *Determine the border
        if(goal.x != pose.x){
            slope_robot_to_goal = (goal.y - pose.y)/(goal.x - pose.x);                                                  // *Calculate the slope of the robot - goal line
            pose_border = point_buffer.x+slope_robot_to_goal*point_buffer.y-(slope_robot_to_goal*pose.y+pose.x);        // *Calculate the pose border
            goal_border = point_buffer.x+slope_robot_to_goal*point_buffer.y-(slope_robot_to_goal*goal.y+goal.x);        // *Calculate the goal border
            // - ROS_INFO("pose -> (%lf,%lf)", pose.x, pose.y);
            // - ROS_INFO("goal -> (%lf,%lf)", goal.x, goal.y);
            // - ROS_INFO("slope -> %lf", slope_robot_to_goal);
            // - ROS_INFO("Border -> (%lf, %lf)", pose_border, goal_border);
        }
        else{
            ROS_WARN("border slope is vertical !!");
            if(point_buffer.x > pose.x)    pose_border = 1;
            else if(point_buffer.x < pose.x)   pose_border = -1;
            else    pose_border = 0;     
            if(point_buffer.x > goal.x)    goal_border = 1;
            else if(point_buffer.x < goal.x)   goal_border = -1;
            else    goal_border = 0;   
        }
        // *Border consideration
        if(goal.x >= pose.x && goal.y >= pose.y){
            if(pose_border > 0 && goal_border < 0)    valid[i_point] = true;
            else    valid[i_point] = false; 
        }
        else if(goal.x <= pose.x && goal.y > pose.y){
            if(pose_border < 0 && goal_border > 0)    valid[i_point] = true;
            else    valid[i_point] = false;
        }
        else if(goal.x < pose.x && goal.y <= pose.y){   
            if(pose_border < 0 && goal_border > 0)    valid[i_point] = true;
            else    valid[i_point] = false;
        }
        else if(goal.x > pose.x && goal.y < pose.y){
            if(pose_border > 0 && goal_border < 0)    valid[i_point] = true;
            else    valid[i_point] = false;                           
        }
        else    ROS_WARN("Undefined Circumstances");
    }
    // - ROS_INFO("valid : %d,%d,%d,%d",valid[0],valid[1],valid[2],valid[3]);
    // *Determine the point base on priority & valid -> Shortest path
    if(rank == 0){
        if(valid[sequence[0]])  point_sequence = sequence[0];
        else if(valid[sequence[1]]) point_sequence = sequence[1];
        else if(valid[sequence[2]]) point_sequence = sequence[2];
        else if(valid[sequence[3]]) point_sequence = sequence[3];
    }
    // *Determine the point base on priority & valid -> Second shortest path
    else if(rank == 1){
        for(int k=0; k<4; k++){
            if(cut == 1)    break;
            if(valid[sequence[k]]){
                for(int g=k+1; g<4; g++){
                    if(valid[sequence[g]]){
                        point_sequence = sequence[g];
                        cut = 1;
                        break;
                    }   
                }
            }
        }
        if(cut == 1)    cut = 0;
    }
    else    ROS_WARN("invalid rank");

    // *Convert the sequence into point
    if(point_sequence == 0)  point = a;
    if(point_sequence == 1)  point = b;
    if(point_sequence == 2)  point = c;
    if(point_sequence == 3)  point = d;


    // - ROS_INFO("rank : %d", rank);
    // - ROS_INFO("Point_select -> (%lf,%lf)", point.x, point.y);

    ROS_WARN("----------------------------------------- Intersection filter -----------------------------------------");
    return(point);
}
// *Intersection filter -> 1 robot line & 4 obs line
Point Intersection_filter_14(Point a, Point b, Point c, Point d, int rank, Point old_point){
    ROS_WARN("----------------------------------------- Intersection filter -----------------------------------------");
    
    // *Variables
    Point point;                                            // *Point to return
    double dis_offset[4] = {0.0};                           // *Distance for the specific path
    int sequence[4] = {0,1,2,3};                            // *Sequence of the paths
    int point_sequence = 0;                                 // *The sequence of the point we want
    bool cut = 0;                                           // *If we have found the secondary point we want -> jump out of the loop
    // *Debug -> Input points
    // - ROS_INFO("Input -> (%lf,%lf)", a.x, a.y);
    // - ROS_INFO("Input -> (%lf,%lf)", b.x, b.y);
    // - ROS_INFO("Input -> (%lf,%lf)", c.x, c.y);
    // - ROS_INFO("Input -> (%lf,%lf)", d.x, d.y);
    // *Determine the priority of the sequence by the difference of the old point and the new point -> The first & second shortest should tell us the desired intersection point
    dis_offset[0] = dis_point_to_point(old_point,a);
    dis_offset[1] = dis_point_to_point(old_point,b);
    dis_offset[2] = dis_point_to_point(old_point,c);
    dis_offset[3] = dis_point_to_point(old_point,d);
    // - ROS_INFO("distance(unsort) : %lf,%lf,%lf,%lf",dis_offset[0],dis_offset[1],dis_offset[2],dis_offset[3]);
    sort_seq_with_path_dis(dis_offset,sequence);
    // - ROS_INFO("distance(sorted) : %lf,%lf,%lf,%lf",dis_offset[0],dis_offset[1],dis_offset[2],dis_offset[3]);
    // - ROS_INFO("priority : %d,%d,%d,%d", sequence[0],sequence[1],sequence[2],sequence[3]);
    // - ROS_INFO("valid : %d,%d,%d,%d",valid[0],valid[1],valid[2],valid[3]);
    // *Determine the point base on priority & valid -> Shortest path & Second shortest path
    // *Point that is too far from the old point should be denied -> Sequence [2] & [3]
    if(rank == 0)   point_sequence = sequence[0];
    else if(rank == 1)  point_sequence = sequence[1];
    else    ROS_WARN("invalid rank");

    // *Convert the sequence into point
    if(point_sequence == 0)  point = a;
    if(point_sequence == 1)  point = b;
    if(point_sequence == 2)  point = c;
    if(point_sequence == 3)  point = d;

    // - ROS_INFO("rank : %d", rank);
    // - ROS_INFO("Point_select -> (%lf,%lf)", point.x, point.y);

    ROS_WARN("----------------------------------------- Intersection filter -----------------------------------------");
    return(point);
}