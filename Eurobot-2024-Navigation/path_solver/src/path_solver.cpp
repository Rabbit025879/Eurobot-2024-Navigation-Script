//----------------------------------------------- Header Files  -----------------------------------------------
#include "path_solver.h" 

//----------------------------------------------- Global Variables -----------------------------------------------
// Obstacles position
std::vector<Point>  obs_pose_static;    // Static obstacles from obstacle simulation -> map
std::vector<Point>  obs_pose_pot;       // Static obstacles from sensors -> pot & plant
Point  obs_pose_rival;                  // Dynamic obstacles from sensors -> rival
// Obstacle radius
double obstacle_size = 0.08;
double normal_inflation = 0.03;
double safety_inflation = 0.1;
double clear_radius = obstacle_size+robot_size+normal_inflation;
double avoid_radius = obstacle_size+robot_size+safety_inflation;
// Replan counter
int cnt_replan = 2;
Point Target_goal;
// Begin point
Point begin_point;   // Begin point of the simulation
// Previous path
std::vector<geometry_msgs::PoseStamped> prev_path;

//----------------------------------------------- Custom Functions -----------------------------------------------
// Path Solving Process
std::vector<geometry_msgs::PoseStamped> Path_Solving_Process(Point Begin_Point, Point pose, Point goal, std::vector<Point> obs_pose);
// Merge all types of obstacles
std::vector<Point> Merge_obstacles();
// Debug function
void DEBUG(double data, int i);
// Calculate point to point distance
double dis_point_to_point(Point a, Point b);
// Calculate point to line distance
double dis_line_to_point(Point line_a, Point line_b, Point point);
// Raw data convert into Point
Point Point_convert(double x, double y);
// Bubble sort for array
void sort_seq_with_path_dis(double master[2], double slave[2]);
// Bubble sort for vector
std::vector<Point> sort_obs_with_robot_dis(std::vector<double> master, std::vector<Point> slave);
//Bubble sort for cost
std::vector<std::vector<Point>> sort_path(std::vector<double> cost, std::vector<std::vector<Point>> multipath);
// Intersection filters
Point Intersection_filter_22(Point a, Point b, Point c, Point d, int rank, Point pose, Point goal);
Point Intersection_filter_14(Point a, Point b, Point c, Point d, int rank, Point old_point);

//---------------------------------------------- Subscriber Callback ----------------------------------------------

// Get obstacle position for simulation
void getobs_static(const geometry_msgs::PoseArray& obs_detector_static){
    int i = 0;
    obs_pose_static.clear();
    obs_pose_static.reserve(obs_detector_static.poses.size());
    for(i=0; i<obs_detector_static.poses.size(); i++){
        obs_pose_static.push_back(Point_convert(obs_detector_static.poses[i].position.x, obs_detector_static.poses[i].position.y));
        // ROS_INFO("obs %d : (%lf,%lf)",i, obs_pose[i].x, obs_pose[i].y);
    }
}
// Get static obstacle position for real
void getobs_pot(const obstacle_detector::Obstacles& obs_detector_pot){
    int i = 0;
    obs_pose_pot.clear();
    obs_pose_pot.reserve(obs_detector_pot.circles.size());
    for(i=0; i<obs_detector_pot.circles.size(); i++){
        obs_pose_pot.push_back(Point_convert(obs_detector_pot.circles[i].center.x, obs_detector_pot.circles[i].center.y));
        // ROS_INFO("obs %d : (%lf,%lf)",i, obs_pose[i].x, obs_pose[i].y);
    }
}
// Get rival obstacle position for real
// void getobs_rival(const obstacle_detector::Obstacles& obs_detector_rival){
//     obs_pose_rival.x = obs_detector_rival;
//     obs_pose_rival.y = obs_detector_rival;
// }

//------------------------------------------------- Service Callback ----------------------------------------------
// GetPlan Server
bool Make_plan_server(nav_msgs::GetPlan::Request &request, nav_msgs::GetPlan::Response &responce){
    // Target goal
    Point goal;          // Target goal from service request
    // Robot pose   
    Point pose;          // Start point from service request
    // Obstacle position
    std::vector<Point> obs_pose;
    obs_pose = Merge_obstacles();
    
    ROS_WARN("|------------------------------- Make plan ----------------------------|");

    // Request
    goal.x = request.goal.pose.position.x;
    goal.y = request.goal.pose.position.y;
    pose.x = request.start.pose.position.x;
    pose.y = request.start.pose.position.y;

    // Reset frequency devider & previous path
    if((Target_goal.x != goal.x) && (Target_goal.y != goal.y)){
        cnt_replan = 1;
        // prev_path.clear();
    }
    // Replan count -> Frequency devider
    cnt_replan++;
    if(cnt_replan % 20 == 2){
        begin_point.x = pose.x;
        begin_point.y = pose.y;
    }
    ROS_FATAL("begin_point -> (%lf, %lf)", begin_point.x, begin_point.y);

    // Responce
    responce.plan.header.frame_id = "/robot1/map";
    responce.plan.header.stamp = ros::Time::now();
    responce.plan.poses = Path_Solving_Process(begin_point, pose, goal, Merge_obstacles());

    ROS_FATAL("cnt_replan -> %d", cnt_replan);

    //Record goal
    Target_goal = goal;

    return true;
}
//------------------------------------------------- Main Function ------------------------------------------------
int main(int argc, char** argv){
//---------------------------------------------- ROS Initialization ----------------------------------------------
    ros::init(argc, argv, "path_solver");
    ros::NodeHandle nh;

//----------------------------------------------------- Service ----------------------------------------------------
    ros::ServiceServer server = nh.advertiseService("/make_plan", Make_plan_server);                        // GetPlan server

//----------------------------------------------------- Topics ----------------------------------------------------
    // Obstacles
    ros::Subscriber obs_sub_static = nh.subscribe("obstacle_position_array", 1, getobs_static);              // Static obstacles received
    ros::Subscriber obs_sub_pot = nh.subscribe("obstacle_array", 1, getobs_pot);                             // Pot/Plant position received
    // ros::Subscriber obs_sub_rival = nh.subscribe("obstacle_array", 1000, getobs_rival);                   // Rival position received
    ros::Publisher obs_pub_Rviz = nh.advertise<geometry_msgs::PolygonStamped>("obstacle_position_rviz",1);   // Rviz visualization for obstacles

//----------------------------------------------------- Variables ---------------------------------------------------
    // Rviz visualization for obstacles
    int N = 16;                                         // Transform point to polygon with 16 dots
    double angle = 0.0;                                 // Transform point to polygon with 16 dots
    geometry_msgs::Point32  point32;                    // Point if polygon to push back
    std::vector <geometry_msgs::Point32>  rviz_obs;     // Vector of polygon point to publish 
    std::vector <Point> total_obs;                      // Obstacle buffer to get Merge_obstacles
    geometry_msgs::PolygonStamped obs_point;            // Obstacle output for rviz visualization

//----------------------------------------------------- Loop ---------------------------------------------------
    // Rate
    // ros::Rate rate_1000000hz(1000000);

    while(ros::ok()){
        //RVIZ visualization for obstacles
        total_obs = Merge_obstacles();
        for(int i=0; i<total_obs.size(); i++){
            for (int j = 0; j < N; ++j){
                angle = j * 2 * M_PI / N;
                point32.x = total_obs[i].x+(cos(angle) * obstacle_size);
                point32.y = total_obs[i].y+(sin(angle) * obstacle_size);
                // ROS_FATAL("obs_rviz -> (%lf, %lf)", point32.x, point32.y);
                rviz_obs.push_back(point32);
            }
            for (int j = 0; j < N; ++j){
                angle = j * 2 * M_PI / N;
                point32.x = total_obs[i].x+(cos(angle) * avoid_radius);
                point32.y = total_obs[i].y+(sin(angle) * avoid_radius);
                // ROS_FATAL("obs_rviz -> (%lf, %lf)", point32.x, point32.y);
                rviz_obs.push_back(point32);
            }
            for (int j = 0; j < N; ++j){
                angle = j * 2 * M_PI / N;
                point32.x = total_obs[i].x+(cos(angle) * clear_radius);
                point32.y = total_obs[i].y+(sin(angle) * clear_radius);
                // ROS_FATAL("obs_rviz -> (%lf, %lf)", point32.x, point32.y);
                rviz_obs.push_back(point32);
            }
            obs_point.header.frame_id = "/robot1/map";
            obs_point.header.stamp = ros::Time::now();
            obs_point.polygon.points = rviz_obs;
            
            obs_pub_Rviz.publish(obs_point);
            
            rviz_obs.clear();
        }

        // Callback
        ros::spinOnce();

        // Rate
        // rate_1000000hz.sleep();
    }
}

//---------------------------------------------------------------------------- Functions --------------------------------------------------------------------------------
// Path Solving Process
std::vector<geometry_msgs::PoseStamped> Path_Solving_Process(Point Begin_Point, Point pose, Point goal, std::vector<Point> obs_pose){
//----------------------------------------------------- Lines -----------------------------------------------------
    Line_tan robot_line;    // The tangent line of robot and obstacle
    Line_tan goal_line;     // The tangent line of goal and obstacle    
    Line_tan obs_line;      // The tangent line of obstacles

//------------------------------------------- Path solving process state -------------------------------------------
    Step path_solving_process = Step::Checking; // Checking -> Planning -> Selecting -> Publishing

//----------------------------------------------------- Variables ---------------------------------------------------
    // Entry of the "path solving process"
    Point goal_ed;          // Previous goal  
    bool new_goal = 0;      // Is new goal received ?

    // Paths
    int level = 0;                                                  // Level for binary counter
    std::vector<int> binary = {0};                                  // Binary counter for paths
    std::vector<Point> path_point;                                  // Path buffer to push back
    std::vector<std::vector<Point>> multipath;                      // All path found
    double cost_buffer = 0.0;                                       // Cost buffer to push back 
    std::vector<double> cost;                                       // Cost for each path
    std::vector<Point> final_path;                                  // Final path we select
    geometry_msgs::PoseStamped Final_path_buffer;                   // Path buffer for service respond
    std::vector<geometry_msgs::PoseStamped> Final_path_responce;    // Path output for service respond

    // Obstacles
    int i_obs = 0;                  // Counter of obstacles 
    int which_obs = 0;              // Which obstacles we need to avoid 
    int obs_enc = 0;                // How many obstacles we have encountered 
    Point last_obs;                 // Previous obstacles that we ecountered

    // Path simulation 
    Point begin_point;                                                      // Simulation begin point, end point will always be the target goal
    Point intersection_1, intersection_2, intersection_3, intersection_4;   // Tangent line intersections     
    Point old_point;                                                        // The buffer of the point that pop_back                                  

    // Obstacle detections
    double begin_border = 0.0;              // Begin border to determine which obstacles is inbetween the begin_point and the goal  
    double goal_border = 0.0;               // Goal border to determine which obstacles is inbetween the begin_point and the goal
    bool is_path_clear = 0;                 // Is the path clear ? -> check one by one
    int all_clear = 0;                      // Is the path clear ? -> all obstacles
    double slope_robot_to_goal = 0.0;       // The slope of robot - goal line
    std::vector<double> dis_obs_to_robot;   // The distance between obstacle and robot
    std::vector<double> dis_obs_to_path;    // The distance between obstacle and path

    // Timeout
    int timeout = 0;    // Timeout if the process is stucked !!

//----------------------------------------------------- Loop started ---------------------------------------------------
    // Repeat until process finished
    while(path_solving_process != Step::Finishing){
        // Timeout
        timeout++;
        if(timeout>=50 && timeout<100){
            clear_radius = obstacle_size+robot_size;
            avoid_radius = obstacle_size+robot_size;
            new_goal = 1;
            path_solving_process = Step::Checking;
        }
        else if(timeout>=100){
            path_solving_process = Step::Finishing;
            ROS_FATAL("Path solving timeout -> Process has stopped !!");
        }
        else{
            clear_radius = obstacle_size+robot_size+normal_inflation;
            avoid_radius = obstacle_size+robot_size+safety_inflation;
        }
        // Determination of new goal
        if((goal.x != goal_ed.x && goal.y != goal_ed.y)){
            // Is the goal valid ?
            if(goal.x > 3-robot_size || goal.x < 0+robot_size || goal.y > 2-robot_size || goal.y < 0+robot_size){
                new_goal = 0;
                ROS_ERROR("!---------------- Unvalid Goal ----------------!");
                break;
            }
            else    new_goal = 1;
            if(obs_pose.size()!=0){
                for(i_obs=0; i_obs<obs_pose.size(); i_obs++){
                    if(dis_point_to_point(obs_pose[i_obs], goal) < clear_radius){
                        new_goal = 0;
                        ROS_ERROR("!---------------- Unvalid Goal ----------------!");
                        break;
                    } 
                    else    new_goal = 1;
                }
            }
            // Record the last goal
            goal_ed.x = goal.x;
            goal_ed.y = goal.y;
            // Initialize the process             
            path_solving_process = Step::Checking;     
            // Path simulation point
            begin_point = Begin_Point;
            // Clear vector
            path_point.clear();
            // Path point
            path_point.push_back(pose);
            if(new_goal)    ROS_INFO("!-------------------------- New goal received -------------------------!");
            ROS_INFO("[Path Solver] Start point -> (%lf, %lf)", begin_point.x, begin_point.y);
            ROS_INFO("[Path Solver] Target Goal -> (%lf, %lf)", goal.x, goal.y);
            // Binary tree
            binary.clear();
            binary.push_back(0);
            level = 0;
        }

        //Path solving process -> Step 1 & 2 is going to repeat again and again until the point-to-point path to the target goal is clear & all path have been searched
        //If new goal received
        if(new_goal){
            //if all path has been found -> jump out of the cycle & move to step 3
            if(path_solving_process == Step::Checking){
                // Gaurd -> Stablize the path in case of the robot went into the obstacle inflations by accident 
                for(int i_obs=0; i_obs<obs_pose.size(); i_obs++){
                    if(dis_point_to_point(obs_pose[i_obs], pose) <= safety_inflation){
                        Final_path_responce = prev_path;
                        path_solving_process = Step::Finishing;
                        for(int i;i<100;i++)    ROS_WARN("potential collision may occur !!");
                    }
                }
                if(path_solving_process == Step::Finishing) break;
                //Step 1: First, we check if the point-to-point path is clear, if true -> keep on finding the next path, if not -> go to Step 2 & node++
                ROS_WARN("<---------- Checking ----------->");
                //Sorting the obstacles according to the distance between the obstacles and robot pose  (near -> far)
                // First, we sort the obstacles by its distance with the begin point      
                dis_obs_to_robot.clear();              
                dis_obs_to_robot.reserve(obs_pose.size());
                for(i_obs=0; i_obs<obs_pose.size(); i_obs++)  dis_obs_to_robot.push_back(dis_point_to_point(begin_point, obs_pose[i_obs]));
                obs_pose = sort_obs_with_robot_dis(dis_obs_to_robot, obs_pose);                  
                // Second, we culculate the distance of the obstacles and robot path(straight to goal) 
                dis_obs_to_path.clear();
                dis_obs_to_path.reserve(obs_pose.size());
                for(i_obs=0; i_obs<obs_pose.size(); i_obs++){
                    if(goal.x != begin_point.x) dis_obs_to_path.push_back(dis_line_to_point(goal, begin_point, obs_pose[i_obs]));
                    else dis_obs_to_path.push_back(fabs(obs_pose[i_obs].x - goal.x));
                }

                // Reset all_clear
                all_clear = 0;
                
                // Third, we determine is there any obstacles on the way & which one we will first encounter(because we have already sort the obstacles)
                // Execptions for 0 obstacles -> no need to run path solving process
                if(obs_pose.size() == 0){
                    path_point.push_back(goal);
                    multipath.push_back(path_point);
                    ROS_INFO("No obstacles exist");
                    path_solving_process = Step::Selecting;
                }
                else{
                    for(i_obs=0; i_obs<obs_pose.size(); i_obs++){
                        // We shall check from the nearest obstacle
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
                        if(dis_obs_to_path[i_obs] >= clear_radius) is_path_clear = true;
                        else{
                            // Border consideration
                            if(goal.x >= pose.x && goal.y >= pose.y){
                                if(begin_border > 0 && goal_border < 0)    is_path_clear = false;
                                else    is_path_clear = true;
                            }
                            else if(goal.x <= pose.x && goal.y > pose.y){
                                if(begin_border < 0 && goal_border > 0)    is_path_clear = false;
                                else    is_path_clear = true;
                            }
                            else if(goal.x < pose.x && goal.y <= pose.y){   
                                if(begin_border < 0 && goal_border > 0)    is_path_clear = false;
                                else    is_path_clear = true;
                            }
                            else if(goal.x > pose.x && goal.y < pose.y){
                                if(begin_border > 0 && goal_border < 0)    is_path_clear = false;
                                else    is_path_clear = true;                            
                            }
                            else    ROS_WARN("Undefined Circumstances");
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
                                begin_point = Begin_Point;
                                // Try out other branches
                                binary[level]++;
                                // If the level has been fully searched -> back to the last level
                                for(int i = level; i>=0; i--){
                                    if(binary[i] > 1){
                                        // Binary counter of the path carry
                                        binary.pop_back();  
                                        level--;            
                                        binary[level]++; 
                                    }
                                }
                                // If the tree has been fully searched -> evaluate the path & select the best path
                                // If not -> keep searching
                                if(level == 0 && binary[level] == 1){
                                    path_solving_process = Step::Selecting;
                                    binary.clear();
                                }
                                
                                // Search from the beginning of the tree
                                level = 0;
                                obs_enc = 0;

                                ROS_INFO("|-------------------------- path %ld finished --------------------------|", multipath.size());
                            }   
                        }
                        else{
                            obs_enc++;
                            if(obs_enc >= 2){
                                ROS_INFO("|----------------- Replacing Process Started -----------------|");
                                // Calculate the tangent line of the two obstacles(which_obs -> old data, i_obs -> new data)
                                obs_line.line_param_insert_cc(last_obs, obs_pose[i_obs], avoid_radius);
                                robot_line.line_param_insert_pc(path_point[path_point.size()-2], last_obs, avoid_radius);
                                // Find the intersection of obs_tan & robot_tan
                                intersection_1 = obs_line.line_intersection_14(robot_line, obs_line, 11, path_point.back());   //robot line 1 or 2 -> obs line 1
                                intersection_2 = obs_line.line_intersection_14(robot_line, obs_line, 12, path_point.back());   //robot line 1 or 2 -> obs line 2
                                intersection_3 = obs_line.line_intersection_14(robot_line, obs_line, 13, path_point.back());   //robot line 1 or 2 -> obs line 3
                                intersection_4 = obs_line.line_intersection_14(robot_line, obs_line, 14, path_point.back());   //robot line 1 or 2 -> obs line 4
                                // Correct the path_point with the result above
                                old_point = path_point.back();                                                                            // Record the last path point which is going to be replaced
                                path_point.pop_back();                                                                                                                      // Discard the last point    
                                path_point.push_back(Intersection_filter_14(intersection_1, intersection_2, intersection_3, intersection_4, binary[level-1], old_point));   // Replace the last point
                                // Correct the simlulate begin point
                                begin_point = path_point.back();

                                ROS_INFO("point replace -> (%lf, %lf)", path_point.back().x, path_point.back().y);
                                ROS_INFO("|----------------- Replacing Process Finished -----------------|");
                            }

                            last_obs = obs_pose[i_obs];                         // Record the last obstacle we try to avoid
                            which_obs = i_obs;                                  // Record which obstacle we try to avoid
                            level++;                                            // Obstacle encountered -> update the binary counter 
                            if(level > binary.size()-1)   binary.push_back(0);  // Obstacle encountered -> update the binary counter
                            path_solving_process = Step::Planning;              // Go on to the next process
                            // ROS_INFO("%ld", binary.size());
                            break;
                        }
                    }       
                }
            }
            //Step 2: Then, we find out the tangent line between the obstacle and our robot/goal, find out the turning point, when it's done -> back to step 1
            if(path_solving_process == Step::Planning){
                ROS_WARN("<----------- Planning ----------->");
                ROS_INFO("obstacle detected -> (%lf, %lf)", obs_pose[which_obs].x, obs_pose[which_obs].y);
                ROS_INFO("begin point -> (%lf, %lf)", begin_point.x, begin_point.y);

                goal_line.line_param_insert_pc(goal, obs_pose[which_obs], avoid_radius);
                robot_line.line_param_insert_pc(begin_point, last_obs, avoid_radius);

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
                if(obs_enc == 1)    path_point.push_back(Intersection_filter_22(intersection_1, intersection_2, intersection_3, intersection_4, binary[level], begin_point, goal));
                else if(obs_enc >= 2)   path_point.push_back(obs_line.line_intersection_12(obs_line, goal_line, begin_point, goal));
                ROS_INFO("point raw -> (%lf, %lf)", path_point.back().x, path_point.back().y);
                //Simulate begin point
                begin_point = path_point.back();

                ROS_WARN("obs_enc -> %d", obs_enc);

                path_solving_process = Step::Checking;   
            }
            // Step 3: Select the shortest path
            if(path_solving_process == Step::Selecting){
                ROS_WARN("<---------- Selecting ---------->");
                for(int m = 0; m < multipath.size(); m++){
                    cost_buffer = 0.0;
                    for(int s = 1; s < multipath[m].size(); s++){
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
                path_solving_process = Step::Publishing;
            }
            //Step 4: Finally, we publish the path
            if(path_solving_process == Step::Publishing){
                ROS_WARN("<--------- Publishing ---------->");
                Final_path_responce.clear();
                Final_path_responce.reserve(final_path.size());
                // Undevide path
                // for(int p=0; p<final_path.size(); p++){
                //     Final_path_buffer.header.frame_id = "/robot1/map";              // Path frame_id
                //     Final_path_buffer.header.stamp = ros::Time::now();              // Path stamp
                //     Final_path_buffer.pose.position.x = final_path[p].x; 
                //     Final_path_buffer.pose.position.y = final_path[p].y; 
                //     ROS_INFO("Path -> (%lf, %lf)", Final_path_buffer.pose.position.x, Final_path_buffer.pose.position.y);
                //     Final_path_responce.push_back(Final_path_buffer);
                // }
                // Divide path
                for(int p=0; p<final_path.size()-1; p++){
                    Final_path_buffer.header.frame_id = "/robot1/map";              // Path frame_id
                    Final_path_buffer.header.stamp = ros::Time::now();              // Path stamp
                    for(double c=0.0; c<=60; c++){
                        Final_path_buffer.pose.position.x = final_path[p].x+((final_path[p+1].x-final_path[p].x)*(c/60)); 
                        Final_path_buffer.pose.position.y = final_path[p].y+((final_path[p+1].y-final_path[p].y)*(c/60)); 
                        // ROS_INFO("Path -> (%lf, %lf)", Final_path_buffer.pose.position.x, Final_path_buffer.pose.position.y);
                        Final_path_responce.push_back(Final_path_buffer);
                    }  
                }
                // Prevent path roll back
                for(int i_path=1; i_path<final_path.size(); i_path++){
                    if((fabs(final_path[i_path].x - pose.x) <= 0.08) && (fabs(final_path[i_path].y - pose.y) <= 0.08))    cnt_replan = 0;
                }
                path_solving_process = Step::Finishing;
            }        
        }
    }
    // Reset timeout
    timeout = 0;
    // Record the path
    prev_path = Final_path_responce; 
    // Return Path responce
    return Final_path_responce;
}
// Merge obstacles
std::vector<Point> Merge_obstacles(){
    // Obstacles
    std::vector<Point>  obs_pose;   // The position of Obstacles 
    // Push in obstacles
    obs_pose.clear();                                                                          // Clear the vector first
    obs_pose.reserve(obs_pose_static.size() + obs_pose_pot.size());                            // Reserve memories for the vector
    for(int S=0; S<obs_pose_static.size(); S++)    obs_pose.push_back(obs_pose_static[S]);     // Push back static obstacles from obstacle simulation                                 
    for(int P=0; P<obs_pose_pot.size(); P++)   obs_pose.push_back(obs_pose_pot[P]);            // Push back static obstacles from sensors
    // if(rival_state == Rival_state::Static)  obs_pose.push_back(obs_pose_rival);             // Push back dynamic obstacles from sencors

    return obs_pose;
}
// Raw data convert into Point
Point Point_convert(double x, double y){
    Point data;
    data.x = x;
    data.y = y;
    return(data);
}
// Calculate the distance between point to point
double dis_point_to_point(Point a, Point b){
    double distance = 0.0;
    distance = sqrt(pow((a.x-b.x),2)+pow((a.y-b.y),2));
    return(distance);
}
// Distance between point and line -> slope needs to have value !!
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
// Insert & calculate the parameters of point - circle line
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
// Insert & calculate the parameters of circle - circle line
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
// Find out the intersections of 2 lines & 2 lines
Point Line_tan::line_intersection_22(Line_tan t, Line_tan u, int n){
    Point intersection;
    double delta, delta_x, delta_y;
    // Calculate parameters
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
    // Calculate intersections
    if(delta != 0){
        intersection.x = delta_x/delta;
        intersection.y = delta_y/delta;        
    }
    else{
        ROS_WARN("No intersections !!!");
        intersection.x = 0;
        intersection.y = 0;
    }
    ROS_FATAL("intersections -> (%lf,%lf)", intersection.x, intersection.y);
    return(intersection);
}
// Find out the intersections of 1 line & 4 lines
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
            delta = (t.a1*u.b3)-(t.b1*u.a3);
            delta_x = -(t.c1*u.b3)+(t.b1*u.c3);
            delta_y = -(t.a1*u.c3)+(t.c1*u.a3);
            // delta = 1;
            // delta_x = -1;
            // delta_y = -1;
        }
        if(n == 14){
            delta = (t.a1*u.b4)-(t.b1*u.a4);
            delta_x = -(t.c1*u.b4)+(t.b1*u.c4);
            delta_y = -(t.a1*u.c4)+(t.c1*u.a4);
        //     delta = 1;
        //     delta_x = -1;
        //     delta_y = -1;
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
            delta = (t.a2*u.b3)-(t.b2*u.a3);
            delta_x = -(t.c2*u.b3)+(t.b2*u.c3);
            delta_y = -(t.a2*u.c3)+(t.c2*u.a3);
            // delta = 1;
            // delta_x = -1;
            // delta_y = -1;
        }
        if(n == 14){
            delta = (t.a2*u.b4)-(t.b2*u.a4);
            delta_x = -(t.c2*u.b4)+(t.b2*u.c4);
            delta_y = -(t.a2*u.c4)+(t.c2*u.a4);
            // delta = 1;
            // delta_x = -1;
            // delta_y = -1;
        }
    }
    else    ROS_ERROR("the point is not on the robot_line !!");

    intersection.x = delta_x/delta;
    intersection.y = delta_y/delta;
    ROS_INFO("intersection_cc : (%lf, %lf)", intersection.x, intersection.y);
    // ROS_INFO("<------------------------------------------>");
    return(intersection);
}
// Find out the intersections of 1 line & 2 lines
Point Line_tan::line_intersection_12(Line_tan t, Line_tan u, Point comfirm_point ,Point goal){
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
// Bubble sort for intersection filter
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
// Bubble sort for obstacles with distance between robot & obstacles 
std::vector<Point> sort_obs_with_robot_dis(std::vector<double> master, std::vector<Point> slave){
    double buffer = 0.0;
    Point tmp;

    for(int i = 0; i < master.size()-1; i++){
        for(int j = 0; j < master.size()-1-i; j++){
            if(master[j] > master[j+1]){
                buffer = master[j];
                tmp = slave[j];
                master[j] = master[j+1];
                slave[j] = slave[j+1];
                master[j+1] = buffer;
                slave[j+1] = tmp;
            }
        }
    }

    return(slave);
}
// Bubble sort for cost
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
    ROS_INFO("Smallest cost -> %lf", cost[0]);
    return(multipath);
}
// Intersection filter -> 2 robot line & 2 goal line
Point Intersection_filter_22(Point a, Point b, Point c, Point d, int rank, Point pose, Point goal){
    ROS_WARN("----------------------------------------- Intersection filter -----------------------------------------");
    
    // Variables
    Point point;                                            // Point to return
    double path_dis[4] = {0.0};                             // Distance for the specific path
    int sequence[4] = {0,1,2,3};                            // Sequence of the paths
    bool valid[4] = {false};                                // Determine is the point valid
    double slope_robot_to_goal = 0.0;                       // The slope of the robot - goal line
    double pose_border = 0.0;                               // The border of the pose vertical line 
    double goal_border = 0.0;                               // The border of the goal vertical line
    Point point_buffer;                                     // Point buffer for border filter
    int point_sequence = 0;                                 // The sequence of the point we want
    bool cut = 0;                                           // If we have found the secondary point we want -> jump out of the loop
    // Debug -> Input points
    ROS_INFO("Input -> (%lf,%lf)", a.x, a.y);
    ROS_INFO("Input -> (%lf,%lf)", b.x, b.y);
    ROS_INFO("Input -> (%lf,%lf)", c.x, c.y);
    ROS_INFO("Input -> (%lf,%lf)", d.x, d.y);
    // Determine the priority of the sequence by the distance of robot to point to goal -> The first & second shortest should tell us the desired intersection point
    path_dis[0] = dis_point_to_point(pose,a)+dis_point_to_point(goal,a);
    path_dis[1] = dis_point_to_point(pose,b)+dis_point_to_point(goal,b);
    path_dis[2] = dis_point_to_point(pose,c)+dis_point_to_point(goal,c);
    path_dis[3] = dis_point_to_point(pose,d)+dis_point_to_point(goal,d);
    ROS_INFO("distance(unsort) : %lf,%lf,%lf,%lf",path_dis[0],path_dis[1],path_dis[2],path_dis[3]);
    sort_seq_with_path_dis(path_dis,sequence);
    ROS_INFO("distance(sorted) : %lf,%lf,%lf,%lf",path_dis[0],path_dis[1],path_dis[2],path_dis[3]);
    ROS_INFO("priority : %d,%d,%d,%d", sequence[0],sequence[1],sequence[2],sequence[3]);
    // Determine is the point valid -> Static map boundaries(point that is not on our map should be denied)
    if(a.x >= 0+robot_size && a.x <= 3-robot_size && a.y >= 0+robot_size && a.y <= 2-robot_size)  valid[0] = true;
    if(b.x >= 0+robot_size && b.x <= 3-robot_size && b.y >= 0+robot_size && b.y <= 2-robot_size)  valid[1] = true;
    if(c.x >= 0+robot_size && c.x <= 3-robot_size && c.y >= 0+robot_size && c.y <= 2-robot_size)  valid[2] = true;    
    if(d.x >= 0+robot_size && d.x <= 3-robot_size && d.y >= 0+robot_size && d.y <= 2-robot_size)  valid[3] = true;   
    // Determine is the point valid -> Vertical line boundaries(point that is not inbetween robot & goal should be denied)
    for(int i_point=0; i_point<4; i_point++){
        // Determine which point is going to be verify
        if(i_point == 0)    point_buffer = a;
        if(i_point == 1)    point_buffer = b;
        if(i_point == 2)    point_buffer = c;
        if(i_point == 3)    point_buffer = d;
        // Determine the border
        if(goal.x != pose.x){
            slope_robot_to_goal = (goal.y - pose.y)/(goal.x - pose.x);                                                  // Calculate the slope of the robot - goal line
            pose_border = point_buffer.x+slope_robot_to_goal*point_buffer.y-(slope_robot_to_goal*pose.y+pose.x);        // Calculate the pose border
            goal_border = point_buffer.x+slope_robot_to_goal*point_buffer.y-(slope_robot_to_goal*goal.y+goal.x);        // Calculate the goal border
            ROS_INFO("pose -> (%lf,%lf)", pose.x, pose.y);
            ROS_INFO("goal -> (%lf,%lf)", goal.x, goal.y);
            ROS_INFO("slope -> %lf", slope_robot_to_goal);
            ROS_INFO("Border -> (%lf, %lf)", pose_border, goal_border);
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
        // Border consideration
        if(goal.x >= pose.x && goal.y >= pose.y){
            if(pose_border > 0 && goal_border < 0)    valid[i_point] = true;
            else{    
                valid[i_point] = false; 
                ROS_WARN("Filtered by border 1");
            }
        }
        else if(goal.x <= pose.x && goal.y > pose.y){
            if(pose_border < 0 && goal_border > 0)    valid[i_point] = true;
            else    valid[i_point] = false; ROS_WARN("Filtered by border 2");
        }
        else if(goal.x < pose.x && goal.y <= pose.y){   
            if(pose_border < 0 && goal_border > 0)    valid[i_point] = true;
            else    valid[i_point] = false; ROS_WARN("Filtered by border 3 ");
        }
        else if(goal.x > pose.x && goal.y < pose.y){
            if(pose_border > 0 && goal_border < 0)    valid[i_point] = true;
            else    valid[i_point] = false; ROS_WARN("Filtered by border 4");                            
        }
        else    ROS_WARN("Undefined Circumstances");
    }
    ROS_INFO("valid : %d,%d,%d,%d",valid[0],valid[1],valid[2],valid[3]);
    // Determine the point base on priority & valid -> Shortest path
    if(rank == 0){
        if(valid[sequence[0]])  point_sequence = sequence[0];
        else if(valid[sequence[1]]) point_sequence = sequence[1];
        else if(valid[sequence[2]]) point_sequence = sequence[2];
        else if(valid[sequence[3]]) point_sequence = sequence[3];
        else    point_sequence = 4;
    }
    // Determine the point base on priority & valid -> Second shortest path
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
        else    point_sequence = 4;
    }
    else    ROS_WARN("invalid rank");

    // Convert the sequence into point
    if(point_sequence == 0)  point = a;
    if(point_sequence == 1)  point = b;
    if(point_sequence == 2)  point = c;
    if(point_sequence == 3)  point = d;
    if(point_sequence == 4){
        ROS_WARN("No Valid Point -> Alternative point is going to be select");
        // if(rank == 0)   point = Intersection_filter_22(a, b, c, d, 1, pose, goal);  
        // if(rank == 1)   point = Intersection_filter_22(a, b, c, d, 0, pose, goal);
    }

    ROS_INFO("rank : %d", rank);
    ROS_INFO("Point_select -> (%lf,%lf)", point.x, point.y);

    ROS_WARN("----------------------------------------- Intersection filter -----------------------------------------");
    return(point);
}
// Intersection filter -> 1 robot line & 4 obs line
Point Intersection_filter_14(Point a, Point b, Point c, Point d, int rank, Point old_point){
    ROS_WARN("----------------------------------------- Intersection filter -----------------------------------------");
    
    // Variables
    Point point;                                            // Point to return
    double dis_offset[4] = {0.0};                           // Distance for the specific path
    int sequence[4] = {0,1,2,3};                            // Sequence of the paths
    bool valid[4] = {false};                                // Determine is the point valid
    int point_sequence = 0;                                 // The sequence of the point we want
    bool cut = 0;                                           // If we have found the secondary point we want -> jump out of the loop
    // Debug -> Input points
    ROS_INFO("Input -> (%lf,%lf)", a.x, a.y);
    ROS_INFO("Input -> (%lf,%lf)", b.x, b.y);
    ROS_INFO("Input -> (%lf,%lf)", c.x, c.y);
    ROS_INFO("Input -> (%lf,%lf)", d.x, d.y);
    // Determine the priority of the sequence by the difference of the old point and the new point -> The first & second shortest should tell us the desired intersection point
    dis_offset[0] = dis_point_to_point(old_point,a);
    dis_offset[1] = dis_point_to_point(old_point,b);
    dis_offset[2] = dis_point_to_point(old_point,c);
    dis_offset[3] = dis_point_to_point(old_point,d);
    ROS_INFO("distance(unsort) : %lf,%lf,%lf,%lf",dis_offset[0],dis_offset[1],dis_offset[2],dis_offset[3]);
    sort_seq_with_path_dis(dis_offset,sequence);
    ROS_INFO("distance(sorted) : %lf,%lf,%lf,%lf",dis_offset[0],dis_offset[1],dis_offset[2],dis_offset[3]);
    ROS_INFO("priority : %d,%d,%d,%d", sequence[0],sequence[1],sequence[2],sequence[3]);
    // Determine is the point valid -> Static map boundaries(point that is not on our map should be denied)
    if(a.x >= 0+robot_size && a.x <= 3-robot_size && a.y >= 0+robot_size && a.y <= 2-robot_size)  valid[0] = true;
    if(b.x >= 0+robot_size && b.x <= 3-robot_size && b.y >= 0+robot_size && b.y <= 2-robot_size)  valid[1] = true;
    if(c.x >= 0+robot_size && c.x <= 3-robot_size && c.y >= 0+robot_size && c.y <= 2-robot_size)  valid[2] = true;    
    if(d.x >= 0+robot_size && d.x <= 3-robot_size && d.y >= 0+robot_size && d.y <= 2-robot_size)  valid[3] = true;   
    ROS_INFO("valid : %d,%d,%d,%d",valid[0],valid[1],valid[2],valid[3]);
    // Determine the point base on priority & valid -> Shortest path
    if(rank == 0){
        // Point that is too far from the old point should be denied -> Sequence [2] & [3] 
        if(valid[sequence[0]])  point_sequence = sequence[0];
        else if(valid[sequence[1]]) point_sequence = sequence[1];
        else    point_sequence = 4;
    }
    // Determine the point base on priority & valid -> Second shortest path
    else if(rank == 1){
        // Point that is too far from the old point should be denied -> Sequence [2] & [3] 
        if(valid[sequence[1]])  point_sequence = sequence[1];
        else if(valid[sequence[0]]) point_sequence = sequence[0];
        else    point_sequence = 4;
    }
    else    ROS_WARN("invalid rank");

    // Convert the sequence into point
    if(point_sequence == 0)  point = a;
    if(point_sequence == 1)  point = b;
    if(point_sequence == 2)  point = c;
    if(point_sequence == 3)  point = d;
    if(point_sequence == 4){
        ROS_WARN("No Valid Point !!!");
    }

    ROS_INFO("rank : %d", rank);
    ROS_INFO("Point_select -> (%lf,%lf)", point.x, point.y);

    ROS_WARN("----------------------------------------- Intersection filter -----------------------------------------");
    return(point);
}