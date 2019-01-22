#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"
#include "json.hpp"
#include "spline.h"
#include "helper_functions.hpp"
#include "Constants.hpp"

using namespace std;
using json = nlohmann::json;

/*
 
 #############################################
 #####                                  ######
 ##### START OF LOCAL HELPER FUNCTIONS  ######
 #####                                  ######
 #############################################
 
 */

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.find_first_of("}");
    if (found_null != string::npos) {
        return "";
    } else if (b1 != string::npos && b2 != string::npos) {
        return s.substr(b1, b2 - b1 + 2);
    }
    return "";
}

// Euclidean distance
double distance(double x1, double y1, double x2, double y2)
{
    return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

// Waypoints are pre-defined landmarks that exist within a map, such as mapx_x and maps_y
// x & y are the current position in global coordinates
int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
{
    
    double closestLen = 100000; //large number
    int closestWaypoint = 0;
    
    for(int i = 0; i < maps_x.size(); i++)
    {
        double map_x = maps_x[i];
        double map_y = maps_y[i];
        double dist = distance(x,y,map_x,map_y);
        if(dist < closestLen)
        {
            closestLen = dist;
            closestWaypoint = i;
        }
        
    }
    
    return closestWaypoint;
    
}

// get the next map landmark
int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{
    
    int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);
    
    double map_x = maps_x[closestWaypoint];
    double map_y = maps_y[closestWaypoint];
    
    double heading = atan2( (map_y-y),(map_x-x) );
    
    double angle = abs(theta-heading);
    
    if(angle > pi()/4)
    {
        closestWaypoint++;
    }
    
    return closestWaypoint;
    
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{
    int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);
    
    int prev_wp;
    prev_wp = next_wp-1;
    if(next_wp == 0)
    {
        prev_wp  = maps_x.size()-1;
    }
    
    double n_x = maps_x[next_wp]-maps_x[prev_wp];
    double n_y = maps_y[next_wp]-maps_y[prev_wp];
    double x_x = x - maps_x[prev_wp];
    double x_y = y - maps_y[prev_wp];
    
    // find the projection of x onto n
    double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
    double proj_x = proj_norm*n_x;
    double proj_y = proj_norm*n_y;
    
    double frenet_d = distance(x_x,x_y,proj_x,proj_y);
    
    //see if d value is positive or negative by comparing it to a center point
    
    double center_x = 1000-maps_x[prev_wp];
    double center_y = 2000-maps_y[prev_wp];
    double centerToPos = distance(center_x,center_y,x_x,x_y);
    double centerToRef = distance(center_x,center_y,proj_x,proj_y);
    
    if(centerToPos <= centerToRef)
    {
        frenet_d *= -1;
    }
    
    // calculate s value
    double frenet_s = 0;
    for(int i = 0; i < prev_wp; i++)
    {
        frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
    }
    
    frenet_s += distance(0,0,proj_x,proj_y);
    
    return {frenet_s,frenet_d};
    
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
{
    int prev_wp = -1;
    
    while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
    {
        prev_wp++;
    }
    
    int wp2 = (prev_wp+1)%maps_x.size();
    
    double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
    // the x,y,s along the segment
    double seg_s = (s-maps_s[prev_wp]);
    
    double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
    double seg_y = maps_y[prev_wp]+seg_s*sin(heading);
    
    double perp_heading = heading-pi()/2;
    
    double x = seg_x + d*cos(perp_heading);
    double y = seg_y + d*sin(perp_heading);
    
    return {x,y};
    
}

/*
 
 #############################################
 #####                                  ######
 ##### END   OF LOCAL HELPER FUNCTIONS  ######
 #####                                  ######
 #############################################

 
 */

// hold car speed from the prior simulator update
double prior_car_speed = 0.0;


int main(int argc, const char * argv[]) {
    
    // this program uses Web Sockets to communicate with the highway simulator
    uWS::Hub h;
    
    // Load up map values for waypoint's x,y,s and d normalized normal vectors
    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_s;
    vector<double> map_waypoints_dx;
    vector<double> map_waypoints_dy;
    
    // Waypoint map file to read from command line
    std::string map_file_;
    
    // check if command line has correct number of arguments
    if (2 > argc)
    {
        std::cout << "Error: no file name provided\n";
        return EXIT_FAILURE;
    }
    else
    {
        map_file_ = (argv[1]);
    }

    // creat file object. Initialize with map file
    ifstream in_map_(map_file_.c_str(), ifstream::in);
    
    // ensure file is open
    if (!in_map_)
    {
        std::cout << "Error: could not open file: " << argv[1] << std::endl;
        return EXIT_FAILURE;
    }
    
    // read in data from map file.
    // populate respective containers.
    string line;
    while (getline(in_map_, line)) {
        istringstream iss(line);
        double x;
        double y;
        float s;
        float d_x;
        float d_y;
        iss >> x;
        iss >> y;
        iss >> s;
        iss >> d_x;
        iss >> d_y;
        map_waypoints_x.push_back(x);
        map_waypoints_y.push_back(y);
        map_waypoints_s.push_back(s);
        map_waypoints_dx.push_back(d_x);
        map_waypoints_dy.push_back(d_y);
    }
    
    // Web Socket iterface with simulator
    h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                                                                                                         uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        if (length && length > 2 && data[0] == '4' && data[1] == '2') {
            
            auto s = hasData(data);
            
            // if 's' is not empty, parse it. it contains all the information from the simulator.
            if (s != "") {
                
                auto j = json::parse(s);
                
                string event = j[0].get<string>();
                
                if (event == "telemetry") {
                    
                    // j[1] is the data JSON object
                    
                    // Main car's localization Data
                    double car_x = j[1]["x"];
                    double car_y = j[1]["y"];
                    double car_s = j[1]["s"];
                    double car_d = j[1]["d"];
                    double car_yaw = j[1]["yaw"];
                    double car_speed = j[1]["speed"];
                    
                    // Previous path data given to the Planner.
                    // The simulator does not execute the entire trajectory within a cycle,
                    // so these are the remaining points in the trajectory
                    auto previous_path_x = j[1]["previous_path_x"];
                    auto previous_path_y = j[1]["previous_path_y"];
                    
                    // Previous path's end s and d values
                    double end_path_s = j[1]["end_path_s"];
                    
                    // Sensor Fusion Data, a list of all other cars on the same side of the road.
                    auto sensor_fusion = j[1]["sensor_fusion"];
                    
                    // the message this program will send to the simulator
                    json msgJson;
                    
                    // containers to hold the trajectory that will be given to the simulator for execution.
                    vector<double> next_x_vals;
                    vector<double> next_y_vals;
                    
                    // define container to hold points that are used to generate spline
                    vector<double> ptsx;
                    vector<double> ptsy;
                    
                    
                    // define number of points left in the previous path
                    int prev_size = previous_path_x.size();
                    
                    // booleans that signals when the car must stay in its lane or change langes
                    bool keep_lane = false;
                    bool change_lane = false;
                    bool edge_lane_signaled = false;
                    
                    // will hold JMT coefficients when a lane change is signaled
                    Trajectory_Coeffs trajectory;
                    
                    // define variables to hold reference state of car
                    double ref_x = car_x;
                    double ref_y = car_y;
                    double ref_yaw = deg2rad(car_yaw);
                    double fsm_car_s = car_s;
                    
                    // set car's s to prior s if not starting out from scratch.
                    // useful for path generation
                    if (0 != prev_size)
                        car_s = end_path_s;
                    
                    /*
                        
                     #####################################
                     #####                          ######
                     #####    EVALUATE FSM_FAR()    ######
                     #####                          ######
                     #####################################
                     
                     
                     */
                    
                    // If i am not in the process of changing lanes & am in the edge lanes, evaluate FSM_FAR()
                    if ( (1 != LANE) && (  0 == LANE || 2 == LANE) &&  ((std::abs((2 + 4 * LANE) - car_d) < 0.5)) )
                    {
                        
                        // define the car's current state
                        State current_state;
                        current_state.car_s = fsm_car_s;
                        current_state.car_d = car_d;
                        current_state.car_speed = car_speed * 0.447;    // to meters/s
                        
                        // initialize and set current state vector
                        State_Vectors current_state_vector;
                        current_state_vector.s = { fsm_car_s, car_speed * 0.447, ((car_speed * 0.447) - (prior_car_speed * 0.447)) / 0.02 };
                        current_state_vector.d = { 2 + 4 * double(LANE), 0.0, 0.0 };
                        
                        // call into FSM_FAR() to obtain decision
                        std::string fsm_outcome = FSM_Far(current_state, sensor_fusion);
                        
                        if ("far_right" == fsm_outcome)
                        {
                            // FSM_Far() decided that far right lane has greater clearing. Move to midle lane.
                            
                            // Call into MTG(), which will return a vector of coefficients of a jerk-minimizing trajectory
                            
                            // this is where I want to be in 4 seconds.
                            State_Vectors goal_state_vector;
                            
                            // current_state_vector[1] + 3.0 is taking current vel and finding distance after X seconds
                            goal_state_vector.s.push_back(current_state_vector.s[0] + current_state_vector.s[1] * 4.0);
                            goal_state_vector.s.push_back(current_state_vector.s[1]);
                            goal_state_vector.s.push_back(0);
                            goal_state_vector.d.push_back(current_state_vector.d[0] + 4);
                            goal_state_vector.d.push_back(0);
                            goal_state_vector.d.push_back(0);
                            
                            // obtain Jerk Minimizing Trajectory to goal state.
                            // timeframe for lane change = 4.0 seconds.
                            // If MTG cannot return a trajectory, no action is taken
                            if (MTG(current_state_vector, goal_state_vector, trajectory, 4.0))
                            {
                                
                                // JMT coefficients have been created that satisfy cost functions.
                                // this trajectory is stored in 'trajectory'
                                
                                // set booleans, which control how trajectory generation proceeds.
                                edge_lane_signaled = true;
                                change_lane = true;
                                
                                // update lane
                                LANE++;
                            }
                            else
                            {
                                std::cout << "main() -- trajectory generation failed. will keep lane\n";
                            }
                        }
                        else if ("far_left" == fsm_outcome)
                        {
                            // same process as "far_right" above, except goal 'd' is -4 from current LANE (move to middle lane)
                            
                            State_Vectors goal_state_vector;
                            goal_state_vector.s.push_back(current_state_vector.s[0] + current_state_vector.s[1] * 4.0);
                            goal_state_vector.s.push_back(current_state_vector.s[1]);
                            goal_state_vector.s.push_back(0);
                            goal_state_vector.d.push_back(current_state_vector.d[0] - 4);
                            goal_state_vector.d.push_back(0);
                            goal_state_vector.d.push_back(0);
                            
                            if (MTG(current_state_vector, goal_state_vector, trajectory, 4.0))
                            {
                                edge_lane_signaled = true;
                                change_lane = true;
                                LANE--;
                            }
                            else
                            {
                                std::cout << "main() -- trajectory generation failed. will keep lane\n";
                            }
                        }
                        else if ("keep_lane" == fsm_outcome)
                        {
                            // do nothing
                        }
                        else
                        {
                            std::cout << "FSM_Far returned error\n";
                            // do nothing
                        }
                    }
                    
                    /*
                     
                     #########################################
                     #####                              ######
                     #####    END EVALUATE FSM_FAR()    ######
                     #####                              ######
                     #########################################
                     
                     */
                    
                    
                    
                    /*
                     
                     ############################################################
                     #####                                                 ######
                     #####  EVALUATE COLLISION DETECTION AND FSM_NEAR()    ######
                     #####                                                 ######
                     ############################################################
                     
                     */
                    
                    // If a decision to change lanes was already made by FSM_Far(), bypass collision detection and FSM_near() evaluation
                    if (!edge_lane_signaled)
                    {
                    
                        // Check if any car is too close in front of me, which would warranty a lane change
                        
                        // iterate through sensor_fusion vector, which contains all cars on the highway and their
                        // current states.
                        for (int i = 0; i < sensor_fusion.size(); i++)
                        {
                            // check if the car is in my lane.
                            // NOTE: each lane is 4 meters wide. The center divider is 0.
                            // Tthe middle of the middle lane is d = 6 meters.
                            float d = sensor_fusion[i][6];
                            
                            if (d < (2+4*LANE+2) && d > (2+4*LANE-2) )
                            {
                                // car is within my lane.
                                
                                // check if car is in front and is within 30 meters.
                                // any car within 30 meters is defined to be too close for
                                // save driving and warrants a lane change.
                                float s = sensor_fusion[i][5];
                                
                                double vx = sensor_fusion[i][3];
                                double vy = sensor_fusion[i][4];
                                double check_speed = sqrt(vx*vx+vy*vy);
                                
                                s += ((double)prev_size*0.02*check_speed);
                                
                                // if NOT in the process of changing lanes AND the car in front is too close, enter FSM_near()
                                if (( s - car_s > 0) && (s - car_s < 30) && ((std::abs((2 + 4 * LANE) - car_d) < 0.5)))
                                {
                                    
                                    // initialize and set current state
                                    State current_state;
                                    current_state.car_s = fsm_car_s;
                                    current_state.car_d = car_d;
                                    current_state.car_speed = car_speed * 0.447;    // to meters/s
                                    
                                    // initialize and set current state vector
                                    State_Vectors current_state_vector;
                                    current_state_vector.s = { fsm_car_s, car_speed * 0.447, ((car_speed * 0.447) - (prior_car_speed * 0.447)) / 0.02 };
                                    current_state_vector.d = { 2 + 4 * double(LANE), 0.0, 0.0 };
                                    
                                    // the FSM_near() will return a decision
                                    std::string outcome = FSM(current_state, sensor_fusion);
                                    
                                    // FSM_Near() will return a decision. From this point on, the process is nearly identical
                                    // to FSM_Far() above.
                                    
                                    if (outcome == "turn_left")
                                    {
                                        
                                        // MTG will return a vector of coefficients of a jerk-minimizing trajectory
                                        State_Vectors goal_state_vector;
                                        
                                        // current_state_vector[1] + 3.0 is taking current vel and finding distance after X seconds
                                        goal_state_vector.s.push_back(current_state_vector.s[0] + current_state_vector.s[1] * 4.0);
                                        goal_state_vector.s.push_back(current_state_vector.s[1]);
                                        goal_state_vector.s.push_back(0);
                                        goal_state_vector.d.push_back(current_state_vector.d[0] - 4);
                                        goal_state_vector.d.push_back(0);
                                        goal_state_vector.d.push_back(0);
                                        
                                        // obtain Jerk Minimizing Trajectory to goal state
                                        // timeframe for lane change = 4.0 seconds
                                        if (!MTG(current_state_vector, goal_state_vector, trajectory, 4.0))
                                        {
                                            std::cout << "main() -- trajectory generation failed. will keep lane\n";
                                            
                                            // keep lane means decreasing speed to match the car in front
                                            // and keeping the current lane.
                                            // update target velocity (conversion necessary)
                                            TARGET_VEL = check_speed * 2.24;
                                            keep_lane = true;
                                        }
                                        else
                                        {
                                            // 'trajectory' contains JMT coefficients for a successful lane change.
                                            change_lane = true;
                                            LANE--;
                                        }
                                        
                                    }
                                    else if (outcome == "turn_right")
                                    {
                                        
                                        State_Vectors goal_state_vector;
                                        goal_state_vector.s.push_back(current_state_vector.s[0] + current_state_vector.s[1] * 4.0);
                                        goal_state_vector.s.push_back(current_state_vector.s[1]);
                                        goal_state_vector.s.push_back(0);
                                        goal_state_vector.d.push_back(current_state_vector.d[0] + 4);
                                        goal_state_vector.d.push_back(0);
                                        goal_state_vector.d.push_back(0);
                                        
                                        if (!MTG(current_state_vector, goal_state_vector, trajectory, 4.0))
                                        {
                                            std::cout << "trajectory generation failed. will keep lane\n";
                                            TARGET_VEL = check_speed * 2.24;
                                            keep_lane = true;
                                        }
                                        else
                                        {
                                            change_lane = true;
                                            LANE++;
                                        }
                                        
                                    }
                                    else if (outcome == "keep_lane")
                                    {
                                        // keep the current lane.
                                        // update target velocity and set boolean
                                        TARGET_VEL = check_speed * 2.24;
                                        keep_lane = true;
                                    }
                                    else
                                    {
                                        std::cout << "main() -- ERROR: received no decision from FSM_Near()\n";
                                    }
                                }
                                // if in the process of changing lanes and car in front is too close, reduce speed
                                else if (( s - car_s > 0) && (s - car_s < 30) && !((std::abs((2 + 4 * LANE) - car_d) < 0.5)))
                                {
                                    TARGET_VEL = check_speed * 2.24;
                                    keep_lane = true;
                                }
                            }
                        }
                    }
                    
                    /*
                     
                     ############################################################
                     #####                                                 ######
                     #####  END OF COLLISION DETECTION AND FSM_NEAR()      ######
                     #####                                                 ######
                     ############################################################
                     
                     */
                    
                    
                    // REF_VEL defines the current speed of the car. REF_VEL will always try to catch
                    // TARGET_VEL, as TARGET_VEL is always being updated.
                    if (keep_lane)
                    {
                        // need to slow down
                        if (REF_VEL > TARGET_VEL)
                        {
                            //REF_VEL -= 0.225;
                            REF_VEL -= 0.325;
                        }
                        // need to speed up
                        else if (REF_VEL < TARGET_VEL)
                        {
                            REF_VEL += 0.225;
                        }
                    }
                    else if (REF_VEL < 49.5)
                    {
                        // if i don't have to keep the lane and my current velocity
                        // is less than the speed limit, speed up.
                        REF_VEL += 0.225;
                    }
                    
                    
                    /*
                     
                     ##################################
                     #####                       ######
                     #####  PATH GENERATION      ######
                     #####                       ######
                     ##################################
                     
                     */
                    
                    
                    // Check if the change lane flag is 'true'. If 'true', then 'trajectory' contains
                    // a JMT. Also, the car needs to be near the center of the lane as this is where
                    // the JMT starts.
                    if (change_lane && (std::abs((2 + 4 * LANE) - car_d) < 0.5) )
                    {
                        // if 'change_lane' is true, this indicates that the simulator has been running for
                        // some time and has pre-existing trajectory data.
                        
                        
                        // set reference points. useful in conversion between cartesian and frenet coordinate systems.
                        ref_x = previous_path_x[prev_size - 1];
                        ref_y = previous_path_y[prev_size - 1];
                        double ref_x_prev = previous_path_x[prev_size - 2];
                        double ref_y_prev = previous_path_y[prev_size - 2];
                        
                        // calculate reference yaw
                        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
                        
                        // add these to ptsx and ptsy. These are for spline generation
                        ptsx.push_back(ref_x_prev);
                        ptsy.push_back(ref_y_prev);
                        
                        ptsx.push_back(ref_x);
                        ptsy.push_back(ref_y);
                        
                        // use the JMT to obtain several points along the trajectory given different times.
                        // these points are in frenet coordinate system and must be converted.
                        // These points represent the lane change.
                        vector<double> next_wp0 = getXY(evaluate_poly(trajectory.s_coeffs, 1.5), evaluate_poly(trajectory.d_coeffs, 1.5), map_waypoints_s, map_waypoints_x, map_waypoints_y);
                        vector<double> next_wp1 = getXY(evaluate_poly(trajectory.s_coeffs, 2.5), evaluate_poly(trajectory.d_coeffs, 2.5), map_waypoints_s, map_waypoints_x, map_waypoints_y);
                        vector<double> next_wp2 = getXY(evaluate_poly(trajectory.s_coeffs, 3.5), evaluate_poly(trajectory.d_coeffs, 3.5), map_waypoints_s, map_waypoints_x, map_waypoints_y);
                        vector<double> next_wp3 = getXY(evaluate_poly(trajectory.s_coeffs, 4.0), evaluate_poly(trajectory.d_coeffs, 4.0), map_waypoints_s, map_waypoints_x, map_waypoints_y);
                        
                        
                        // push these points onto ptsx/ptsy
                        ptsx.push_back(next_wp0[0]);
                        ptsx.push_back(next_wp1[0]);
                        ptsx.push_back(next_wp2[0]);
                        ptsx.push_back(next_wp3[0]);
                        
                        ptsy.push_back(next_wp0[1]);
                        ptsy.push_back(next_wp1[1]);
                        ptsy.push_back(next_wp2[1]);
                        ptsy.push_back(next_wp3[1]);
                        
                        // convert these points to car's coordinate system.
                        for (int i = 0; i < ptsx.size(); i++)
                        {
                            double shift_x = ptsx[i] - ref_x;
                            double shift_y = ptsy[i] - ref_y;
                            
                            ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y*sin(0 - ref_yaw));
                            ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y*cos(0 - ref_yaw));
                        }
                        
                        // declare a spline.
                        // A spline is useful for fitting a curve smoothly throughly a trajectory
                        tk::spline s;
                        
                        // set spline points and fit curve
                        s.set_points(ptsx, ptsy);
                        
                        // populate next_x/y_vals with prior path points.
                        // this helps with transitioning between the new lane changing path and the
                        // path the car is currently on.
                        for (int i = 0; i < previous_path_x.size(); i++)
                        {
                            next_x_vals.push_back(previous_path_x[i]);
                            next_y_vals.push_back(previous_path_y[i]);
                        }
                        
                        // this is how to space points along spline to go desired speed.
                        // the sim will place the car at the points fed into it.
                        // the spacing of the points determines how fast the car goes.
                        // the car is being updated every 2 milliseconds
                        
                        double target_x = 30;           // my distance horizon
                        double target_y = s(target_x);
                        double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));       // distance from car to target
                        
                        double x_add_on = 0;        // will be used to add to points
                        
                        for (int i = 0; i <= 50 - prev_size; i++)
                        {
                            double N = (target_dist/(0.02*REF_VEL/2.24));
                            double x_point = x_add_on+(target_dist)/N;
                            // use the spline to obtain the y-value from a given x-value.
                            // these values come from the curve that spline fits to the points given
                            // in its constructor.
                            double y_point = s(x_point);
                            
                            x_add_on = x_point;
                            
                            // convert back to global coordinates and add to trajectory
                            double x_ref = x_point;
                            double y_ref = y_point;
                            
                            // rotate back to normal after rotating it earlier
                            x_point = (x_ref * cos(ref_yaw) - y_ref*sin(ref_yaw));
                            y_point = (x_ref * sin(ref_yaw) + y_ref*cos(ref_yaw));
                            
                            x_point += ref_x;
                            y_point += ref_y;
                            
                            // push onto vector holding the lane-chane trajectory
                            next_x_vals.push_back(x_point);
                            next_y_vals.push_back(y_point);
                        }
                    }
                    else
                    {
                        // change_lane has not be set to 'true', or the car is in the process of changing lanes, or both
                        // this else statement generates a trajectory to keep the lane.
                        
                        // check if there exists a previous path
                        if (2 > prev_size)
                        {
                            // no prior path. Most likley start of simulator.
                            // reference the initial state
                            double prev_car_x = car_x - cos(car_yaw);
                            double prev_car_y = car_y - sin(car_yaw);
                            
                            // populate ptsx and ptsy with these
                            ptsx.push_back(prev_car_x);
                            ptsy.push_back(prev_car_y);
                            
                            ptsx.push_back(car_x);
                            ptsy.push_back(car_y);
                            
                            // don't update reference, since we are in initial state and those variables have
                            // already been set to the initial state
                            
                        }
                        else
                        {
                            // not starting out. a prev path already exists and has been returned from simulator.
                            // update reference variables using this prior path.
                            ref_x = previous_path_x[prev_size - 1];
                            ref_y = previous_path_y[prev_size - 1];
                            double ref_x_prev = previous_path_x[prev_size - 2];
                            double ref_y_prev = previous_path_y[prev_size - 2];
                            
                            // calculate reference yaw
                            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
                            
                            // add these to ptsx and ptsy
                            ptsx.push_back(ref_x_prev);
                            ptsy.push_back(ref_y_prev);
                            
                            ptsx.push_back(ref_x);
                            ptsy.push_back(ref_y);
                            
                        }
                        
                        // now, push 3 more points that are spaced 30 meters apart
                        vector<double> next_wp0 = getXY(car_s+30, (2+4*LANE), map_waypoints_s, map_waypoints_x, map_waypoints_y);
                        vector<double> next_wp1 = getXY(car_s+60, (2+4*LANE), map_waypoints_s, map_waypoints_x, map_waypoints_y);
                        vector<double> next_wp2 = getXY(car_s+90, (2+4*LANE), map_waypoints_s, map_waypoints_x, map_waypoints_y);
                        
                        // push these points onto ptsx/ptsy
                        ptsx.push_back(next_wp0[0]);
                        ptsx.push_back(next_wp1[0]);
                        ptsx.push_back(next_wp2[0]);
                        
                        ptsy.push_back(next_wp0[1]);
                        ptsy.push_back(next_wp1[1]);
                        ptsy.push_back(next_wp2[1]);
                        
                        // convert these points to car's coordinate system
                        // this comes from MPC project
                        for (int i = 0; i < ptsx.size(); i++)
                        {
                            double shift_x = ptsx[i] - ref_x;
                            double shift_y = ptsy[i] - ref_y;
                            
                            ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y*sin(0 - ref_yaw));
                            ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y*cos(0 - ref_yaw));
                        }
                        
                        // declare a spline
                        tk::spline s;
                        
                        // set spline points
                        s.set_points(ptsx, ptsy);
                        
                        // populate next_x/y_vals with prior path points.
                        for (int i = 0; i < previous_path_x.size(); i++)
                        {
                            next_x_vals.push_back(previous_path_x[i]);
                            next_y_vals.push_back(previous_path_y[i]);
                        }
                        
                        // this is how to space points along spline to go desired speed.
                        // the sim will place the car at the points fed into it.
                        // the spacing of the points determines how fast the car goes.
                        // the car is being updated every 2 milliseconds
                        
                        double target_x = 30;           // my distance horizon
                        double target_y = s(target_x);
                        double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));       // distance from car to target
                        
                        double x_add_on = 0;        // will be used to add to points
                        
                        for (int i = 0; i <= 50 - prev_size; i++)
                        {
                            double N = (target_dist/(0.02*REF_VEL/2.24));
                            double x_point = x_add_on+(target_dist)/N;
                            double y_point = s(x_point);
                            
                            x_add_on = x_point;
                            
                            // convert back to global coordinates and add to trajectory
                            double x_ref = x_point;
                            double y_ref = y_point;
                            
                            // rotate back to normal after rotating it earlier
                            x_point = (x_ref * cos(ref_yaw) - y_ref*sin(ref_yaw));
                            y_point = (x_ref * sin(ref_yaw) + y_ref*cos(ref_yaw));
                            
                            x_point += ref_x;
                            y_point += ref_y;
                            
                            // push onto the lane-keep trajectory.
                            // this trajectory will be sent to the simulator.
                            next_x_vals.push_back(x_point);
                            next_y_vals.push_back(y_point);
                        }
                    }
                    
                    /*
                     
                     ##################################
                     #####                       ######
                     #####  END PATH GENERATION  ######
                     #####                       ######
                     ##################################
                     
                     */
                    
                    
                    // set prior car speed.
                    prior_car_speed = car_speed;
                    
                    /// set the trajectory points to give to simulator
                    msgJson["next_x"] = next_x_vals;
                    msgJson["next_y"] = next_y_vals;
                    auto msg = "42[\"control\","+ msgJson.dump()+"]";
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                    
                }
            }
            else
            {
                // Manual driving
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }
    });
    
    // below is more Web Socket code for communcation with the simulator
    
    // We don't need this since we're not using HTTP but if it's removed the
    // program
    // doesn't compile :-(
    h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                       size_t, size_t) {
        const std::string s = "<h1>Hello world!</h1>";
        if (req.getUrl().valueLength == 1) {
            res->end(s.data(), s.length());
        } else {
            // i guess this should be done more gracefully?
            res->end(nullptr, 0);
        }
    });
    
    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
        std::cout << "Connected!!!" << std::endl;
    });
    
    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                           char *message, size_t length) {
        ws.close();
        std::cout << "Disconnected" << std::endl;
    });
    
    int port = 4567;
    if (h.listen(port)) {
        std::cout << "Listening to port " << port << std::endl;
    } else {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }
    h.run();
}

