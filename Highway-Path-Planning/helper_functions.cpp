//
//  helper_functions.cpp
//  Path_Planning
//
//  Created by Shawn Ricardo on 8/25/17.
//  Copyright © 2017 Shawn Ricardo. All rights reserved.
//

#include "helper_functions.hpp"
#include "Constants.hpp"
#include "Eigen-3.3/Eigen/Dense"
#include "cost_functions.hpp"

#include <iostream>
#include <vector>
#include <string>
#include <chrono>
#include <algorithm>
#include <random>


void perturb(State_Vectors target, std::vector<State_Vectors> &perturbed_goal, double T)
{
    
    std::random_device rd;
    std::mt19937 e2(rd());
    
    double timestep = 0.5;
    double t = T - 4 * timestep;
    
    while (t <= T + 4 * timestep)
    {
        State_Vectors local_state;
        
        // generate "N_SAMPLES" of perturbed goal states at each time step
        for (int j = 0; j < N_SAMPLES; j++)
        {
            
            for (int i = 0; i < SIGMA_S.size(); i++)
            {
                std::normal_distribution<double> dist_s(target.s[i], SIGMA_S[i]);
                std::normal_distribution<double> dist_d(target.d[i], SIGMA_D[i]);
                
                // generate perturbed version of the goal
                local_state.s.push_back(dist_s(e2));
                local_state.d.push_back(dist_d(e2));
            }
            
            local_state.t = t;
            perturbed_goal.push_back(local_state);
        }
        t += timestep;
    }
    
}


std::vector<double> JMT(std::vector<double> start, std::vector <double> end, double T)
{
    
    Eigen::MatrixXd A = Eigen::MatrixXd(3, 3);
    A <<  T*T*T,   T*T*T*T,   T*T*T*T*T,
    3 * T*T, 4 * T*T*T, 5 * T*T*T*T,
    6 * T,  12 * T*T,  20 * T*T*T;
    
    Eigen::MatrixXd B = Eigen::MatrixXd(3, 1);
    B << end[0] - (start[0] + start[1] * T + .5*start[2] * T*T),
    end[1] - (start[1] + start[2] * T),
    end[2] - start[2];
    
    Eigen::MatrixXd Ai = A.inverse();
    
    Eigen::MatrixXd C = Ai*B;
    
    std::vector <double> result = { start[0], start[1], .5*start[2] };
    
    for (int i = 0; i < C.size(); i++)
    {
        result.push_back(C.data()[i]);
    }
    
    return result;
    
}

std::vector<double> differentiate(std::vector<double> coeffs)
{
    std::vector<double> new_cos;
    
    int count = 0;
    
    for (int i = 1; i < coeffs.size(); i++)
    {
        new_cos.push_back((count + 1) * coeffs[i]);
        count++;
    }
    
    return new_cos;
}

double evaluate_poly(std::vector<double> coeffs, double T)
{
    double total = 0.0;
    for (int i = 0; i < coeffs.size(); i++)
        total += coeffs[i] * (pow(T, i));
    return total;
}

std::string FSM(State initial_state, std::vector< std::vector<double> > sensors)
{
    // FSM decision that will be returned
    std::string decision = "error";
    
    // start with far left lane
    if (0 == LANE)
    {
        // set adjacent lane
        int right_lane = 1;
        
        // container to hold cars in adjacent lane
        std::vector<int> cars_in_right_lane;
        
        // variables to describe closest car in adjacent lane
        double distance_of_closest_right = 1000000.0;
        double speed_of_closest_right = 0.0;
        int index_of_closest_right = 0;
        
        // booleans describing adjacent lane
        bool forward_right_lane_clear = true;
        bool rear_right_lane_clear = true;
        
        // find cars in right lane
        for (int i = 0; i < sensors.size(); i++)
        {
            // current car's d parameter
            float d = sensors[i][6];
            
            // check if d is within right lane
            if (   (    d < (2+4*right_lane+2) && (d > (2+4*right_lane-2))  )    )
            {
                // add car's index to vector
                cars_in_right_lane.push_back(i);
            }
        }
        
        // see if any cars are preventing a lane change
        for (int i = 0; i < cars_in_right_lane.size(); i++)
        {
            // get car's s value
            float s = sensors[cars_in_right_lane[i]][5];
            
            // obtain speed
            double vx = sensors[cars_in_right_lane[i]][3];
            double vy = sensors[cars_in_right_lane[i]][4];
            double check_speed = sqrt(vx*vx+vy*vy);
            
            float distance_to_car = s - initial_state.car_s;
            
            // check if car is too close on the rear
            //if (  s - initial_state.car_s < 0 && initial_state.car_s - s < 5 )
            if (distance_to_car < 0)
            {
                
                //if (std::abs(distance_to_car) <= 5)
                if (initial_state.car_s - s < 5)
                {
                    // lane change not safe
                    std::cout << "FSM_Near() -- car too close on the rear right side\n";
                    rear_right_lane_clear = false;
                    continue;
                }
                //else if (std::abs(distance_to_car) <= 20 && check_speed > initial_state.car_speed)
                else if (initial_state.car_s - s <= 20 && check_speed > initial_state.car_speed)
                {
                    // lane change not safe
                    std::cout << "FSM_Near() -- car too fast on the rear right side\n";
                    rear_right_lane_clear = false;
                    continue;
                }
                else
                {
                    std::cout << "FSM_Near() -- no rear right obstruction\n";
                }
            }
            
            // check if car is in front
            if (s - initial_state.car_s > 0)
            {
                // obtain speed
                double vx = sensors[cars_in_right_lane[i]][3];
                double vy = sensors[cars_in_right_lane[i]][4];
                double check_speed = sqrt(vx*vx+vy*vy);
                
                // check if car is within 30 meters
                if(distance_to_car < 30)
                {
                    // car is too close for a safe lane change
                    std::cout << "FSM_Near() -- car too close on the front right side\n";
                    forward_right_lane_clear = false;
                    
                    // if car is the closest on right side, save it's state
                    if (distance_to_car < distance_of_closest_right)
                    {
                        distance_of_closest_right = distance_to_car;
                        speed_of_closest_right = check_speed;
                        index_of_closest_right = i;
                    }
                    
                    continue;
                }
                else
                {
                    // lane is clear.
                    // record this car's state
                    if (distance_to_car < distance_of_closest_right)
                        distance_of_closest_right = distance_to_car;
                }
                
            }
             
        }
        
        if (forward_right_lane_clear && rear_right_lane_clear)
        {
            std::cout << "FSM_Near() -- turn right\n";
            decision = "turn_right";
        }
        else
        {
            // check if the car in right lane is moving faster
            // and not too close.
            if (rear_right_lane_clear)
            {
                if (speed_of_closest_right > initial_state.car_speed && 20 < distance_of_closest_right)
                {
                    std::cout << "FSM_Near() -- turn right\n";
                    decision = "turn_right";
                }
                else
                {
                    std::cout << "FSM_Near() -- keep lane\n";
                    decision = "keep_lane";
                }
            }
            else
            {
                std::cout << "FSM_Near() -- keep lane\n";
                decision = "keep_lane";
            }
        }
    }
    else if (2 == LANE)     // far right lane
    {
        // the only lane we can change to safely. adjacent left lane.
        // the process here is the same as for (0 == LANE), only
        // targeted for the car being in the far right lane.
        int left_lane = 1;
        
        // variables to describe closest car in adjacent lane
        double distance_of_closest_left = 1000000.0;
        double speed_of_closest_left = 0.0;
        int index_of_closest_left = 0;
        
        // hold indecies of cars in left lane
        std::vector<int> cars_in_left_lane;
        
        bool forward_left_lane_clear = true;
        bool rear_left_lane_clear = true;
        
        // find cars in left lane lane
        for (int i = 0; i < sensors.size(); i++)
        {
            
            // current car's d parameter
            float d = sensors[i][6];
            
            // check if d is within left lane
            if (   (    d < (2+4*left_lane+2) && (d > (2+4*left_lane-2))  )    )
            {
                // add car's index to vector
                cars_in_left_lane.push_back(i);
            }
        }
        
        
        // check if cars in left lane prevent lane change
        for (int i = 0; i < cars_in_left_lane.size(); i++)
        {
            float s = sensors[cars_in_left_lane[i]][5];
            
            // obtain speed
            double vx = sensors[cars_in_left_lane[i]][3];
            double vy = sensors[cars_in_left_lane[i]][4];
            double check_speed = sqrt(vx*vx+vy*vy);
            
            float distance_to_car = s - initial_state.car_s;
            
            // car is in rear
            //if (s - initial_state.car_s < 0 && initial_state.car_s - s <= 5)
            if (distance_to_car < 0)
            {
                
                //if (std::abs(distance_to_car) <= 5)
                if (initial_state.car_s - s <= 5)
                {
                    std::cout << "FSM_Near() -- car too close on the rear left side\n";
                    rear_left_lane_clear = false;
                    continue;
                }
                else if (initial_state.car_s - s <= 20 && check_speed > initial_state.car_speed)
                {
                    std::cout << "FSM_Near() -- car too fast on the rear left side\n";
                    rear_left_lane_clear = false;
                    continue;
                }
                else
                {
                    std::cout << "FSM_Near() -- no rear left obstruction\n";
                }
            }
            
            // check if car is in front
            if (s - initial_state.car_s > 0)
            {
                
                // obtain speed
                double vx = sensors[cars_in_left_lane[i]][3];
                double vy = sensors[cars_in_left_lane[i]][4];
                double check_speed = sqrt(vx*vx+vy*vy);
                
                // check if car is within 30 meters
                if(distance_to_car < 30)
                {
                    // car is too close for a safe lane change
                    std::cout << "FSM_Near() -- car too close on the front left side\n";
                    forward_left_lane_clear = false;
                    
                    if (distance_to_car < distance_of_closest_left)
                    {
                        distance_of_closest_left = distance_to_car;
                        speed_of_closest_left = check_speed;
                        index_of_closest_left = i;
                    }
                    
                    continue;
                }
                else
                {
                    // lane is clear. record this distance
                    if (distance_to_car < distance_of_closest_left)
                        distance_of_closest_left = distance_to_car;
                }
                
            }
        }
        
        
        if (forward_left_lane_clear && rear_left_lane_clear)
        {
            std::cout << "FSM_Near() -- turn left\n";
            decision = "turn_left";
        }
        else
        {
            // check if the car in left lane is moving faster
            if (rear_left_lane_clear)
            {
                if (speed_of_closest_left > initial_state.car_speed && 20 < distance_of_closest_left)
                {
                    std::cout << "FSM_Near() -- turn left\n";
                    decision = "turn_left";
                }
                else
                {
                    std::cout << "FSM_Near() -- keep lane\n";
                    decision = "keep_lane";
                }
            }
            else
            {
                std::cout << "FSM_Near() -- keep lane\n";
                decision = "keep_lane";
            }
        }
    }
    else if (1 == LANE)
    {
        // Car is in middle lane.
        // Process is same as the two above.
        
        int left_lane = 0;
        int right_lane = 2;
        
        // variables to record other vehicle information
        float distance_of_closest_right = 1000000.0;
        float distance_of_closest_left = 1000000.0;
        
        double speed_of_closest_right = 0.0;
        double speed_of_closest_left = 0.0;
        
        int index_of_closest_right = 0;
        int index_of_closest_left = 0;
        
        
        std::vector<int> cars_in_left_lane;
        std::vector<int> cars_in_right_lane;
        
        bool forward_right_lane_clear = true;
        bool forward_left_lane_clear = true;
        bool rear_right_lane_clear = true;
        bool rear_left_lane_clear = true;
        
        // find cars in left lane and right lane
        for (int i = 0; i < sensors.size(); i++)
        {
            // current car's d parameter
            float d = sensors[i][6];
            
            // check if d is within left lane
            if (   (    d < (2+4*left_lane+2) && (d > (2+4*left_lane-2))  )    )
            {
                // add car's index to vector
                cars_in_left_lane.push_back(i);
                continue;
            }
            
            // check if d is within right lane
            if (   (    d < (2+4*right_lane+2) && (d > (2+4*right_lane-2))  )    )
            {
                // add car's index to vector
                cars_in_right_lane.push_back(i);
            }
        }
        
        // check if cars in right lane prevent lane change
        for (int i = 0; i < cars_in_right_lane.size(); i++)
        {
            float s = sensors[cars_in_right_lane[i]][5];
            
            // obtain speed
            double vx = sensors[cars_in_right_lane[i]][3];
            double vy = sensors[cars_in_right_lane[i]][4];
            double check_speed = sqrt(vx*vx+vy*vy);
            
            float distance_to_car = s - initial_state.car_s;
            
            //if (distance_to_car < 0 && initial_state.car_s - s <= 5)
            // car is in rear
            if (distance_to_car < 0)
            {
                if (initial_state.car_s - s <= 5)
                {
                    std::cout << "FSM_Near() -- car too close on the rear right side\n";
                    rear_right_lane_clear = false;
                    continue;
                }
                else if (initial_state.car_s - s <= 10 && check_speed > initial_state.car_speed)
                {
                    std::cout << "FSM_Near() -- car too fast on the rear right side\n";
                    rear_right_lane_clear = false;
                    continue;
                }
                else
                {
                    std::cout << "FSM_Near() -- no rear right obstruction\n";
                }
            }
            
            // check if car is in front
            if (distance_to_car > 0)
            {
                
                // obtain speed
                double vx = sensors[cars_in_right_lane[i]][3];
                double vy = sensors[cars_in_right_lane[i]][4];
                double check_speed = sqrt(vx*vx+vy*vy);
                
                // check if car is within 40 meters
                if(distance_to_car < 30)
                {
                    // car is too close for a safe lane change
                    std::cout << "FSM_Near() -- car too close on the front right side\n";
                    forward_right_lane_clear = false;
                    
                    if (distance_to_car < distance_of_closest_right)
                    {
                        distance_of_closest_right = distance_to_car;
                        speed_of_closest_right = check_speed;
                        index_of_closest_right = i;
                    }
                    
                    continue;
                }
                else
                {
                    // lane is clear. record this distance
                    if (distance_to_car < distance_of_closest_right)
                        distance_of_closest_right = distance_to_car;
                }
                
            }
            
        }
        
        // check if cars in left lane prevent lane change
        for (int i = 0; i < cars_in_left_lane.size(); i++)
        {
            float s = sensors[cars_in_left_lane[i]][5];
            
            // obtain speed
            double vx = sensors[cars_in_left_lane[i]][3];
            double vy = sensors[cars_in_left_lane[i]][4];
            double check_speed = sqrt(vx*vx+vy*vy);
            
            float distance_to_car = s - initial_state.car_s;
            
            //if (s - initial_state.car_s < 0 && initial_state.car_s - s < 5)
            // car is in rear
            if (distance_to_car < 0)
            {
                
                if (initial_state.car_s - s <= 5)
                {
                    std::cout << "FSM_Near() -- car too close on the rear left side\n";
                    rear_left_lane_clear = false;
                    continue;
                }
                else if (initial_state.car_s - s <= 10 && check_speed > initial_state.car_speed)
                {
                    std::cout << "FSM_Near() -- car too fast on the rear left side\n";
                    rear_left_lane_clear = false;
                    continue;
                }
                else
                {
                    std::cout << "FSM_Near() -- no rear left obstruction\n";
                }
            }
            
            // check if car is in front
            if (s - initial_state.car_s > 0)
            {
                // obtain speed
                double vx = sensors[cars_in_left_lane[i]][3];
                double vy = sensors[cars_in_left_lane[i]][4];
                double check_speed = sqrt(vx*vx+vy*vy);
                
                // check if car is within 30 meters
                if(distance_to_car < 30)
                {
                    // car is too close for a safe lane change
                    std::cout << "FSM_Near() -- car too close on the front left side\n";
                    forward_left_lane_clear = false;
                    
                    if (distance_to_car < distance_of_closest_left)
                    {
                        distance_of_closest_left = distance_to_car;
                        speed_of_closest_left = check_speed;
                        index_of_closest_left = i;
                    }
                    
                    continue;
                }
                else
                {
                    // lane is clear. record this distance
                    if (distance_to_car < distance_of_closest_left)
                        distance_of_closest_left = distance_to_car;
                }
                
            }
        }
        
        // if both lanes are clear, choose the lane with greatest forward clearance
        // UPDATE: incorpate speed, as well
        if (forward_left_lane_clear && forward_right_lane_clear && rear_left_lane_clear && rear_right_lane_clear)
        {
            if (distance_of_closest_right > distance_of_closest_left)
                forward_left_lane_clear = false;
        }
        
        // passing should happen on left lane, if in middle
        if (forward_left_lane_clear && rear_left_lane_clear)
        {
            std::cout << "FSM_Near() -- turn left\n";
            decision = "turn_left";
        }
        else if (forward_right_lane_clear && rear_right_lane_clear)
        {
            std::cout << "FSM_Near() -- turn right\n";
            decision = "turn_right";
        }
        else
        {
            // left & right lane not clear for normal lane change. try to get behind the faster moving vehicle
            
            if (rear_left_lane_clear && rear_right_lane_clear)
            {
            
                // keep lane if ego car is moving faster than cars in either lane
                if (initial_state.car_speed > speed_of_closest_right && initial_state.car_speed > speed_of_closest_right)
                {
                    std::cout << "FSM_Near() -- keep lane\n";
                    decision = "keep_lane";
                }
                else
                {
                    // find which car is faster
                    if (speed_of_closest_right > speed_of_closest_left && speed_of_closest_right > initial_state.car_speed)
                    {
                        // right car is faster, see if minimum distance is met for lane change
                        if (20 < distance_of_closest_right)
                        {
                            std::cout << "FSM_Near() -- turn right\n";
                            decision = "turn_right";
                        }
                        else
                        {
                            std::cout << "FSM_Near() -- keep lane\n";
                            decision = "keep_lane";
                        }
                    }
                    else if (speed_of_closest_left > speed_of_closest_right && speed_of_closest_left > initial_state.car_speed)
                    {
                        // left car is faster, see if minimum distance is met for lane change
                        if (20 < distance_of_closest_left)
                        {
                            std::cout << "FSM_Near() -- turn left\n";
                            decision = "turn_left";
                        }
                        else
                        {
                            std::cout << "FSM_Near() -- keep lane\n";
                            decision = "keep_lane";
                        }
                    }
                    else
                    {
                        std::cout << "FSM_Near() -- keep lane\n";
                        decision = "keep_lane";
                    }
                }
            }
            else
            {
                std::cout << "FSM_Near() -- keep lane\n";
                decision = "keep_lane";
            }
        }
    }
    else
    {
        std::cout << "FSM_Near() -- ERROR: no lane defined\n";
    }
    
    return decision;
}


std::string FSM_Far(State initial_state, std::vector< std::vector<double> > sensors)
{
    std::string decision = "error";
    
    if (0 == LANE)
    {
        int right_lane = 1;
        int far_right = 2;
        
        // variables to record other vehicle information
        float distance_of_closest_right = 1000000.0;
        float distance_of_closest_far_right = 1000000.0;
        float distance_of_closest_front = 1000000.0;
        
        float distance_of_closest_rear_right = 1000000.0;
        
        double speed_of_closest_right = 0.0;
        double speed_of_closest_far_right = 0.0;
        double speed_of_closest_front = 0.0;
        double speed_of_closest_rear_right = 0.0;
        
        int index_of_closest_right = 0;
        int index_of_closest_far_right = 0;
        int index_of_closest_front = 0;

        // find cars
        for (int i = 0; i < sensors.size(); i++)
        {
            // current car's d parameter
            float d = sensors[i][6];
            
            // current car s parameter
            float s = sensors[i][5];
            
            float distance_between_cars = s - initial_state.car_s;
            
            //current car speed
            double vx = sensors[i][3];
            double vy = sensors[i][4];
            double check_speed = sqrt(vx*vx+vy*vy);
            
            // check if d is within right lane
            if (   (    d < (2+4*right_lane+2) && (d > (2+4*right_lane-2))  )    )
            {
                
                // car is in front
                if (distance_between_cars > 0)
                {
                    
                    if (distance_between_cars < distance_of_closest_right)
                    {
                        distance_of_closest_right = distance_between_cars;
                        speed_of_closest_right = check_speed;
                        index_of_closest_right = i;
                    }
                }
                // car is in rear
                else
                {
                    if (std::abs(distance_between_cars) < distance_of_closest_rear_right)
                    {
                        distance_of_closest_rear_right = std::abs(distance_between_cars);
                        speed_of_closest_rear_right = check_speed;
                    }
                }
            }
            
            // check if d is within far right lane
            else if (   (    d < (2+4*far_right+2) && (d > (2+4*far_right-2))  )    )
            {
                if (distance_between_cars > 0)
                {
                    if (distance_between_cars < distance_of_closest_far_right)
                    {
                        distance_of_closest_far_right = distance_between_cars;
                        speed_of_closest_far_right = check_speed;
                        index_of_closest_far_right = i;
                    }
                }
            }
            else if (   (    d < (2+4*LANE+2) && (d > (2+4*LANE-2))  )    )
            {
                if (distance_between_cars > 0)
                {
                    if (distance_between_cars < distance_of_closest_front)
                    {
                        distance_of_closest_front = distance_between_cars;
                        speed_of_closest_front = check_speed;
                        index_of_closest_front = i;
                    }
                }
            }
        }
        
        if (distance_of_closest_rear_right >= 5)
        {
            //if (15 <= distance_of_closest_rear_right && speed_of_closest_rear_right > initial_state.car_speed)
            if (distance_of_closest_rear_right >= 15 || speed_of_closest_rear_right <= initial_state.car_speed)
            {
            
                // UPDATE: incorporate speed
                if (distance_of_closest_far_right > distance_of_closest_right && distance_of_closest_far_right > distance_of_closest_front)
                {
                    // farthest right lane clear, check if middle is blocking
                    if (40 > distance_of_closest_right)
                    {
                        // middle is blocking so do nothing
                        std::cout << "FSM_Far() -- car blocked on right lane\n";
                        decision = "keep_lane";
                    }
                    else
                    {
                        std::cout << "FSM_Far() -- move to middle lane\n";
                        decision = "far_right";
                    }
                }
                else
                {
                    // for now, just return keep lane.
                    std::cout << "FSM_Far() -- farthest right lane not best\n";
                    decision = "keep_lane";
                }
            }
            else
            {
                std::cout << "FSM_Far() -- car too fast on rear right lane: " << speed_of_closest_rear_right << std::endl;
                decision = "keep_lane";
            }
        }
        else
        {
            std::cout << "FSM_Far() -- car too close on rear right lane: " << distance_of_closest_rear_right << std::endl;
            decision = "keep_lane";
        }
        
        
    }
    else if (2 == LANE)
    {
        int left_lane = 1;
        int far_left_lane = 0;
        
        // variables to record other vehicle information
        float distance_of_closest_left = 1000000.0;
        float distance_of_closest_far_left = 1000000.0;
        float distance_of_closest_front = 1000000.0;
        
        float distance_of_closest_rear_left = 1000000.0;
        
        double speed_of_closest_left = 0.0;
        double speed_of_closest_far_left = 0.0;
        double speed_of_closest_front = 0.0;
        double speed_of_closest_rear_left = 0.0;
        
        int index_of_closest_left = 0;
        int index_of_closest_far_left = 0;
        int index_of_closest_front = 0;
        
        // find cars
        for (int i = 0; i < sensors.size(); i++)
        {
            // current car's d parameter
            float d = sensors[i][6];
            
            // current car s parameter
            float s = sensors[i][5];
            
            float distance_between_cars = s - initial_state.car_s;
            
            //current car speed
            double vx = sensors[i][3];
            double vy = sensors[i][4];
            double check_speed = sqrt(vx*vx+vy*vy);
            
            // check if d is within left lane
            if (   (    d < (2+4*left_lane+2) && (d > (2+4*left_lane-2))  )    )
            {
                // car is in front
                if (distance_between_cars > 0)
                {
                    if (distance_between_cars < distance_of_closest_left)
                    {
                        distance_of_closest_left = distance_between_cars;
                        speed_of_closest_left = check_speed;
                        index_of_closest_left = i;
                    }
                }
                // car is in rear
                else
                {
                    if (std::abs(distance_between_cars) < distance_of_closest_rear_left)
                    {
                        distance_of_closest_rear_left = std::abs(distance_between_cars);
                        speed_of_closest_rear_left = check_speed;
                    }
                }
            }
            
            // check if d is within far right lane
            else if (   (    d < (2+4*far_left_lane+2) && (d > (2+4*far_left_lane-2))  )    )
            {
                if (distance_between_cars > 0)
                {
                    if (distance_between_cars < distance_of_closest_far_left)
                    {
                        distance_of_closest_far_left = distance_between_cars;
                        speed_of_closest_far_left = check_speed;
                        index_of_closest_far_left = i;
                    }
                }
            }
            else if (   (    d < (2+4*LANE+2) && (d > (2+4*LANE-2))  )    )
            {
                if (distance_between_cars > 0)
                {
                    if (distance_between_cars < distance_of_closest_front)
                    {
                        distance_of_closest_front = distance_between_cars;
                        speed_of_closest_front = check_speed;
                        index_of_closest_front = i;
                    }
                }
            }
        }
        
        
        if (distance_of_closest_rear_left >= 5)
        {
            if (distance_of_closest_rear_left >= 15 || speed_of_closest_rear_left <= initial_state.car_speed)
            {
                
                // update to go behind fastest car
                
                if (distance_of_closest_far_left > distance_of_closest_left && distance_of_closest_far_left > distance_of_closest_front)
                {
                    // farthest right lane clear, check if middle is blocking
                    if (40 > distance_of_closest_left)
                    {
                        // middle is blocking so do nothing
                        std::cout << "FSM_Far() -- car blocked on left lane\n";
                        decision = "keep_lane";
                    }
                    else
                    {
                        std::cout << "FSM_Far() -- move to middle lane\n";
                        decision = "far_left";
                    }
                }
                else
                {
                    // for now, just return keep lane.
                    std::cout << "FSM_Far() -- farthest left lane not best\n";
                    decision = "keep_lane";
                }
            }
            else
            {
                std::cout << "FSM_Far() -- car too fast on rear left lane: " << speed_of_closest_rear_left << std::endl;
                decision = "keep_lane";
            }
        }
        else
        {
            std::cout << "FSM_Far() -- car too close on rear left lane: " << distance_of_closest_rear_left << std::endl;
            decision = "keep_lane";
        }                       
    }
    else
    {
        std::cout << "FSM_Far() -- ERROR: no lane defined\n";
    }
    
    return decision;
}


// multiple trajectory generation
bool MTG(const State_Vectors initial_state, const State_Vectors final_state, Trajectory_Coeffs &jmt, double T)
{
    
    bool generation_complete = false;
    
    // generate random goal states;
    std::vector<State_Vectors> perturbed_goals;
    perturb(final_state, perturbed_goals, T);
    
    // find a trajectory for each perturbed goal
    std::vector<Trajectory_Coeffs> JMT_coeffs;
    for (int i = 0; i < perturbed_goals.size(); i++)
    {
        Trajectory_Coeffs local;
        local.s_coeffs = JMT(initial_state.s, perturbed_goals[i].s, perturbed_goals[i].t);
        local.d_coeffs = JMT(initial_state.d, perturbed_goals[i].d, perturbed_goals[i].t);
        local.t = perturbed_goals[i].t;
        JMT_coeffs.push_back(local);
    }
    
    /** EVALUATE COST FUNCTIONS **/
    
    // keep record of all violating trajectories
    std::vector<int> jerk_violating_trajectories;
    
    // max jerk
    for (int i = 0; i < JMT_coeffs.size(); i++)
    {
        if (max_jerk_cost(JMT_coeffs[i], JMT_coeffs[i].t))
            jerk_violating_trajectories.push_back(i);
    }
    
    // max accel
    for (int i = 0; i < JMT_coeffs.size(); i++)
    {
        // if current trajectory did not violate any constraints, then evaluate
        if (!(std::find(jerk_violating_trajectories.begin(), jerk_violating_trajectories.end(), i) != jerk_violating_trajectories.end()))
        {
            if (max_accel_cost(JMT_coeffs[i], JMT_coeffs[i].t))
                jerk_violating_trajectories.push_back(i);
        }
    }
    
    // total jerk
    for (int i = 0; i < JMT_coeffs.size(); i++)
    {
        // if current trajectory did not violate any constraints, then evaluate
        if (!(std::find(jerk_violating_trajectories.begin(), jerk_violating_trajectories.end(), i) != jerk_violating_trajectories.end()))
        {
            if (total_jerk_cost(JMT_coeffs[i], JMT_coeffs[i].t))
                jerk_violating_trajectories.push_back(i);
        }
    }
    
    // total accel
    for (int i = 0; i < JMT_coeffs.size(); i++)
    {
        // if current trajectory did not violate any constraints, then evaluate
        if (!(std::find(jerk_violating_trajectories.begin(), jerk_violating_trajectories.end(), i) != jerk_violating_trajectories.end()))
        {
            if (total_accel_cost(JMT_coeffs[i], JMT_coeffs[i].t))
                jerk_violating_trajectories.push_back(i);
        }
    }
    
    std::cout << float(jerk_violating_trajectories.size()) / float(JMT_coeffs.size()) * 100 << "% of trajectories violated cost functions\n";
    
    if (jerk_violating_trajectories.size() != JMT_coeffs.size())
    {
        
        for (int i = 0; i < JMT_coeffs.size(); i++)
        {
            if (!(std::find(jerk_violating_trajectories.begin(), jerk_violating_trajectories.end(), i) != jerk_violating_trajectories.end()))
            {
                // set jmt argument to first non-violating trajectory and return
                jmt.s_coeffs = JMT_coeffs[i].s_coeffs;
                jmt.d_coeffs = JMT_coeffs[i].d_coeffs;
                jmt.t = JMT_coeffs[i].t;
                break;
            }
        }
        
        generation_complete = true;
    }
    
    return generation_complete;
}









