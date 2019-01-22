//
//  cost_functions.hpp
//  Path_Planning
//
//  Created by Shawn Ricardo on 8/25/17.
//  Copyright © 2017 Shawn Ricardo. All rights reserved.
//

#ifndef cost_functions_hpp
#define cost_functions_hpp

#include <stdio.h>
#include "Constants.hpp"


// will return false if the instantaneous jerk is greater than allowed
bool max_jerk_cost(Trajectory_Coeffs traj, double T);

// wil return false if the instantaneous acceleration is greater than allowed
bool max_accel_cost(Trajectory_Coeffs traj, double T);

// will return false if jerk per second is greater than 2 m^s3 over the entire
// trajectory
bool total_jerk_cost(Trajectory_Coeffs traj, double T);

// will return false if acceleration per second is greater than 1 m/s^2 over
// the entire trajectory
bool total_accel_cost(Trajectory_Coeffs traj, double T);

#endif /* cost_functions_hpp */
