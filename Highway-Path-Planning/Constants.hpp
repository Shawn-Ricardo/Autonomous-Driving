//
//  Constants.hpp
//  Path_Planning
//
//  Created by Shawn Ricardo on 8/25/17.
//  Copyright © 2017 Shawn Ricardo. All rights reserved.
//

#ifndef Constants_hpp
#define Constants_hpp

#include <stdio.h>
#include <vector>
#include <string>

/* DEFINE GLOBALS */

// define the lane the vehicle is in
extern int LANE;

// set to the speed limit
extern double REF_VEL;

// the velocity the car will move toward.
// in the event the car must slow down, target_vel
// will be the reduced speed
extern double TARGET_VEL;

// the standard deviations used in perturb()
extern std::vector<double> SIGMA_S;
extern std::vector<double> SIGMA_D;

// number of perturbed samples to draw from
// gaussian
extern int N_SAMPLES;

// constraints that the vehicle must not break
extern int MAX_JERK;
extern int MAX_ACCEL;
extern int EXPECTED_JERK_IN_ONE_SECOND;
extern int EXPECTED_ACCEL_IN_ONE_SECOND;


/* DEFINE STRUCTURES -- useful for describing states of the vehicle */

typedef struct state_{
    double car_s;
    double car_d;
    double car_speed;
} State;

// s and d should be 3 element vectors, containing
// s: s, s_dot, s_dot_dot
// d: d, d_dot, d_dot_dot
typedef struct state_vectors_{
    std::vector<double> s;
    std::vector<double> d;
    double t;
} State_Vectors;

// will hold the coefficients of jerk-minimizing trajectories
typedef struct trajectory_coeffs_ {
    std::vector<double> s_coeffs;
    std::vector<double> d_coeffs;
    double t;
} Trajectory_Coeffs;


#endif /* Constants_hpp */

