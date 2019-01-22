//
//  Constants.cpp
//  Path_Planning
//
//  Created by Shawn Ricardo on 8/25/17.
//  Copyright © 2017 Shawn Ricardo. All rights reserved.
//

#include "Constants.hpp"

/*  SET GLOBALS  */

int LANE = 1;
double REF_VEL = 0;
double TARGET_VEL = 49.5;

std::vector<double> SIGMA_S = { 10.0, 4.0, 2.0 };
std::vector<double> SIGMA_D = { 1.0, 1.0, 1.0 };

int N_SAMPLES = 10;

int MAX_JERK = 10;
int MAX_ACCEL = 10;
int EXPECTED_JERK_IN_ONE_SECOND = 2;
int EXPECTED_ACCEL_IN_ONE_SECOND = 1;
