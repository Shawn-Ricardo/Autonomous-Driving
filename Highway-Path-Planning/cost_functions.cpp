//
//  cost_functions.cpp
//  Path_Planning
//
//  Created by Shawn Ricardo on 8/25/17.
//  Copyright © 2017 Shawn Ricardo. All rights reserved.
//

#include "cost_functions.hpp"
#include "helper_functions.hpp"
#include "Constants.hpp"

#include <vector>
#include <iostream>
#include <cmath>

bool max_jerk_cost(Trajectory_Coeffs traj, double T)
{
    
    bool violated = false;
    
    // get vel, accel, and jerk of s
    std::vector<double> s_dot = differentiate(traj.s_coeffs);
    std::vector<double> s_d_dot = differentiate(s_dot);
    std::vector<double> s_jerk = differentiate(s_d_dot);
    
    // get vel, accel, and jerk of d
    std::vector<double> d_dot = differentiate(traj.d_coeffs);
    std::vector<double> d_d_dot = differentiate(d_dot);
    std::vector<double> d_jerk = differentiate(d_d_dot);
    
    // break timeframe up into 100, and evaluate jerk at each step.
    float dt = float(T) / 100;
    
    for (int i = 0; i < 100; i++)
    {
        
        double j = evaluate_poly(s_jerk, dt * i);
        double k = evaluate_poly(d_jerk, dt * i);
        
        if (std::abs(j) > MAX_JERK)
        {
            violated = true;
            break;
        }
        else if (std::abs(k) > MAX_JERK)
        {
            violated = true;
            break;
        }
    }
    
    return violated;
}

bool total_jerk_cost(Trajectory_Coeffs traj, double T)
{
    bool violated = false;
    
    // get vel, accel, and jerk of s
    std::vector<double> s_dot = differentiate(traj.s_coeffs);
    std::vector<double> s_d_dot = differentiate(s_dot);
    std::vector<double> s_jerk = differentiate(s_d_dot);
    
    // get vel, accel, and jerk of d
    std::vector<double> d_dot = differentiate(traj.d_coeffs);
    std::vector<double> d_d_dot = differentiate(d_dot);
    std::vector<double> d_jerk = differentiate(d_d_dot);
    
    // break timeframe up into 100, and evaluate jerk at each step.
    float dt = float(T) / 100;
    
    double s_total_jerk = 0;
    double d_total_jerk = 0;
    
    for (int i = 0; i < 100; i++)
    {
        double t = dt * i;
        double j = evaluate_poly(s_jerk, t);
        double k = evaluate_poly(d_jerk, t);
        
        s_total_jerk += std::abs(j * dt);
        d_total_jerk += std::abs(k * dt);
    }
    
    double s_jerk_per_second = s_total_jerk / T;
    double d_jerk_per_second = d_total_jerk / T;
    
    if (1 < s_jerk_per_second / EXPECTED_JERK_IN_ONE_SECOND)
        violated = true;
    else if (1 < d_jerk_per_second / EXPECTED_JERK_IN_ONE_SECOND)
        violated = true;
    
    return violated;
    
}

bool max_accel_cost(Trajectory_Coeffs traj, double T)
{
    bool violated = false;
    
    // get velocity and acceleration in s
    std::vector<double> s_dot = differentiate(traj.s_coeffs);
    std::vector<double> s_d_dot = differentiate(s_dot);
    
    // get velocity and accerlation in d
    std::vector<double> d_dot = differentiate(traj.d_coeffs);
    std::vector<double> d_d_dot = differentiate(d_dot);
    
    // break timeframe into 100, evaluate accel at each step
    double dt = float(T) / 100.0;
    
    for (int i = 0; i < 100; i++)
    {
        
        double s_acc = evaluate_poly(s_d_dot, dt * i);
        double d_acc = evaluate_poly(d_d_dot, dt * i);
        
        if (std::abs(s_acc) > MAX_ACCEL)
        {
            violated = true;
            break;
        }
        else if (std::abs(d_acc) > MAX_ACCEL)
        {
            violated = true;
            break;
        }
    }
    
    return violated;
}


bool total_accel_cost(Trajectory_Coeffs traj, double T)
{
    bool violated = false;
    
    // get vel and accel of s
    std::vector<double> s_dot = differentiate(traj.s_coeffs);
    std::vector<double> s_d_dot = differentiate(s_dot);
    
    // get vel and accel of d
    std::vector<double> d_dot = differentiate(traj.d_coeffs);
    std::vector<double> d_d_dot = differentiate(d_dot);
    
    // break timeframe up into 100, and evaluate jerk at each step.
    float dt = float(T) / 100;
    
    double s_total_accel = 0;
    double d_total_accel = 0;
    
    for (int i = 0; i < 100; i++)
    {
        double t = dt * i;
        double j = evaluate_poly(s_d_dot, t);
        double k = evaluate_poly(d_d_dot, t);
        
        s_total_accel += std::abs(j * dt);
        d_total_accel += std::abs(k * dt);
    }
    
    double s_accel_per_second = s_total_accel / T;
    double d_accel_per_second = d_total_accel / T;
    
    if (s_accel_per_second > EXPECTED_ACCEL_IN_ONE_SECOND)
        violated = true;
    else if (d_accel_per_second > EXPECTED_ACCEL_IN_ONE_SECOND)
        violated = true;
    
    return violated;
    
}

