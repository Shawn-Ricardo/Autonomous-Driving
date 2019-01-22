//
//  helper_functions.hpp
//  Path_Planning
//
//  Created by Shawn Ricardo on 8/25/17.
//  Copyright © 2017 Shawn Ricardo. All rights reserved.
//

#ifndef helper_functions_hpp
#define helper_functions_hpp

#include "Constants.hpp"

 /**
 
  Finite State Machine
 
  The FSM will return 1 of 3 decisions -- keep lane, turn right, or turn left.
  
  FSM() is used for distance around 30 meters

 **/
std::string FSM(State initial_state, std::vector<std::vector<double>> sensors);

 /**
 
  FSM Far
  
  FSM_Far() is used to circumvent the situation of the car being blocked in by vehicles 
  traveling at the same speed in adjacent lanes.
  
  This is used distances greater than 30 meters
  
 **/

std::string FSM_Far(State initial_state, std::vector< std::vector<double> > sensors);

 /**
 
  Multiple Trajectory Generator
  
  MTG() will attempt to generate a jerk-minimizing trajectory to the 'final_state' within the specified time frame 'T'.
  
  If successful, MTG() will populate 'jmt' and return true.
  
 **/
bool MTG(const State_Vectors initial_state, const State_Vectors final_state, Trajectory_Coeffs &jmt, double T);

 /**
  
  
  Jerk Minimizing Trajectory
  
  JMT() return a vector of coefficients that describe a polynomial which minimizes jerk (m/s^3) between 'start' and 'end' within
  the defined timeframe 'T'
  
 **/
std::vector<double> JMT(std::vector<double> start, std::vector <double> end, double T);

 /**
 
  perturb() will take in a target car state, 'target', and populate 'perturbed_goal' with new target states
  that are pulled from a gaussian.
  
  SIGMA_S & SIGMA_D define standard deviations for s, s_dot, s_dot_dot, d, d_dot, and d_dot_dot.
  
  Multiple perturbed_goals are obtained around the timeframe T.
  
 **/

// returns a normally distributed vector of s and d based on a target s and d
void perturb(State_Vectors target, std::vector<State_Vectors> &perturbed_goal, double T);

 /**
 
  differentiate() will return the derivative of a polynomial
 
 **/
std::vector<double> differentiate(std::vector<double> coeffs);

 /**
 
  evaluate_poly() will evalute a polynomial at time T.
  
 **/
double evaluate_poly(std::vector<double> coeffs, double T);


#endif /* helper_functions_hpp */
