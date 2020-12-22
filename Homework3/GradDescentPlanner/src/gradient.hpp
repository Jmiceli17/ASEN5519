/*
 * gradient.hpp
 *
 *  Created on: Oct 18, 2020
 *      Author: Joe
 */

#ifndef GRADIENT_HPP_
#define GRADIENT_HPP_

#include<vector>

#include "configuration.hpp"
#include "GetDistance.hpp"
#include "rectangle.hpp"
#include "utilities.hpp"

using namespace std;

std::vector<double> compute_gradient(std::vector<double> q, std::vector<double> q_goal, params_t PLANNER_PARAMS, std::vector<RectangleObs> obs_vec);




#endif /* GRADIENT_HPP_ */
