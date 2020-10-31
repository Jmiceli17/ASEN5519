/*
 * ComputePathLength.hpp
 *
 *  Created on: Oct 30, 2020
 *      Author: Joe
 *
 *  Description: function for returning the path length of a computed motion planning path
 */

#ifndef SRC_COMPUTEPATHLENGTH_HPP_
#define SRC_COMPUTEPATHLENGTH_HPP_

#include <iostream>
#include <vector>

#include "GetDistance.hpp"

using namespace std;

double ComputePathLength(std::vector<std::vector<double>> path);



#endif /* SRC_COMPUTEPATHLENGTH_HPP_ */
