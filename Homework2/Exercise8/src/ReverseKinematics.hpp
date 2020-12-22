/*
 * ReverseKinematics.hpp
 *
 *  Created on: Oct 11, 2020
 *      Author: Joe
 */

#ifndef REVERSEKINEMATICS_HPP_
#define REVERSEKINEMATICS_HPP_
#include <cmath>
#include <iostream>
#include <vector>

using namespace std;

// Function to compute the angles required to achieve a certain desired (x,y) position for a 2D
// 2-link manipulator with link lengths of l1, l2

std::vector<double> ReverseKinematics_positive(std::vector<double> desired_pt, double l1, double l2);

std::vector<double> ReverseKinematics_negative(std::vector<double> desired_pt, double l1, double l2);


#endif /* REVERSEKINEMATICS_HPP_ */
