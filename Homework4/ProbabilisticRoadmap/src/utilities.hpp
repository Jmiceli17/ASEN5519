/*
 * utilities.hpp
 *
 *  Created on: Oct 19, 2020
 *      Author: Joe
 */

#ifndef UTILITIES_HPP_
#define UTILITIES_HPP_

#include <iostream>
#include <vector>
#include <math.h>

using namespace std;


// Function for subtracting two 2-D vectors
std::vector<double> Subtract2DVectors(std::vector<double> v1, std::vector<double> v2);

// Function for adding two 2-D vectors
std::vector<double> Add2DVectors(std::vector<double> v1, std::vector<double> v2);

// Function for computing the norm of a vector
double VectorNorm(std::vector<double> v);


// Function for computing a unit vector of a provided vector
std::vector<double> UnitVector(std::vector<double> v);

#endif /* UTILITIES_HPP_ */
