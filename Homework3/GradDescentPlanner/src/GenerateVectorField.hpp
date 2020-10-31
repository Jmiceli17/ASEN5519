/*
 * GenerateVectorField.hpp
 *
 *  Created on: Oct 21, 2020
 *      Author: Joe
 *
 *  Overview: A method that generates a vector field of the gradient of a potential function
 *  using the points from a discretized workspace and set of obstacles.
 *
 *  Input:
 *  	workspace_points - set of 2D points from discretized workspace
 *  	obstacle_vector - set of rectangular obstacles in the workspace
 *  	q_goal - the workspace goal
 *  	PLANNER_PARAMS - configuration parameters used to compute the gradient
 *
 *  Output:
 *  	A csv file containing the x, y coordinates each point and the u, v components
 *  	of the vector at each point.
 *
 */

#ifndef GENERATEVECTORFIELD_HPP_
#define GENERATEVECTORFIELD_HPP_

#include <fstream>
#include <iostream>
#include <vector>

#include "configuration.hpp"
#include "gradient.hpp"
#include "rectangle.hpp"



using namespace std;

void GenerateVectorField(std::vector<std::vector<double>> workspace_points, std::vector<RectangleObs> obstacle_vector, std::vector<double> q_goal,  params_t PLANNER_PARAMS);



#endif /* GENERATEVECTORFIELD_HPP_ */
