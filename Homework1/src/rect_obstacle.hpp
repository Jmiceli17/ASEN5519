/*
 * rect_obstacle.hpp
 *
 *  Created on: Sep 18, 2020
 *      Author: Joe
 */

#ifndef RECT_OBSTACLE_HPP_
#define RECT_OBSTACLE_HPP_
#include <vector>
#include <cmath>
#include <iostream>
#include <windows.h>


class rect_obstacle
{
public:
	// Member variables to store the locations of the obstacles vertices
	std::vector<double> v1;
	std::vector<double> v2;
	std::vector<double> v3;
	std::vector<double> v4;

	rect_obstacle();
	rect_obstacle(std::vector<double> vertex1, std::vector<double> vertex2, std::vector<double> vertex3, std::vector<double> vertex4);

	// default destructor
	~rect_obstacle();



};



#endif /* RECT_OBSTACLE_HPP_ */
