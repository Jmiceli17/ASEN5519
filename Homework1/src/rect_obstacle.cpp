/*
 * rect_obstacle.cpp
 *
 *  Created on: Sep 18, 2020
 *      Author: Joe
 */

#include "rect_obstacle.hpp"

rect_obstacle::rect_obstacle()
{
	v1 = {0,0};
	v2 = {0,0};
	v3 = {0,0};
	v4 = {0,0};
}

rect_obstacle::rect_obstacle(std::vector<double> vertex1, std::vector<double> vertex2, std::vector<double> vertex3, std::vector<double> vertex4)
{
	v1 = vertex1;
	v2 = vertex2;
	v3 = vertex3;
	v4 = vertex4;
}

rect_obstacle::~rect_obstacle()
{
	//
}




