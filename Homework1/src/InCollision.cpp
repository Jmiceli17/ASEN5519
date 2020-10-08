/*
 * InCollision.cpp
 *
 *  Created on: Sep 18, 2020
 *      Author: Joe
 */

#include "InCollision.hpp"




//bool InCollision(std::vector<double> position, rect_obstacle O, double obstacle_offset)
//{
//
//	bool point_in_col = false; // flag indicating if point is in collision with obstacle
//
//	// max and min x values
//	double min_x = std::min({O.v1[0], O.v2[0], O.v3[0], O.v4[0]});
//	double max_x = std::max({O.v1[0], O.v2[0], O.v3[0], O.v4[0]});
//
//	// max and min y values
//	double min_y = std::min({O.v1[1], O.v2[1], O.v3[1], O.v4[1]});
//	double max_y = std::max({O.v1[1], O.v2[1], O.v3[1], O.v4[1]});
//
//	// Apply the offset to the boundary of the obstacle so the robot doesn't actually collide with it
//	min_x -= obstacle_offset;
//	max_x += obstacle_offset;
//
//	min_y -= obstacle_offset;
//	max_y += obstacle_offset;
//
//	// check if point is within the perimeter of the obstacle
//	if ((position[0] > min_x) && (position[0] < max_x) && (position[1] > min_y) && (position[1] < max_y))
//	{
//		point_in_col = true;
//	}
//
//
//	return point_in_col;
//
//
//
//}


bool InCollision(std::vector<double> position, AlignedRectangle  O, double obstacle_offset)
{

	bool point_in_col = false; // flag indicating if point is in collision with obstacle

	// max and min x values
	double min_x = std::min({O.v1[0], O.v2[0], O.v3[0], O.v4[0]});
	double max_x = std::max({O.v1[0], O.v2[0], O.v3[0], O.v4[0]});

	// max and min y values
	double min_y = std::min({O.v1[1], O.v2[1], O.v3[1], O.v4[1]});
	double max_y = std::max({O.v1[1], O.v2[1], O.v3[1], O.v4[1]});

	// Apply the offset to the boundary of the obstacle so the robot doesn't actually collide with it
	min_x -= obstacle_offset;
	max_x += obstacle_offset;

	min_y -= obstacle_offset;
	max_y += obstacle_offset;

	// check if point is within the perimeter of the obstacle
	if ((position[0] > min_x) && (position[0] < max_x) && (position[1] > min_y) && (position[1] < max_y))
	{
		point_in_col = true;
	}


	return point_in_col;



}
