/*
 * ComputePathLength.cpp
 *
 *  Created on: Oct 30, 2020
 *      Author: Joe
 */




#include "ComputePathLength.hpp"

double ComputePathLength(std::vector<std::vector<double>> path)
{
	double path_length = 0;

	std::vector<double> prev_pos = path[0];	// previous position point on the path

	for (int c = 0; c<int(path.size()); c++)
	{
		std::vector<double> pos = path[c];	// current position point on the path


		path_length += GetDistance(prev_pos, pos);

		// Update previous position
		prev_pos = pos;

	}

	return path_length;
}
