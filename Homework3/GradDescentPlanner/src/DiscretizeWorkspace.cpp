/*
 * DiscretizeWorkspace.cpp
 *
 *  Created on: Oct 21, 2020
 *      Author: Joe
 */


#include "DiscretizeWorkspace.hpp"

std::vector<std::vector<double>> DiscretizeWorkspace(double MAX_X, double MIN_X, double MAX_Y, double MIN_Y, double grid_spacing)
{
	std::vector<std::vector<double>> points_vec;

	// Number of X and Y values to be generated
	int num_X_vals = (MAX_X - MIN_X)/grid_spacing;
	int num_Y_vals = (MAX_Y - MIN_Y)/grid_spacing;

	// Arrays to hold all the grid points from discretization
	double W_xvals[num_X_vals];
	double W_yvals[num_Y_vals];

	// Generate X values
	int xidx = 0;
	for (double i = MIN_X; i < MAX_X; i += grid_spacing)
	{
		W_xvals[xidx] = i;
		xidx++;
	}

	// Generate Y values
	int yidx = 0;
	for (double i = MIN_Y; i < MAX_Y; i += grid_spacing)
	{
		W_yvals[yidx] = i;
		yidx++;
	}

	// Get vector of all points
	for (int i=0; i<num_Y_vals ; i++)
	{
		for (int j=0; j<num_X_vals; j++)
		{
			// Single x,y point
			std::vector<double> point = {double(W_xvals[j]), double(W_yvals[i])};

			// Add the point to the collection of points
			points_vec.push_back(point);
		}
	}

	return points_vec;

}

