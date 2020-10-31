/*
 * DiscretizeWorkspace.cpp
 *
 *  Created on: Oct 21, 2020
 *      Author: Joe
 */


#include "DiscretizeWorkspace.hpp"

std::vector<WaveFrontCell> DiscretizeWorkspace(double MAX_X, double MIN_X, double MAX_Y, double MIN_Y, double grid_spacing)
//double DiscretizeWorkspace(double MAX_X, double MIN_X, double MAX_Y, double MIN_Y, double grid_spacing)
{
//	// The vector containing all cells in the space
//		// NOTE: the index of this vector does not correspond to anything
//
//
//		// Number of X and Y values to be generated
//		int num_X_vals = (MAX_X - MIN_X)/grid_spacing;
//		int num_Y_vals = (MAX_Y - MIN_Y)/grid_spacing;
//
//		// Arrays to hold all the grid points from discretization
//		double W_xvals[num_X_vals];
//		double W_yvals[num_Y_vals];
//		double grid_cells[num_X_vals][num_Y_vals];
//
//		for (int y=0; y<num_Y_vals; y++)
//		{
//			for (int x=0; x<num_X_vals; x++)
//			{
//				grid_cells[x][y] = 0;
//			}
//		}
//
//		return grid_cells;






	// The vector containing all cells in the space
	// NOTE: the index of this vector does not correspond to anything
	std::vector<WaveFrontCell> grid_cells;

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

	// Generate cells along the border of the space
//	for (int x=0; x<num_X_vals; x++)
//	{
//		// Hold Y at max value
//		WaveFrontCell cell({double(W_xvals[x]), double(W_yvals[num_Y_vals-1])});
//		// Give these cells a value of 1 (they're essentially an obstacle)
//		cell.value = 1;
//		// hold Y at min value
//		WaveFrontCell cell2({double(W_xvals[x]), double(W_yvals[0])});
//		// Give these cells a value of 1 (they're essentially an obstacle)
//		cell2.value = 1;
//		// TODO: neighbors?
//		cell.neighbors.push_back(cell2);
//		cell2.neighbors.push_back(cell);
//
//		// Add them to the vector of cells
//		grid_cells.push_back(cell);
//		grid_cells.push_back(cell2);
//	}
//	for (int y=0; y<num_Y_vals; y++)
//	{
//		// Hold X at max value
//		WaveFrontCell cell({double(W_xvals[num_X_vals - 1]), double(W_yvals[y])});
//		// Give these cells a value of 1 (they're essentially an obstacle)
//		cell.value = 1;
//
//		// hold Y at min value
//		WaveFrontCell cell2({double(W_xvals[y]), double(W_yvals[y])});
//		// Give these cells a value of 1 (they're essentially an obstacle)
//		cell2.value = 1;
//		// TODO: neighbors?
//
//		cell.neighbors.push_back(cell2);
//		cell2.neighbors.push_back(cell);
//
//		// Add them to the vector of cells
//		grid_cells.push_back(cell);
//		grid_cells.push_back(cell2);
//	}



	// Generate remaining grid cells
	// Note: we're ignoring the boundary values of x and y in order to create the cell neighbors
//	for (int i=1; i<num_Y_vals -1 ; i++)
//	{
//		for (int j=1; j<num_X_vals - 1; j++)
//		{

	for (int i=1; i<num_Y_vals; i++)
	{
		for (int j=1; j<num_X_vals; j++)
		{
			// Cell neighbors
//			WaveFrontCell n1({double(W_xvals[j-1]), double(W_yvals[i])});
//			WaveFrontCell n2({double(W_xvals[j]), double(W_yvals[i-1])});
//			WaveFrontCell n3({double(W_xvals[j+1]), double(W_yvals[i])});
//			WaveFrontCell n4({double(W_xvals[j]), double(W_yvals[i+1])});



			// Create a cell with the current x,y position
			WaveFrontCell cell({double(W_xvals[j]), double(W_yvals[i])});
//			cell.neighbors.push_back(n1);
//			cell.neighbors.push_back(n2);
//			cell.neighbors.push_back(n3);
//			cell.neighbors.push_back(n4);

			cell.SetNeighbors(grid_spacing);


			// Add the point to the collection of points
			grid_cells.push_back(cell);
		}
	}

	return grid_cells;

}

