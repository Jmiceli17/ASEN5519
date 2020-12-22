/*
 * WaveFrontPath.cpp
 *
 *  Created on: Oct 28, 2020
 *      Author: Joe
 */


#include "WaveFrontPath.hpp"

std::vector<WaveFrontCell> WaveFrontPath(std::vector<WaveFrontCell> &grid_cells, std::vector<double> q_start)
{
	std::vector<WaveFrontCell> path;
	//WaveFrontCell start_cell = grid_cells[0];
	WaveFrontCell start_cell;

	// Identify the starting cell
	for (int c = 0; c<int(grid_cells.size()); c++ )
	{
		//WaveFrontCell *start_cell = new WaveFrontCell;
		if (grid_cells[c].position == q_start)
		{
			std::cout<<"FOUND THE START CELL" <<std::endl;
			std::cout<<grid_cells[c].position[0] << " " << grid_cells[c].position[1] << std::endl;
			start_cell = grid_cells[c];
			//break;	// We found the start
		}

	}

	WaveFrontCell curr_cell = start_cell;

	int curr_val = curr_cell.value;

	while (curr_val > 2)
	{
		// Add this cell to the path
		path.push_back(curr_cell);

		std::cout<< "Num neighbors "<< curr_cell.neighbors.size() << std::endl;

		for (int n = 0; n<int(curr_cell.neighbors.size()); n++ )
		{
			std::cout<<" Neighbor # " << n << std::endl;
			std::cout<<" Current Cell "<<curr_cell.position[0] << " " << curr_cell.position[1] << std::endl;
			std::cout<<" Neighbor Value " << curr_cell.neighbors[n].value<< std::endl;
			// We found a neighbor with a val
			if (curr_cell.neighbors[n].value == curr_val - 1)
			{
				// update the current cell
				curr_cell = curr_cell.neighbors[n];

				// Update the current value
				curr_val = curr_cell.neighbors[n].value;

				// Stop searching neighbors
				//break;

			}
			else
			{
				// std::cout<<"ERROR NO VALID NEIGHBORS"<<std::endl;
				//break;
			}
		}
	}

	return path;
}
