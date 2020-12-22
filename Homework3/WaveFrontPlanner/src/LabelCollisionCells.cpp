/*
 * LabelCollisionCells.cpp
 *
 *  Created on: Oct 26, 2020
 *      Author: Joe
 */




#include "LabelCollisionCells.hpp"

void LabelCollisionCells(std::vector<WaveFrontCell> &grid_cells, std::vector<RectangleObs> obs_vec)
{

	// Loop over all cells in the workspace
	for (int c = 0; c<int(grid_cells.size()); c++ )
	{
		// Check every obstacle if this cell is in collision
		for (int o = 0; o < int(obs_vec.size()); o++)
		{
			if (obs_vec[o].InCollision(grid_cells[c].position))
			{
//				std::cout<<"FOUND A CELL" <<std::endl;
//				std::cout<<grid_cells[c].position[0] << " " << grid_cells[c].position[1] << std::endl;

				// Label this cell with 1
				grid_cells[c].value = 1;

				// We don't need to check the remaining obstacles
				break;
			}
		}

	}

} /* end LabelCollisionCells */
