/*
 * LabelRemainingCells.cpp
 *
 *  Created on: Oct 26, 2020
 *      Author: Joe
 */


#include "LabelRemainingCells.hpp"

void LabelRemainingCells(std::vector<WaveFrontCell> &grid_cells)
{
	// Loop over all cells in the workspace
	for (int c = 0; c<int(grid_cells.size()); c++ )
	{
		// The values of all cells is equal to the 1 + the value of its minimum neighbor (that is not 0 or 1)
		int min_val_of_neighbors = grid_cells[c].GetMinValNeighbors();

		grid_cells[c].value = min_val_of_neighbors + 1;
	}
}
