/*
 * 	LabelGoalCell.hpp
 *
 *  Created on: Oct 21, 2020
 *      Author: Joe
 *
 *  Description: Function for identifying start and goal cells in a wavefront planner
 *
 *  Input:
 *  	grid_cells - pointer to vector of WaveFrontCell cells that make up the
 *  				 discretized workspace
 *
 *  Output:
 *  	grid_cells - Vector of WaveFrontCells that describe the discretized space,
 *  				 each cell is centered at a single point in the discretized
 *  				 workspace, Goal cell has been labeled
 *
 */

#ifndef SRC_LABELGOALCELL_HPP_
#define SRC_LABELGOALCELL_HPP_


#include<iostream>
#include<vector>

#include "WaveFrontCell.hpp"
#include "SetNeighborValues.hpp"

using namespace std;


void LabelGoalCell(std::vector<WaveFrontCell> &grid_cells, std::vector<double> q_goal);





#endif /* SRC_LABELGOALCELL_HPP_ */
