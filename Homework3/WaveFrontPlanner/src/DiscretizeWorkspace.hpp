/*
 * DiscretizeWorkspace.hpp
 *
 *  Created on: Oct 21, 2020
 *      Author: Joe
 *
 *  Description: Function for discretizing a 2D workspace and creating wavefront cells
 *
 *  Input:
 *  	MAX_X - Maximum x value of the workspace
 *  	MIN_X - Minimum x value of the workspace
 *  	MAX_Y - Maximum y value of the workspace
 *  	MIN_Y - Minimum y value of the workspace
 *  	grid_spaceing - resolution of discretization
 *
 *  Output:
 *  	grid_cells - Vector of WaveFrontCells that describe the discretized space,
 *  				 each cell is centered at a single point in the discretized
 *  				 workspace
 *
 */

#ifndef DISCRETIZEWORKSPACE_HPP_
#define DISCRETIZEWORKSPACE_HPP_

#include<iostream>
#include<vector>

#include "WaveFrontCell.hpp"

using namespace std;

// Function discretizes workspace based on provided workspace bounds and
std::vector<WaveFrontCell> DiscretizeWorkspace(double MAX_X, double MIN_X, double MAX_Y, double MIN_Y, double grid_spacing);




#endif /* DISCRETIZEWORKSPACE_HPP_ */
