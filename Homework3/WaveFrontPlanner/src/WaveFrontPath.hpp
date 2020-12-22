/*
 * WaveFrontPath.hpp
 *
 *  Created on: Oct 28, 2020
 *      Author: Joe
 *
 *  Description: Function for generating a path of WaveFrontCells
 *  			 from a starting point to a goal point.
 */

#ifndef SRC_WAVEFRONTPATH_HPP_
#define SRC_WAVEFRONTPATH_HPP_

#include <iostream>
#include <vector>

#include "WaveFrontCell.hpp"

using namespace std;

std::vector<WaveFrontCell> WaveFrontPath(std::vector<WaveFrontCell> &grid_cells, std::vector<double> q_start);



#endif /* SRC_WAVEFRONTPATH_HPP_ */
