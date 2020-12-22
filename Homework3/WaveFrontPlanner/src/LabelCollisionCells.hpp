/*
 * LabelCollisionCells.hpp
 *
 *  Created on: Oct 26, 2020
 *      Author: Joe
 */

#ifndef SRC_LABELCOLLISIONCELLS_HPP_
#define SRC_LABELCOLLISIONCELLS_HPP_

#include<iostream>
#include<vector>

#include "rectangle.hpp"
#include "WaveFrontCell.hpp"

using namespace std;

void LabelCollisionCells(std::vector<WaveFrontCell> &grid_cells, std::vector<RectangleObs> obs_vec);




#endif /* SRC_LABELCOLLISIONCELLS_HPP_ */
