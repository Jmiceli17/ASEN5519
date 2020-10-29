/*
 * WaveFrontCell.hpp
 *
 *  Created on: Oct 25, 2020
 *      Author: Joe
 */

#ifndef WAVEFRONTCELL_HPP_
#define WAVEFRONTCELL_HPP_

#include <iostream>
#include <vector>


class WaveFrontCell
	{

	public:


	std::vector<double> position;				// X,Y Position of the center point of this cell
	int value; 									// The wavefront value of this cell
	std::vector<WaveFrontCell> neighbors;		// The L1 neighbors of this cell (max # of neighbors is 4



	WaveFrontCell(std::vector<double> pos);	// Constructor
	WaveFrontCell();
	~WaveFrontCell();	// Destructor

	void SetNeighborValues();
	void SetNeighbors(double grid_spacing);
	int GetMinValNeighbors();

	};




#endif /* WAVEFRONTCELL_HPP_ */
