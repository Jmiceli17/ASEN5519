/*
 * WaveFrontCell.cpp
 *
 *  Created on: Oct 25, 2020
 *      Author: Joe
 */


#include "WaveFrontCell.hpp"


WaveFrontCell::WaveFrontCell(std::vector<double> pos)
{
	position = pos;				// X,Y Position of the center point of this cell
	value = 0;
	neighbors = {};
}
WaveFrontCell::WaveFrontCell()
{
	position = {};				// X,Y Position of the center point of this cell
	value = 0;
	neighbors = {};
}
WaveFrontCell::~WaveFrontCell()
{

}

void WaveFrontCell::SetNeighborValues()
{
	//std::cout<< "Num Neighbors: " << this->neighbors.size()<< std::endl;
	std::cout<< "Neighbor's neighbor position: " << this->neighbors[1].neighbors.size()<< std::endl;
	for (int n = 0; n<int(this->neighbors.size()); n++ )
	{
		std::cout<< "Neighbor pos: " << this->neighbors[n].position[0]<< "," << this->neighbors[n].position[1] << std::endl;

		// Only Set values for unexplored neighbors
		if (neighbors[n].value == 0)
		{
			std::cout<<"SETTING NEIGHBOR VALUE TO: " << int(this->value + 1) << std::endl;
			neighbors[n].value = this->value + 1;

		}

		this->neighbors[n].SetNeighborValues();


	}
}

void WaveFrontCell::SetNeighbors(double grid_spacing)
{

	if (int(this->neighbors.size())>0)
	{

	}
	else
		{
			std::vector<double> new_pos1 = {this->position[0]+grid_spacing, this->position[1]};
			std::vector<double> new_pos2 = {this->position[0]-grid_spacing, this->position[1]};
			std::vector<double> new_pos3 = {this->position[0], this->position[1]+grid_spacing};
			std::vector<double> new_pos4 = {this->position[0], this->position[1]-grid_spacing};

			WaveFrontCell new1(new_pos1);
			WaveFrontCell new2(new_pos2);
			WaveFrontCell new3(new_pos3);
			WaveFrontCell new4(new_pos4);

			this->neighbors.push_back(new1);
			this->neighbors.push_back(new2);
			this->neighbors.push_back(new3);
			this->neighbors.push_back(new4);


		}


}

int WaveFrontCell::GetMinValNeighbors()
{
	// initialize minimum value
	int min_val = 100000000;

	// Check the values of all neighbors to identify the minimum
	for (int n = 0; n<int(neighbors.size()); n++ )
	{
		if (neighbors[n].value < min_val)
		{
			// Ignore 0 (uninitialized cells) and 1 (obstacles)
			if (neighbors[n].value != 0)
			{
				if (neighbors[n].value != 1)
				{
					min_val = neighbors[n].value;
				}
			}
		}
	}





	return min_val;
}
