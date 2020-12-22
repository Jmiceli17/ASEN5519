/*
 * shape.hpp
 *
 *  Created on: Oct 11, 2020
 *      Author: Joe
 */

#ifndef SHAPE_HPP_
#define SHAPE_HPP_

#include <iostream>
#include <vector>

#include "utilities.hpp"

using namespace std;

// NOT CURRENTLY USED
class Shape{

	// Vector of vectors [[x,y],[x,y],...] containing the vertices of the shape
	public:
		// std::vector<std::vector<double>> vertices;

		// Method to check if a provided point is in collision with the shape
		virtual bool InCollision(std::vector<double> pt_to_check);

		//virtual ~Shape(){};


};




#endif /* SHAPE_HPP_ */
