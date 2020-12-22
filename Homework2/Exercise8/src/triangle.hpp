/*
 * triangle.hpp
 *
 *  Created on: Oct 11, 2020
 *      Author: Joe
 */

#ifndef TRIANGLE_HPP_
#define TRIANGLE_HPP_
#include <algorithm>
#include <iostream>

#include "shape.hpp"


using namespace std;

class Triangle
{
		private:
		// Primitives, these will be used to check for collision
		bool P0(std::vector<double> pt_to_check);
		bool P1(std::vector<double> pt_to_check);
		bool P2(std::vector<double> pt_to_check);

		public:
			std::vector<std::vector<double>> vertex_vector;

			// bounds of obstacle
			double max_x_val;
			double min_x_val;
			double max_y_val;
			double min_y_val;

			// Constructors
			Triangle();
			Triangle(std::vector<std::vector<double>> vert_vect);

			// Destructor
			~Triangle();


			// Check if a given point is in or on the obstacle
			bool InCollision(std::vector<double> pt_to_check);


};



#endif /* TRIANGLE_HPP_ */
