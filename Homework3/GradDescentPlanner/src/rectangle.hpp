/*
 * rectangle.hpp
 *
 *  Created on: Oct 11, 2020
 *      Author: Joe
 */

#ifndef RECTANGLE_HPP_
#define RECTANGLE_HPP_
#include <algorithm>
#include <iostream>

#include "shape.hpp"
#include "GetDistance.hpp"
#include "utilities.hpp"

using namespace std;

class RectangleObs
{

	private:
		// Primitives, these will be used to check for collision
		bool P0(std::vector<double> pt_to_check);	// left vertical
		bool P1(std::vector<double> pt_to_check);
		bool P2(std::vector<double> pt_to_check);	// right vertical
		bool P3(std::vector<double> pt_to_check);

		// Primitives for generating boundary points
		double p0(double y);	// left vertical
		double p1(double x);
		double p2(double y);	// right vertical
		double p3(double x);
		void GenerateBoundaryPoints();	// Used for GetDistanceToObstacle()


	public:
		std::vector<std::vector<double>> vertex_vector;

		// bounds of obstacle
		double max_x_val;
		double min_x_val;
		double max_y_val;
		double min_y_val;

		// Vector of (x,y) points on the boundary of the obstacle
		std::vector<std::vector<double>> boundary_points;

		// Point on obstacle closest to the robot
		std::vector<double> closest_point;
		// Distance from robot to the closest point
		double min_distance;

		// Threshold to consider this obstacles gradient
		double q_star;


		RectangleObs();
		RectangleObs(std::vector<std::vector<double>> vert_vect);

		// Destructor
		~RectangleObs();


		// Check if a given point is in or on the obstacle
		bool InCollision(std::vector<double> pt_to_check);

		// Get the shortest distance to the obstacle
		double GetDistanceToObstacle(std::vector<double> pt_to_check);

		// Get the individual repulsive gradient for this obstacle
		std::vector<double> GetObstacleGradient(std::vector<double> pt_to_check);
};




#endif /* RECTANGLE_HPP_ */
