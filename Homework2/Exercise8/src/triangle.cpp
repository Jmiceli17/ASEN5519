/*
 * triangle.cpp
 *
 *  Created on: Oct 11, 2020
 *      Author: Joe
 */

#include "triangle.hpp"

Triangle::Triangle(){
	// initialize with zeros
	vertex_vector = {{0,0},{0,0},{0,0}};
	std::cout << "[WARNING] EMPTY TRIANGLE INITIALIZED" << std::endl;

}

Triangle::Triangle(std::vector<std::vector<double>> vert_vect)
{

	vertex_vector = vert_vect;

	// Get the ranges of the obstacle for discretization
	double xvals[] = {vertex_vector[0][0], vertex_vector[1][0], vertex_vector[2][0]};
	double yvals[] = {vertex_vector[0][1], vertex_vector[1][1], vertex_vector[2][1]};

	max_x_val = *std::max_element(xvals, xvals+3);
	min_x_val = *std::min_element(xvals, xvals+3);
	max_y_val = *std::min_element(yvals, yvals+3);
	min_y_val = *std::min_element(yvals, yvals+3);
}

Triangle::~Triangle()
{

}

// Primitives
// Left side (connecting v3 to v0)
bool Triangle::P0(std::vector<double> pt_to_check)
{
	// vi+1.y - vi.y / vi+1.x - vi.x
	double v1_y = vertex_vector[1][1];
	double v1_x = vertex_vector[1][0];
	double v0_y = vertex_vector[0][1];
	double v0_x = vertex_vector[0][0];
	double slope = (v1_y-v0_y)/(v1_x-v0_x);
	// Point slope form, point must be above this primitive
	return ((pt_to_check[1] - v1_y) >= slope*(pt_to_check[0]-v1_x));


}

// Bottom side (connecting v0 to v1)
bool Triangle::P1(std::vector<double> pt_to_check)
{
	// vi+1.y - vi.y / vi+1.x - vi.x
	double v2_y = vertex_vector[2][1];
	double v2_x = vertex_vector[2][0];
	double v1_y = vertex_vector[1][1];
	double v1_x = vertex_vector[1][0];
	double slope = (v2_y-v1_y)/(v2_x-v1_x);
	// Point slope form, point must be below this primitive
	return ((pt_to_check[1] - v2_y) <= slope*(pt_to_check[0]-v2_x));

}

// Right side (connecting v1 to v2)
bool Triangle::P2(std::vector<double> pt_to_check)
{
	// vi+1.y - vi.y / vi+1.x - vi.x
	double v0_y = vertex_vector[0][1];
	double v0_x = vertex_vector[0][0];
	double v2_y = vertex_vector[2][1];
	double v2_x = vertex_vector[2][0];
	double slope = (v0_y-v2_y)/(v0_x-v2_x);
	// Point slope form, point must be below this primitive
	return ((pt_to_check[1] - v0_y) <= slope*(pt_to_check[0]-v0_x));


}

bool Triangle::InCollision(std::vector<double> pt_to_check)
{

	bool pt_collides = false;

	if (P0(pt_to_check) && P1(pt_to_check) && P2(pt_to_check))
	{
		pt_collides = true;
	}
	return pt_collides;

}

