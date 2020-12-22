/*
 * rectangle.cpp
 *
 *  Created on: Oct 11, 2020
 *      Author: Joe
 */


#include "rectangle.hpp"

using namespace std;

RectangleObs::RectangleObs(){
	// initialize with zeros
	vertex_vector = {{0,0},{0,0},{0,0},{0,0}};
	std::cout << "[WARNING] EMPTY RECTANGLE INITIALIZED" << std::endl;

}

RectangleObs::RectangleObs(std::vector<std::vector<double>> vert_vect)
{

	vertex_vector = vert_vect;

	// Get the ranges of the obstacle for discretization
	double xvals[] = {vertex_vector[0][0], vertex_vector[1][0], vertex_vector[2][0], vertex_vector[3][0]};
	double yvals[] = {vertex_vector[0][1], vertex_vector[1][1], vertex_vector[2][1], vertex_vector[3][1]};

	max_x_val = *std::max_element(xvals, xvals+4);
	min_x_val = *std::min_element(xvals, xvals+4);
	max_y_val = *std::min_element(yvals, yvals+4);
	min_y_val = *std::min_element(yvals, yvals+4);




}

RectangleObs::~RectangleObs()
{

}

// Primitives
// Left side (connecting v3 to v0)
bool RectangleObs::P0(std::vector<double> pt_to_check)
{
	//TODO: add try catch for checking slopes of vertical lines
	// The slope is undefined so this primitive is vertical
	return (pt_to_check[0] >= vertex_vector[0][0]);

//	try
//	{
//		// Point-slope form
//		double slope = (vertex_vector[3][1]-vertex_vector[0][1])/(vertex_vector[3][0]-vertex_vector[0][0]);
//		return ((pt_to_check[1]-vertex_vector[1][1]) >= slope*(pt_to_check[0]-vertex_vector[0][0]));
//
//	}
//	catch(const std::exception& e)
//	{
//		// The slope is undefined so this primitive is vertical
//		return (pt_to_check[0] >= vertex_vector[0][0]);
//
//	}

}

// Bottom side (connecting v0 to v1)
bool RectangleObs::P1(std::vector<double> pt_to_check)
{
	// Point-slope form
	double slope = (vertex_vector[1][1]-vertex_vector[0][1])/(vertex_vector[1][0]-vertex_vector[0][0]);
	return ((pt_to_check[1]-vertex_vector[1][1]) >= slope*(pt_to_check[0]-vertex_vector[0][0]));
}

// Right side (connecting v1 to v2)
bool RectangleObs::P2(std::vector<double> pt_to_check)
{
	return (pt_to_check[0] <= vertex_vector[2][0]);
}
// Top side (connecting v2 to v3)
bool RectangleObs::P3(std::vector<double> pt_to_check)
{
	// Point-slope form
	double slope = (vertex_vector[3][1]-vertex_vector[2][1])/(vertex_vector[3][0]-vertex_vector[2][0]);
	return ((pt_to_check[1]-vertex_vector[3][1]) <= slope*(pt_to_check[0]-vertex_vector[3][0]));
}

bool RectangleObs::InCollision(std::vector<double> pt_to_check)
{

	bool pt_collides = false;

	if (P0(pt_to_check) && P1(pt_to_check) && P2(pt_to_check) && P3(pt_to_check))
	{
		pt_collides = true;
	}
	return pt_collides;

}

