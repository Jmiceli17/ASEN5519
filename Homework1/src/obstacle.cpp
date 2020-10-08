/*
 * obstacle.cpp
 *
 *  Created on: Sep 21, 2020
 *      Author: Joe
 */


#include "obstacle.hpp"


AlignedRectangle::AlignedRectangle() {


    min_x = 0;
    max_x = 0;
    min_y = 0;
    max_y = 0;
    v4 = {{min_x, max_y}};
    v3 = {{max_x, max_y}};
    v2 = {{max_x, min_y}};
    v1 = {{min_x, min_y}};
	//vertex_vector = {{0, 0}};
    obstacle_traced = false;
    vertex_vector = {{v1}, {v2}, {v3}, {v4}};

}

void AlignedRectangle::DefineBoundary(double x_left, double x_right, double y_bottom, double y_top)
//void AlignedRectangle::DefineBoundary(std::vector<std::vector<double>> vertex_vector)
{
	min_x = x_left;
	max_x = x_right;
	min_y = y_bottom;
	max_y = y_top;

	v4 = {{min_x, max_y}};
    v3 = {{max_x, max_y}};
    v2 = {{max_x, min_y}};
    v1 = {{min_x, min_y}};
    vertex_vector = {{v1}, {v2}, {v3}, {v4}};
}

AlignedRectangle::~AlignedRectangle()
{

}

// Primitives 
bool AlignedRectangle::P0(std::vector<double> pt_to_check)
{
	//TODO: add try catch for checking slopes of vertical lines
	return (pt_to_check[0] >= vertex_vector[0][0]);
}

bool AlignedRectangle::P1(std::vector<double> pt_to_check)
{
	// Point-slope form
	double slope = (vertex_vector[1][1]-vertex_vector[0][1])/(vertex_vector[1][0]-vertex_vector[0][0]);
	return ((pt_to_check[1]-vertex_vector[1][1]) >= slope*(pt_to_check[0]-vertex_vector[0][0]));
}

bool AlignedRectangle::P2(std::vector<double> pt_to_check)
{
	return (pt_to_check[0] <= vertex_vector[2][0]);
}

bool AlignedRectangle::P3(std::vector<double> pt_to_check)
{
	// Point-slope form
	double slope = (vertex_vector[3][1]-vertex_vector[2][1])/(vertex_vector[3][0]-vertex_vector[2][0]);
	return ((pt_to_check[1]-vertex_vector[3][1]) <= slope*(pt_to_check[0]-vertex_vector[3][0]));
}

//// Primitives for identifying boundary
//bool AlignedRectangle::P0_boundary(std::vector<double> pt_to_check)
//{
//	return (abs(pt_to_check[0] - vertex_vector[0][0]) <= 1e-5);
//}
//
//bool AlignedRectangle::P1_boundary(std::vector<double> pt_to_check)
//{
//	// Point-slope form
//	double slope = (vertex_vector[1][1]-vertex_vector[0][1])/(vertex_vector[1][0]-vertex_vector[0][0]);
//	return (abs((pt_to_check[1]-vertex_vector[1][1]) - slope*(pt_to_check[0]-vertex_vector[0][0])) <= 1e-5 );
//}
//
//bool AlignedRectangle::P2_boundary(std::vector<double> pt_to_check)
//{
//	return (abs(pt_to_check[0] - vertex_vector[2][0]) <= 1e-5);
//}
//
//bool AlignedRectangle::P3_boundary(std::vector<double> pt_to_check)
//{
//	// Point-slope form
//	double slope = (vertex_vector[3][1]-vertex_vector[2][1])/(vertex_vector[3][0]-vertex_vector[2][0]);
//	return (abs((pt_to_check[1]-vertex_vector[3][1]) - slope*(pt_to_check[0]-vertex_vector[3][0])) <= 1e-5);
//}


// Check if given point within the boundary of the rectangle
bool AlignedRectangle::InCollision(std::vector<double> pt_to_check)
{
	bool pt_collides = false;

	if (P0(pt_to_check) && P1(pt_to_check) && P2(pt_to_check) && P3(pt_to_check))
	{
		pt_collides = true;
	}
	return pt_collides;
}

// Check if a given point is on the boundary
//bool AlignedRectangle::OnBoundary(std::vector<double> pt_to_check)
//{
//
//	bool on_boundary = false;
//	if (P0_boundary(pt_to_check) && P1_boundary(pt_to_check) && P2_boundary(pt_to_check) && P3_boundary(pt_to_check))
//	{
//		on_boundary = true;
//	}
//	return on_boundary;
//}
//void AlignedRectangle::ApplyBoundaryOffset(double offset)
//        {
//
//        }









