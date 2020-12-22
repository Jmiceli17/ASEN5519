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
	min_distance = 0;
	max_x_val = 0;
	min_x_val = 0;
	max_y_val = 0;
	min_y_val = 0;
	std::cout << "[WARNING] EMPTY RECTANGLE INITIALIZED" << std::endl;

	q_star = 0.1;

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

	// Generate "vector" of boundary points
	GenerateBoundaryPoints();
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
}
// Primitive for generating boundary points
double RectangleObs::p0(double y)
{
	// Vertical line
	double x = min_x_val;
	return x;
}



// Bottom side (connecting v0 to v1)
bool RectangleObs::P1(std::vector<double> pt_to_check)
{
	// Point-slope form
	double slope = (vertex_vector[1][1]-vertex_vector[0][1])/(vertex_vector[1][0]-vertex_vector[0][0]);
	return ((pt_to_check[1]-vertex_vector[1][1]) >= slope*(pt_to_check[0]-vertex_vector[0][0]));
}
// Primitive for generating boundary points
double RectangleObs::p1(double x)
{
	// compute y using point slope form
	double slope = (vertex_vector[1][1]-vertex_vector[0][1])/(vertex_vector[1][0]-vertex_vector[0][0]);
	double y = slope*(x-vertex_vector[0][0]) + vertex_vector[1][1];
	return y;
}



// Right side (connecting v1 to v2)
bool RectangleObs::P2(std::vector<double> pt_to_check)
{
	return (pt_to_check[0] <= vertex_vector[2][0]);
}
// Primitive for generating boundary points
double RectangleObs::p2(double y)
{
	// Vertical line
	double x = max_x_val;
	return x;
}



// Top side (connecting v2 to v3)
bool RectangleObs::P3(std::vector<double> pt_to_check)
{
	// Point-slope form
	double slope = (vertex_vector[3][1]-vertex_vector[2][1])/(vertex_vector[3][0]-vertex_vector[2][0]);
	return ((pt_to_check[1]-vertex_vector[3][1]) <= slope*(pt_to_check[0]-vertex_vector[3][0]));
}
// Primitive for generating boundary points
double RectangleObs::p3(double x)
{
	// compute y using point slope form
	double slope = (vertex_vector[3][1]-vertex_vector[2][1])/(vertex_vector[3][0]-vertex_vector[2][0]);
	double y = slope*(x-vertex_vector[3][0]) + vertex_vector[3][1];
	return y;
}



void RectangleObs::GenerateBoundaryPoints()
{
	// Using each primitive, generate a discrete vector of points on the boundary of the obstacle
	for(double y = min_y_val; y<max_y_val; y+=0.1)
	{
		// Add each point from this primitive to the vector of boundary points
		boundary_points.push_back({p0(y) , y});
	}
	for(double x = min_x_val; x<max_x_val; x+=0.1)
	{
		// Add each point from this primitive to the vector of boundary points
		boundary_points.push_back({x , p1(x)});
	}
	for(double y = min_y_val; y<max_y_val; y+=0.1)
	{
		// Add each point from this primitive to the vector of boundary points
		boundary_points.push_back({p2(y) , y});
	}
	for(double x = min_x_val; x<max_x_val; x+=0.1)
	{
		// Add each point from this primitive to the vector of boundary points
		boundary_points.push_back({x , p3(x)});
	}

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



double RectangleObs::GetDistanceToObstacle(std::vector<double> pt_to_check)
{
	// Initialize minimum distance and closest point
	min_distance = GetDistance(pt_to_check, boundary_points[0]);
	closest_point = boundary_points[0];


	double dist_to_point;	// Distance from input point to a point on the obstacle

	// Loop over discretized boundary points
	for(int i = 0; i<int(boundary_points.size()); i++ )
	{
		dist_to_point = GetDistance(pt_to_check, boundary_points[i]);

		if(dist_to_point < min_distance)
		{
			// Update the minimum distance and closest point on the obstacle
			min_distance = dist_to_point;
			closest_point = boundary_points[i];
		}
	}
	return min_distance;

}



// Get the individual repulsive gradient for this obstacle
std::vector<double> RectangleObs::GetObstacleGradient(std::vector<double> pt_to_check)
{
	// form is grad = (q - c)/d(q)
	std::vector<double> difference = Subtract2DVectors(pt_to_check, closest_point);
	std::vector<double> gradient(difference.size());

	for (int i = 0; i<int(difference.size()); i++)
	{
		gradient[i] = difference[i]/min_distance;
	}

	return gradient;
}

