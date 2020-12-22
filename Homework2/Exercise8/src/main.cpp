/*
 * main.cpp
 *
 *  Created on: Oct 11, 2020
 *      Author: Joe
 */


#include <iostream>
#include <windows.h>
#include <fstream>
#include <vector>
#include <cmath>


#include "triangle.hpp"
#include "ReverseKinematics.hpp"
#include "rectangle.hpp"


using namespace std;


int main(int argc, char **argv) {


	// ======= Inputs ========
	double link_length_1 = 1;
	double link_length_2 = 1;


	// ======= Logging ========
	std::ofstream c_space_log_file("c_space.csv");
	c_space_log_file << "Theta1" << "," << "Theta2" << "\n";

	std::ofstream w_space_log_file("workspace.csv");
	w_space_log_file << "X" << "," << "Y" << "\n";



	// ======= Workspace ranges ========
	double W_MAX_X =  10;
	double W_MIN_X = -10;
	double W_MAX_Y =  10;
	double W_MIN_Y = -10;

	// Discretized workspace
	double grid_spacing = 0.01;
	int num_W_X_vals = (W_MAX_X - W_MIN_X)/grid_spacing;
	int num_W_Y_vals = (W_MAX_Y - W_MIN_Y)/grid_spacing;

	// Arrays to hold all the grid points from discretization
	double W_xvals[num_W_X_vals];
	double W_yvals[num_W_Y_vals];

	std::cout<< "DISCRETIZING WORKSPACE..." << std::endl;
	int xidx = 0;
	for (double i = W_MIN_X; i < W_MAX_X; i += grid_spacing)
	{
		W_xvals[xidx] = i;
		xidx++;
	}

	int yidx = 0;
	for (double i = W_MIN_Y; i < W_MAX_Y; i += grid_spacing)
	{
		W_yvals[yidx] = i;
		yidx++;
	}


	// ======== Configuration Space Ranges ========
	// Configuration q for 2D 2-link manipulator is 2D where q = (theta1, theta2)
//	double C_MAX_THETA_1 =  M_PI;
//	double C_MIN_THETA_1 = -M_PI;
//	double C_MAX_THETA_2 =  M_PI;
//	double C_MIN_THETA_2 = -M_PI;
//
//	// Discretized C-space using same grid size as workspace
//	int num_C_t1_vals = (C_MAX_THETA_1 - C_MIN_THETA_1)/grid_spacing;
//	int num_C_t2_vals = (C_MAX_THETA_2 - C_MIN_THETA_2)/grid_spacing;
//
//	// Arrays to hold all the grid points
//	double C_t1Vals[num_C_t1_vals];
//	double C_t2Vals[num_C_t2_vals];


	// ======== Obstacles ========
	// NOTE: order of vertices matters here
	// Part a:
	// Triangle obs1({ {-0.25, 0.25}, {0.25, 0.25}, {0, 0.75} });
	// Part b:
	// RectangleObs obs1({ {-0.25, 1.1}, {0.25, 1.1}, {0.25, 2}, {-0.25, 2} });
	// RectangleObs obs2({ {-2, -2}, {2, -2}, {2,-1.8}, {-2, -1.8}  });
	// Part c:
	RectangleObs obs1({ {-0.25, 1.1}, {0.25, 1.1}, {0.25, 2}, {-0.25, 2} });
	RectangleObs obs2({ {-2, -0.5}, {2, -0.5}, {2,-0.3}, {-2, -0.3}  });



	// ======== Collision Search =========
	// Search the discretized space for points that are in collision with an obstacle
	std::vector<std::vector<double>> w_space_collision_set;
	std::cout<< "SEARCHING DISCRETIZED SPACE FOR COLLISIONS..." << std::endl;
	for (int i=0; i<num_W_Y_vals ; i++)
	{
		for (int j=0; j<num_W_X_vals; j++)
		{
			std::vector<double> pt_to_check = {double(W_xvals[j]), double(W_yvals[i])};
			if (obs1.InCollision(pt_to_check))
			{
				// Add this point to the set of collision points
				w_space_collision_set.push_back(pt_to_check);

			}
			if (obs2.InCollision(pt_to_check))
			{
				// Add this point to the set of collision points
				w_space_collision_set.push_back(pt_to_check);

			}
		}

	}


	// ======== Map to C-Space ========
	// Get the c-space coordinates (theta1, theta2) of each collision point (x,y)
	// This can be done using reverse kinematics for the 2-link manipulator
	std::vector<std::vector<double>> c_space_collision_set;
	for (auto point : w_space_collision_set)
	{
		// Get the first pair of potential thetas
		std::vector<double> thetas_pos = ReverseKinematics_positive(point, link_length_1, link_length_2);
		// Add the c_space coordinates to the vector of c_space points
		c_space_collision_set.push_back(thetas_pos);

		// Get the second pair of potential thetas
		std::vector<double> thetas_neg = ReverseKinematics_negative(point, link_length_1, link_length_2);
		// Add the c_space coordinates to the vector of c_space points
		c_space_collision_set.push_back(thetas_neg);
	}



	// ======= Plot Workspace and C Space ========
	for (auto point : w_space_collision_set)
	{
		// write to csv in columns [x, y]
		w_space_log_file << point[0] << "," <<  point[1] << "\n";
	}

	for (auto c_point : c_space_collision_set)
	{
		// write to csv in columns [theta1, theta2]
		c_space_log_file << c_point[0] << "," << c_point[1] << "\n";
	}

	// Close the output streams
	w_space_log_file.close();
	c_space_log_file.close();


	std::cout<< "CONFIGURATION SPACE CONSTRUCTED" << std::endl;
	std::cout<< "EXITING" << std::endl;
	return 0;

}

