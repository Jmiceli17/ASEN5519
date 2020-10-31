//============================================================================
// Name        : WaveFrontPlanner.cpp
// Author      : Joe Miceli
// Version     :
// Copyright   :
//============================================================================

#include <iostream>
#include <fstream>
#include <queue>
#include <tuple>

#include "ComputePathLength.hpp"
#include "DiscretizeWorkspace.hpp"
#include "LabelCollisionCells.hpp"
#include "LabelGoalCell.hpp"
#include "LabelRemainingCells.hpp"
#include "LogWaveFrontPath.hpp"
#include "rectangle.hpp"
#include "WaveFrontCell.hpp"
#include "WaveFrontPath.hpp"

using namespace std;

int main() {

	// Homework 3 Problem 2b - W1
	// ======= Workspace Config ========
	// Ranges and grid spacing
	double MAX_X = 15;
	double MIN_X = -5;
	double MAX_Y = 15;
	double MIN_Y = -5;
	double grid_spacing = 0.25;

	// ==== Planning Config ====
	std::vector<double> q_goal = {{10,10}};		// Goal position
	std::vector<double> q_start = {{0,0}};		// Starting position


	// ======= Obstacles =======
	// Workspace 1 from Homework 1 Problem 7
	RectangleObs obs1({ {1, 1}, {2, 1}, {2, 5}, {1, 5} });
	RectangleObs obs2({ {3, 4}, {4, 4}, {4, 12}, {3, 12} });
	RectangleObs obs3({ {3, 12}, {12, 12}, {12, 13}, {3, 13} });
	RectangleObs obs4({ {12, 5}, {13, 5}, {13, 13}, {12, 13} });
	RectangleObs obs5({ {6, 5}, {12, 5}, {12, 6}, {6, 6} });

	std::vector<RectangleObs> obstacle_vector({obs1, obs2, obs3, obs4, obs5});

	// Homework 3 Problem 2b - W2
//	// ======= Workspace Config ========
//	// Ranges and grid spacing
//	double MAX_X = 40;
//	double MIN_X = -10;
//	double MAX_Y = 20;
//	double MIN_Y = -10;
//	double grid_spacing = 0.25;
//
//	// ==== Planning Config ====
//	std::vector<double> q_goal = {{35,0}};		// Goal position
//	std::vector<double> q_start = {{0,0}};		// Starting position
//
//
//
//
//	// ======= Obstacles =======
//	// Workspace 2 from Homework 1 Problem 7
//	RectangleObs obs1({ {-6, -6}, {25, -6}, {25, -5}, {-6, -5} });
//	RectangleObs obs2({ {-6, 5}, {30, 5}, {30, 6}, {-6, 6} });
//	RectangleObs obs3({ {-6, -5}, {-5, -5}, {-5, 5}, {-6, 5} });
//	RectangleObs obs4({ {4, -5}, {5, -5}, {5, 1}, {4, 1} });
//	RectangleObs obs5({ {9,0}, {10, 0}, {10, 5}, {9, 5} });
//	RectangleObs obs6({ {14, -5}, {15, -5}, {15, 1}, {14, 1} });
//	RectangleObs obs7({ {19, 0}, {20,0}, {20, 5}, {19, 5} });
//	RectangleObs obs8({ {24, -5}, {25, -5}, {25, 1}, {24, 1} });
//	RectangleObs obs9({ {29, 0}, {30, 0}, {30, 5}, {29, 5} });
//
//
//
//	std::vector<RectangleObs> obstacle_vector({obs1, obs2, obs3, obs4, obs5, obs6, obs7, obs8, obs9});

	// ====== Discretize the workspace ======
	int num_X_vals = int((MAX_X - MIN_X)/grid_spacing);
	int num_Y_vals = int((MAX_Y - MIN_Y)/grid_spacing);

	// Arrays to hold all the grid points from discretization
	double grid_cells[num_X_vals][num_Y_vals];		// Matrix of the discretized workspace, each cell is centered at x,y and has the value grid_cells[x][y]
	double explored_cells[num_X_vals][num_Y_vals]; 	// bitmap containing 0 or 1 depending on if an index has been explored
	std:: vector<std::vector<int>> queue; 			// index of cells to explore next
	std::queue<std::tuple<int, int, int>> q;  // index of cells to explore next and the value that these are all neighbors of

	for (int y=0; y<num_Y_vals; y++)
	{
		for (int x=0; x<num_X_vals; x++)
		{
			// Initialize cells with 0
			grid_cells[x][y] = 0;
			explored_cells[x][y] = 0;

			// X,Y position in c-space
			double xc = (x*grid_spacing)+MIN_X;
			double yc = (y*grid_spacing)+MIN_Y;

			std::vector<double> pos = {xc, yc};

			// Check all obstacles if this is a collision or not
			for (int o = 0; o < int(obstacle_vector.size()); o++)
			{
				if (obstacle_vector[o].InCollision(pos))
				{
					// This cell is a collision cell
					grid_cells[x][y] = 1;
					explored_cells[x][y] = 1;

				}
			}
		}
	}

	std::cout<<"COLLISION CELLS LABELED!" << std::endl;


	// ====== Label the goal ======
	int xgoal_idx = int((q_goal[0]-MIN_X)/grid_spacing);	// x index of goal cell
	int ygoal_idx = int((q_goal[1]-MIN_Y)/grid_spacing);	// y index of goal cell


	grid_cells[xgoal_idx][ygoal_idx] = 2;
	explored_cells[xgoal_idx][ygoal_idx] = 1;	// mark this index pair as explored

	std::cout<<"GOAL CELL LABELED!" << std::endl;

	// Identify neighbors of goal to initialize the que and the value of the cell that they are neighbors of (i.e. the goal)
	std::tuple<int, int, int> n1 (std::make_tuple(xgoal_idx + 1, ygoal_idx,     grid_cells[xgoal_idx][ygoal_idx]));
	std::tuple<int, int, int> n2 (std::make_tuple(xgoal_idx,     ygoal_idx + 1, grid_cells[xgoal_idx][ygoal_idx]));
	std::tuple<int, int, int> n3 (std::make_tuple(xgoal_idx - 1, ygoal_idx,     grid_cells[xgoal_idx][ygoal_idx]));
	std::tuple<int, int, int> n4 (std::make_tuple(xgoal_idx,     ygoal_idx - 1, grid_cells[xgoal_idx][ygoal_idx]));
	q.push(n1);
	q.push(n2);
	q.push(n3);
	q.push(n4);




	// ====== Label the remaining cells ======
	while(!q.empty())
	{

		// Extract the current indices from the queue
		std::tuple<int,int, int> current_indices_and_value(q.front());

		int xidx = std::get<0>(current_indices_and_value);	// x index
		int yidx = std::get<1>(current_indices_and_value);	// y index
		int cellval = std::get<2>(current_indices_and_value);	// value of the cell this cell was spawned from

		// Remove this element from the queue
		q.pop();

		// If this cell has not been explored
		if (explored_cells[xidx][yidx] == 0)
		{
//
//			std::cout<<"indices: "<< xidx <<" , " << yidx << std::endl;
//			std::cout<<"position: "<< xidx*grid_spacing+MIN_X <<" , " << yidx*grid_spacing+MIN_Y << std::endl;

			grid_cells[xidx][yidx] = cellval+1;
			explored_cells[xidx][yidx] = 1;


			// Add it's neighbors to the queue and the value of the center cell (only if they are in the bounds of the space)
			if( (xidx + 1)<num_X_vals)
			{
				std::tuple<int, int, int> new1 (std::make_tuple(xidx + 1, yidx,    cellval+1));
				q.push(new1);
			}
			if( (yidx + 1) <num_Y_vals )
			{
				std::tuple<int, int, int> new2 (std::make_tuple(xidx,     yidx + 1, cellval+1));
				q.push(new2);
			}
			if( (xidx-1) > 0)
			{
				std::tuple<int, int, int> new3 (std::make_tuple(xidx - 1, yidx,    cellval+1));
				q.push(new3);
			}
			if( (yidx-1) > 0)
			{
				std::tuple<int, int, int> new4 (std::make_tuple(xidx,     yidx - 1, cellval+1));
				q.push(new4);
			}

		}

	}


	std::cout<<"ALL CELLS LABELED!"<< std::endl;

	// ====== Use the labeled cells to compute the path ======

	int xstart_idx = int((q_start[0]-MIN_X)/grid_spacing);	// x index of starting cell
	int ystart_idx = int((q_start[1]-MIN_Y)/grid_spacing);	// y index of starting cell

	int x_curr = xstart_idx;									// Current x index
	int y_curr = ystart_idx;									// Curent y index
	int current_cell_val = grid_cells[xstart_idx][ystart_idx];	// The current value of the cell in the path


	std::vector<std::vector<double>> path;	// vector of x,y position points of the robot

	while (current_cell_val > 2)
	{

		// Check neighbors for the cell containing the current value - 1
		if (grid_cells[x_curr + 1] [y_curr] == (current_cell_val - 1))	// right neighbor
		{
			current_cell_val = grid_cells[x_curr + 1] [y_curr];
			x_curr = x_curr + 1;
//			std::cout<<"N1 selected"<<std::endl;
//			std::cout<<"N1 Curr cell val:" << current_cell_val << std::endl;
		}
		else if(grid_cells[x_curr][y_curr+1] == (current_cell_val - 1))	// top neighbor
		{
			// Update the current value
			current_cell_val = grid_cells[x_curr][y_curr+1];

			// Update the index
			y_curr = y_curr + 1;
//			std::cout<<"N2 selected"<<std::endl;

		}
		else if (grid_cells[x_curr - 1][y_curr] == (current_cell_val - 1))	// left neighbor
		{
			// Update the current value
			current_cell_val = grid_cells[x_curr - 1][y_curr];

			// Update the index
			x_curr = x_curr - 1;
//			std::cout<<"N3 selected"<<std::endl;

		}
		else if(grid_cells[x_curr][y_curr-1] == (current_cell_val - 1))	// bottom neighbor
		{
			// Update the current value
			current_cell_val = grid_cells[x_curr][y_curr-1];

			// Update the index
			y_curr = y_curr - 1;
//			std::cout<<"N4 selected"<<std::endl;

		}
		else
		{
			std::cout << "ERROR: NO VALID NEIGHBOR CELLS" << std::endl;
			return 1;
		}

		// Set the current position from the current indices and add it to the path
		double pos_x = x_curr*grid_spacing+MIN_X;
		double pos_y = y_curr*grid_spacing+MIN_Y;
		path.push_back({pos_x, pos_y});

	}

	std::cout<<"PATH FOUND!"<<std::endl;


	double path_length = ComputePathLength(path);

	LogWaveFrontPath(path);

	std::cout<<"PATH LENGTH: "<< path_length << std::endl;



	return 0;
}
