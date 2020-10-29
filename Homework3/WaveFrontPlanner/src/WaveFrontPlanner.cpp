//============================================================================
// Name        : WaveFrontPlanner.cpp
// Author      : Joe Miceli
// Version     :
// Copyright   :
//============================================================================

#include <iostream>
#include <fstream>

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
//	// ======= Workspace Config ========
//	// Ranges and grid spacing
//	double MAX_X = 15;
//	double MIN_X = 0;
//	double MAX_Y = 15;
//	double MIN_Y = 0;
//	double grid_spacing = 0.25;
//
//	// ==== Planning Config ====
//	std::vector<double> q_goal = {{10,10}};		// Goal position
//	std::vector<double> q_start = {{0,0}};		// Starting position
//	std::vector<double> q_curr = q_start;		// The robot's current position
//	std::vector<double> q_prev = q_curr;		// The robot's previous position (for tracking total distance traveled)
//	std::vector<double> gradient;				// The gradient of the potential function
//	double total_distance_traveled = 0;			// Total distance traveled by the robot
//
//
//
//	// ======= Obstacles =======
//	// Workspace 1 from Homework 1 Problem 7
//	RectangleObs obs1({ {1, 1}, {2, 1}, {2, 5}, {1, 5} });
//	RectangleObs obs2({ {3, 4}, {4, 4}, {4, 12}, {3, 12} });
//	RectangleObs obs3({ {3, 12}, {12, 12}, {12, 13}, {3, 13} });
//	RectangleObs obs4({ {12, 5}, {13, 5}, {13, 13}, {12, 13} });
//	RectangleObs obs5({ {6, 5}, {12, 5}, {12, 6}, {6, 6} });
//
//	std::vector<RectangleObs> obstacle_vector({obs1, obs2, obs3, obs4, obs5});

	// Homework 3 Problem 2b - W2
	// ======= Workspace Config ========
	// Ranges and grid spacing
	double MAX_X = 40;
	double MIN_X = -10;
	double MAX_Y = 20;
	double MIN_Y = -10;
	double grid_spacing = 0.5;

	// ==== Planning Config ====
	std::vector<double> q_goal = {{35,0}};		// Goal position
	std::vector<double> q_start = {{0,0}};		// Starting position




	// ======= Obstacles =======
	// Workspace 2 from Homework 1 Problem 7
	RectangleObs obs1({ {-6, -6}, {25, -6}, {25, -5}, {-6, -5} });
	RectangleObs obs2({ {-6, 5}, {30, 5}, {30, 6}, {-6, 6} });
	RectangleObs obs3({ {-6, -5}, {-5, -5}, {-5, 5}, {-6, 5} });
	RectangleObs obs4({ {4, -5}, {5, -5}, {5, 1}, {4, 1} });
	RectangleObs obs5({ {9,0}, {10, 0}, {10, 5}, {9, 5} });
	RectangleObs obs6({ {14, -5}, {15, -5}, {15, 1}, {14, 1} });
	RectangleObs obs7({ {19, 0}, {20,0}, {20, 5}, {19, 5} });
	RectangleObs obs8({ {24, -5}, {25, -5}, {25, 1}, {24, 1} });
	RectangleObs obs9({ {29, 0}, {30, 0}, {30, 5}, {29, 5} });



	std::vector<RectangleObs> obstacle_vector({obs1, obs2, obs3, obs4, obs5, obs6, obs7, obs8, obs9});

	// Discretize the workspace
	std::vector<WaveFrontCell> grid_cells =	DiscretizeWorkspace(MAX_X, MIN_X, MAX_Y, MIN_Y, grid_spacing);




	// Label all the cells that are in collision with a value of 1
	LabelCollisionCells(grid_cells, obstacle_vector);

	for (int c = 0; c<int(grid_cells.size()); c++ )
	{
		std::cout<<"NUM NEIGHBORS: " << grid_cells[c].neighbors.size() << std::endl;
	}

	// Label the goal cell with a value of 2
	LabelGoalCell(grid_cells, q_goal);



	//LabelRemainingCells(grid_cells);



	//std::vector<WaveFrontCell> path = WaveFrontPath(grid_cells, q_start);

    //LogWaveFrontPath(path);

	LogWaveFrontPath(grid_cells);
///////////////////////////////////////////
//
//	std::vector<std::vector (double)>explored_grid_cells;
//	std::vector<std::vector (double)>unexplored_grid_cells;
//	unexplored_grid_cells[2][2]=2;
//
//	double y_start, x_start = 2;
//	int i = 0;
//	while (explored_grid_cells.size()<unexplored_grid_cells.size())
//	{
//		double curr_cell_val = unexplored_grid_cells[x_start +i ][y_start + i];
//		x_idxUp = x_start + i;
//		y_idxUp = y_start + i;
//		x_idxDwn = x_start - i;
//		y_idxDwn = y_start - i;
//
//		if (!Obs.InCollision(unexplored_grid_cells[x_idxUp],[y_start]))
//	}




	WaveFrontCell cell({1,1});
	WaveFrontCell cell2({2,2});

	cell.value = 2;
	cell2.value = 1;

	cell.neighbors.push_back(cell2);

	cout<< "cell X: " << cell.position[0] <<  " cell Y: " << cell.position[1] << endl;

	// The robot has made it to the goal
	cout << "!!!GOAL ACHIEVED!!!" << endl;
	// std::cout<<"TOTAL DISTANCE TRAVELED = "<< total_distance_traveled << std::endl;

	return 0;
}
