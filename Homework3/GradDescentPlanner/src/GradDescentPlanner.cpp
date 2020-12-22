//============================================================================
// Name        : GradDescentPlanner.cpp
// Author      : Joe Miceli
// Version     :
// Copyright   :
// Description : Implementation of a gradient descent motion planning
// 				 algorithm.
//============================================================================

#include <iostream>
#include <fstream>


#include "rectangle.hpp"
#include "configuration.hpp"
#include "DiscretizeWorkspace.hpp"
#include "gradient.hpp"
#include "GenerateVectorField.hpp"
#include "TakeStep.hpp"


using namespace std;



int main() {

	// ======= Gradient Descent Config =======
	params_t PLANNER_PARAMS;

	// Homework 3 Problem 2a
//	// ======= Workspace Config ========
//	// Ranges and grid spacing
//	double MAX_X =  10;
//	double MIN_X = -10;
//	double MAX_Y =  5;
//	double MIN_Y = -5;
//	double grid_spacing = 0.25;



//	// ==== Planning Config ====
//	std::vector<double> q_goal = {{10,0}};		// Goal position
//	std::vector<double> q_start = {{0,0}};		// Starting position
//	std::vector<double> q_curr = q_start;		// The robot's current position
//	std::vector<double> q_prev = q_curr;		// The robot's previous position (for tracking total distance traveled)
//	std::vector<double> gradient;				// The gradient of the potential function
//	double total_distance_traveled = 0;			// Total distance traveled by the robot
//
//
//
//	// ======= Obstacles =======
//	RectangleObs obs1({ {3.5, 0.5}, {4.5,0.5}, {4.5,1.5}, {3.5,1.5} });
//	RectangleObs obs2({ {6.5, -1.5}, {7.5, -1.5}, {7.5, -0.5}, {6.5, -0.5} });
//	std::vector<RectangleObs> obstacle_vector({obs1, obs2});

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
	std::vector<double> q_curr = q_start;		// The robot's current position
	std::vector<double> q_prev = q_curr;		// The robot's previous position (for tracking total distance traveled)
	std::vector<double> gradient;				// The gradient of the potential function
	double total_distance_traveled = 0;			// Total distance traveled by the robot



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

	obs1.q_star = 0.1;
	obs2.q_star = 4.5;
	obs3.q_star = 0.1;
	obs4.q_star = 2;
	obs5.q_star = 4;
	obs6.q_star = 4;
	obs7.q_star = 5;
	obs8.q_star = 3.5;
	obs9.q_star = 1;

	std::vector<RectangleObs> obstacle_vector({obs1, obs2, obs3, obs4, obs5, obs6, obs7, obs8, obs9});

	// ======= Generate the Vector Field =======
	// Discretize the workspace
	std::vector<std::vector<double>> workspace_points =	DiscretizeWorkspace(MAX_X, MIN_X, MAX_Y, MIN_Y, grid_spacing);
	GenerateVectorField(workspace_points, obstacle_vector, q_goal, PLANNER_PARAMS);


	// ======= Logging ========
	std::ofstream LOG_FILE("GradientDescent.csv");
	LOG_FILE << "X" << "," << "Y" << "," << "U" << "," << "V\n";	// Log file for writing x,y position and u,v gradient at each position


	// ======= Path Planning =======

	// Initialize gradient
	gradient = compute_gradient(q_start, q_goal, PLANNER_PARAMS, obstacle_vector);

	std::cout<< "*** PERFORMING GRADIENT DESCENT ***" << std::endl;

	// Run main gradient descent loop
	while(VectorNorm(gradient) > PLANNER_PARAMS.EPSILON)
	{
		// Compute the gradient for the current position
		gradient = compute_gradient(q_curr, q_goal, PLANNER_PARAMS, obstacle_vector);

		// Take a step in the direction of the negative gradient
		TakeStep(q_curr, gradient, PLANNER_PARAMS.ALPHA);

		// Log the current position and gradient
		LOG_FILE<< q_curr[0] << "," << q_curr[1] << ","<< gradient[0] << "," << gradient[1] << "\n";


		// Increment the total distance traveled
		total_distance_traveled += GetDistance(q_curr, q_prev);

		// Update the robot's memory
		q_prev = q_curr;

	}

	// Close the file stream
	LOG_FILE.close();

	// The robot has made it to the goal
	cout << "!!!GOAL ACHIEVED!!!" << endl;
	std::cout<<"TOTAL DISTANCE TRAVELED = "<< total_distance_traveled << std::endl;

	return 0;
}
