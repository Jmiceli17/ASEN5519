//============================================================================
// Name        : ProbabilisticRoadmap.cpp
// Author      : Joe Miceli
// Version     :
// Copyright   : Your copyright notice
// Description : Probabilistic roadmap for motion planning in 2 dimensions
//				 completed as part of Homework 4 of the course ASEN 5519
// 				 Algorithmic Motion Planning at CU Boulder
//============================================================================


#include <iostream>

#include "edge.hpp"
#include "graph.hpp"
#include "node.hpp"

#include "GenerateGraph.hpp"
#include "PRMQuery.hpp"




using namespace std;

int main() {

	// ====== Homework 4 Problem 2a ======
	int num_cells = 200;
	double radius = 1;




	// Planning Problem from Homework 3 Problem 2a
	// ======= Workspace Config ========
	// Ranges and grid spacing
	double MAX_X =  10;
	double MIN_X = -10;
	double MAX_Y =  5;
	double MIN_Y = -5;

	// ==== Planning Config ====
	std::vector<double> q_goal = {10,0};		// Goal position
	std::vector<double> q_start = {0,0};		// Starting position
	std::vector<double> q_curr = q_start;		// The robot's current position
	std::vector<double> q_prev = q_curr;		// The robot's previous position (for tracking total distance traveled)
	std::vector<double> gradient;				// The gradient of the potential function
	double total_distance_traveled = 0;			// Total distance traveled by the robot

	// ======= Obstacles =======
	RectangleObs obs1({ {3.5, 0.5}, {4.5,0.5}, {4.5,1.5}, {3.5,1.5} });
	RectangleObs obs2({ {6.5, -1.5}, {7.5, -1.5}, {7.5, -0.5}, {6.5, -0.5} });
	std::vector<RectangleObs> obstacle_vector({obs1, obs2});


	node *n_init = new node(q_start);
	node *n_goal = new node(q_goal);


	graph *p_Graph = GenerateGraph(num_cells, radius, obstacle_vector, MIN_X, MAX_X, MIN_Y, MAX_Y);

	cout << "num nodes in graph " << p_Graph->SetOfNodes.size() << endl;
	cout << "num edges in graph " << p_Graph->SetOfEdges.size() << endl;

	std::vector<edge*> PRMPath = PRMQuery(p_Graph, obstacle_vector, n_init, n_goal, radius);


	cout << "num nodes in graph " << p_Graph->SetOfNodes.size() << endl;
	cout << "num edges in graph " << p_Graph->SetOfEdges.size() << endl;

	cout << "num of edges in final path " << PRMPath.size() << endl;



	cout << "!!!Hello World!!!" << endl; // prints !!!Hello World!!!
	return 0;
}
