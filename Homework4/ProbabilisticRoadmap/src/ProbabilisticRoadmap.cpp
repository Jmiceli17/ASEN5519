//============================================================================
// Name        : ProbabilisticRoadmap.cpp
// Author      : Joe Miceli
// Version     :
// Copyright   : Your copyright notice
// Description : Probabilistic roadmap for motion planning in 2 dimensions
//				 completed as part of Homework 4 of the course ASEN 5519
// 				 Algorithmic Motion Planning at CU Boulder
//============================================================================


#include <chrono>
#include <iostream>
#include <fstream>
#include <vector>


#include "edge.hpp"
#include "graph.hpp"
#include "node.hpp"

#include "GenerateGraph.hpp"
#include "PRMQuery.hpp"

//#include "matplotlibcpp.h"




using namespace std;

using namespace std::chrono;

//namespace plt = matplotlibcpp;


int main() {

	// ====== Start the clock ======
	auto start = high_resolution_clock::now();


	// ====== Homework 4 Problem 2a ======
//	int num_cells = 200;
//	double radius = 0.5;
//		// execution time: NULL, program exits during generation of local neighborhoods

//	int num_cells = 200;
//	double radius = 1.0;
//		// execution time: 649628 microseconds,[NUMBER OF A* LOOPS] 181

	int num_cells = 200;
	double radius = 1.5;
		// execution time: 1458165 microseconds, [NUMBER OF A* LOOPS] 198

//	int num_cells = 200;
//	double radius = 2;
//		// execution time: 3301110 microseconds


//	int num_cells = 500;
//	double radius = 0.5;
//		// execution time: 1277269 microseconds, A* algorithm exited after only 14 iterations


//	int num_cells = 500;
//	double radius = 1;
//		// execution time: 4484433 microseconds, [NUMBER OF A* LOOPS] 493

//	int num_cells = 500;
//	double radius = 1.5;
//		// execution time: 12270976 microseconds, [NUMBER OF A* LOOPS] 493

//	int num_cells = 500;
//	double radius = 2.0;
//		// execution time: 28055939 microseconds, [NUMBER OF A* LOOPS] 493



	//int num_cells = 800;
	//int num_cells = 2000;
	//double radius = 1;
	//double radius = 0.5;
	//double radius = 1.5;



	// Planning Problem from Homework 3 Problem 2a
	// ======= Workspace Config ========
	// Ranges and grid spacing
	double MAX_X =  11.0;
	double MIN_X = -1.0;
	double MAX_Y =  3.0;
	double MIN_Y = -3.0;

	// ==== Planning Config ====
	// TODO: change this back to 10,0
	std::vector<double> q_goal = {10,0};		// Goal position
	std::vector<double> q_start = {0,0};		// Starting position
	std::vector<double> q_curr = q_start;		// The robot's current position
	std::vector<double> q_prev = q_curr;		// The robot's previous position (for tracking total distance traveled)
	std::vector<double> gradient;				// The gradient of the potential function

	// ======= Obstacles =======
	RectangleObs obs1({ {3.5, 0.5}, {4.5,0.5}, {4.5,1.5}, {3.5,1.5} });
	RectangleObs obs2({ {6.5, -1.5}, {7.5, -1.5}, {7.5, -0.5}, {6.5, -0.5} });
	std::vector<RectangleObs> obstacle_vector({obs1, obs2});


	node *n_init = new node(q_start);
	node *n_goal = new node(q_goal);


	graph *p_Graph = GenerateGraph(num_cells, radius, obstacle_vector, MIN_X, MAX_X, MIN_Y, MAX_Y);

	//std::vector<edge*> PRMPath = PRMQuery(p_Graph, obstacle_vector, n_init, n_goal, radius);
	std::vector<node*> PROCESSED = PRMQuery(p_Graph, obstacle_vector, n_init, n_goal, radius);

	// DEBUG:
	//cout << "num nodes in graph " << p_Graph->SetOfNodes.size() << endl;
	//cout << "num edges in graph " << p_Graph->SetOfEdges.size() << endl;

	// DEBUG
//	// Check if there are any nodes without neighbors
//	for (int j= 0; j<int(p_Graph->SetOfNodes.size()); j++)
//	{
//		if (p_Graph->SetOfNodes[j]->mp_localNeighbors.size()==0)
//		{
//			std::cout << "NODE HAS NO NEIGHBORS!" << std::endl;
//			std::cout<< p_Graph->SetOfNodes[j]->getX() << "," << p_Graph->SetOfNodes[j]->getY() << std::endl;
//		}
//	}




	// cout << "num of edges in final path " << PRMPath.size() << endl;


	// Log generated nodes
	std::ofstream LOG_FILE("PRM.csv");
	LOG_FILE << "X" << "," << "Y\n";


	for (int i = 0; i<int(p_Graph->SetOfNodes.size()); i++)
	{
		LOG_FILE<< p_Graph->SetOfNodes.at(i)->getX() << "," <<  p_Graph->SetOfNodes.at(i)->getY() << "\n";

	}

	LOG_FILE.close();



	// Log processed nodes
	std::ofstream PROCESSED_LOG_FILE("PRMProcessedNodes.csv");
	PROCESSED_LOG_FILE << "X" << "," << "Y\n";

	for (int i = 0; i<int(PROCESSED.size()); i++)
	{
		PROCESSED_LOG_FILE<< PROCESSED.at(i)->getX() << "," <<  PROCESSED.at(i)->getY() << "\n";

	}

	PROCESSED_LOG_FILE.close();

	// Get ending timepoint
	auto stop = high_resolution_clock::now();

	// Get duration. To cast it to proper unit
    // use duration cast method
    auto duration = duration_cast<microseconds>(stop - start);

    cout << "[NUMBER OF NODES GENERATED (BEFORE VALIDITY CHECK)] " << num_cells << endl;
    cout << "[RADIUS OF LOCAL NEIGHBORHOOD] " << radius <<endl;
    cout << "[TOTAL EXECUTION TIME] " << duration.count() << " microseconds" << endl;

	return 0;
}
