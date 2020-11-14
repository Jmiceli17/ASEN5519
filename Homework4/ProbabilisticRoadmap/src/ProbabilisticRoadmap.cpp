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


//	// ============ Homework 4 Problem 2a ============
//	int num_cells = 200;
//	double radius = 0.5;
		// execution time: NULL, program exits during generation of local neighborhoods

	int num_cells = 200;
	double radius = 1.0;
		// execution time: 649628 microseconds,[NUMBER OF A* LOOPS] 181

//	int num_cells = 200;
//	double radius = 1.5;
//		// execution time: 1458165 microseconds, [NUMBER OF A* LOOPS] 198

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


//	// Planning Problem from Homework 3 Problem 2a
	// ======= Workspace Config ========
	// Ranges and grid spacing
	double MAX_X =  11.0;
	double MIN_X = -1.0;
	double MAX_Y =  3.0;
	double MIN_Y = -3.0;

	// ==== Planning Config ====
	std::vector<double> q_goal = {10,0};		// Goal position
	std::vector<double> q_start = {0,0};		// Starting position

	// ======= Obstacles =======
	RectangleObs obs1({ {3.5, 0.5}, {4.5,0.5}, {4.5,1.5}, {3.5,1.5} });
	RectangleObs obs2({ {6.5, -1.5}, {7.5, -1.5}, {7.5, -0.5}, {6.5, -0.5} });
	std::vector<RectangleObs> obstacle_vector({obs1, obs2});
	// =================================================





	// ============= Homework 4 Problem 2b =============
//	int num_cells = 200;
//	double radius = 2.0;
//		// execution time: 127367463 microseconds, [NUMBER OF A* LOOPS] 166

//	int num_cells = 200;
//	double radius = 1.0;
//		// execution time: 127367463 microseconds, [NUMBER OF A* LOOPS] 166
//
//	int num_cells = 500;
//	double radius = 1.0;
//		// execution time: 127367463 microseconds, [NUMBER OF A* LOOPS] 166
//
//	int num_cells = 500;
//	double radius = 2.0;
//		// execution time: 127367463 microseconds, [NUMBER OF A* LOOPS] 166
//
//
	// Planning Problem from Homework 1 Problem 7
	// ======= Workspace 1 Config ========
//	// Ranges and grid spacing
//	double MAX_X =  13.0;
//	double MIN_X = -1.0;
//	double MAX_Y =  13.0;
//	double MIN_Y = -1.0;
//
//
//
//	// ==== Planning Config ====
//	std::vector<double> q_goal = {{10,10}};		// Goal position
//	std::vector<double> q_start = {{0,0}};		// Starting position
//
//	// ======= Obstacles =======
//	RectangleObs obs1({ {1, 1}, {2, 1}, {2, 5}, {1, 5} });
//	RectangleObs obs2({ {3, 4}, {4, 4}, {4, 12}, {3, 12} });
//	RectangleObs obs3({ {3, 12}, {12, 12}, {12, 13}, {3, 13} });
//	RectangleObs obs4({ {12, 5}, {13, 5}, {13, 13}, {12, 13} });
//	RectangleObs obs5({ {6, 5}, {12, 5}, {12, 6}, {6, 6} });
//	std::vector<RectangleObs> obstacle_vector({obs1, obs2, obs3, obs4, obs5});



	//	// ======= Workspace 2 Config ========
//		// Ranges and grid spacing
//	double MAX_X =  36.0;
//	double MIN_X = -6.0;
//	double MAX_Y =  6.0;
//	double MIN_Y = -6.0;
//
//	// ==== Planning Config ====
//	std::vector<double> q_goal = {{35,0}};		// Goal position
//	std::vector<double> q_start = {{0,0}};		// Starting position
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
//	std::vector<RectangleObs> obstacle_vector({obs1, obs2, obs3, obs4, obs5, obs6, obs7, obs8, obs9});
	// =================================================





	// ======= Create Graph =======
	node *n_init = new node(q_start);
	node *n_goal = new node(q_goal);

	graph *p_Graph = GenerateGraph(num_cells, radius, obstacle_vector, MIN_X, MAX_X, MIN_Y, MAX_Y);

	// ======= Query the Graph ======
	std::vector<edge*> PRMPath = PRMQuery(p_Graph, obstacle_vector, n_init, n_goal, radius);
	//std::vector<node*> PROCESSED = PRMQuery(p_Graph, obstacle_vector, n_init, n_goal, radius);

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



	// ======= LOG GENERATED NODES ======
	std::ofstream LOG_FILE("PRM.csv");
	LOG_FILE << "X" << "," << "Y\n";


	for (int i = 0; i<int(p_Graph->SetOfNodes.size()); i++)
	{
		LOG_FILE<< p_Graph->SetOfNodes.at(i)->getX() << "," <<  p_Graph->SetOfNodes.at(i)->getY() << "\n";

	}

	LOG_FILE.close();



//	// ====== LOG PROCESSED NODES ======
//	std::ofstream PROCESSED_LOG_FILE("PRMProcessedNodes.csv");
//	PROCESSED_LOG_FILE << "X" << "," << "Y\n";
//
//	for (int i = 0; i<int(PROCESSED.size()); i++)
//	{
//		PROCESSED_LOG_FILE<< PROCESSED.at(i)->getX() << "," <<  PROCESSED.at(i)->getY() << "\n";
//
//	}
//
//	PROCESSED_LOG_FILE.close();


//	// ====== LOG PATH ======
//	std::ofstream PATH_LOG_FILE("PRMPath.csv");
//	PATH_LOG_FILE << "X" << "," << "Y\n";
//
//	for (int i = 0; i<int(PRMPath.size()); i++)
//	{
//		PATH_LOG_FILE<< PRMPath.at(i)->m_node1->getX() << "," <<  PRMPath.at(i)->m_node1->getY() << "\n";
//		//PATH_LOG_FILE<< PRMPath.at(i)->m_node2->getX() << "," <<  PRMPath.at(i)->m_node2->getY() << "\n";
//
//
//	}
//
//	PATH_LOG_FILE.close();

	// Get ending time
	auto stop = high_resolution_clock::now();

	// Get duration
    auto duration = duration_cast<microseconds>(stop - start);

    cout << "[NUMBER OF NODES GENERATED (BEFORE VALIDITY CHECK)] " << num_cells << endl;
    cout << "[RADIUS OF LOCAL NEIGHBORHOOD] " << radius <<endl;
    cout << "[TOTAL EXECUTION TIME] " << duration.count() << " microseconds" << endl;

	return 0;
}
