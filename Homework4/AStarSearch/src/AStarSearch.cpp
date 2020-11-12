//============================================================================
// Name        : AStarSearch.cpp
// Author      : 
// Version     :
// Copyright   : Your copyright notice
// Description : A* search algorithm for a pre-computed graph
//============================================================================

#include <algorithm>
#include <iostream>


#include "edge.hpp"
#include "node.hpp"


using namespace std;


int main() {

	// ======== CONFIGURE THE GRAPH ========
	node *n_start=new node("start");
	n_start->m_heuristic = 0;

	node *n_A = new node("A");
	n_A->m_heuristic = 3;

	node *n_B = new node("B");
	n_B->m_heuristic = 2;

	node *n_C = new node("C");
	n_C->m_heuristic = 3;

	node *n_D = new node("D");
	n_D->m_heuristic = 3;

	node *n_E = new node("E");
	n_E->m_heuristic = 1;

	node *n_F = new node("F");
	n_F->m_heuristic = 3;

	node *n_G = new node("G");
	n_G->m_heuristic = 2;

	node *n_H = new node("H");
	n_H->m_heuristic = 1;

	node *n_I = new node("I");
	n_I->m_heuristic = 2;

	node *n_J = new node("J");
	n_J->m_heuristic = 3;

	node *n_K = new node("K");
	n_K->m_heuristic = 2;

	node *n_L = new node("L");
	n_L->m_heuristic = 3;

	node *n_goal = new node("goal");
	n_goal->m_heuristic = 0;


	// Add neighbors to each cell
	// Note: Commented out edges would make the graph bi-directional
	edge *e1 = new edge(n_start,n_A,1);
	edge *e2 = new edge(n_start,n_B,1);
	edge *e3 = new edge(n_start,n_C,1);
	n_start->mp_localNeighbors.push_back(e1);
	n_start->mp_localNeighbors.push_back(e2);
	n_start->mp_localNeighbors.push_back(e3);

	edge *e4 = new edge(n_A,n_D,1);
	edge *e5 = new edge(n_A,n_E,1);
	edge *e6 = new edge(n_A,n_F,3);
	n_A->mp_localNeighbors.push_back(e4);
	n_A->mp_localNeighbors.push_back(e5);
	n_A->mp_localNeighbors.push_back(e6);

	edge *e7 = new edge(n_B,n_G,4);
	edge *e8 = new edge(n_B,n_H,1);
	edge *e9 = new edge(n_B,n_I,2);
	n_B->mp_localNeighbors.push_back(e7);
	n_B->mp_localNeighbors.push_back(e8);
	n_B->mp_localNeighbors.push_back(e9);

	edge *e10 = new edge(n_C,n_J,1);
	edge *e11 = new edge(n_C,n_K,1);
	edge *e12 = new edge(n_C,n_L,1);
	n_C->mp_localNeighbors.push_back(e10);
	n_C->mp_localNeighbors.push_back(e11);
	n_C->mp_localNeighbors.push_back(e12);

	//edge *e13 = new edge(n_D,n_A,1);
	//n_D->mp_localNeighbors.push_back(e13);

	//edge *e14 = new edge(n_E,n_A,1);
	edge *e15 = new edge(n_E,n_goal,3);
	//n_E->mp_localNeighbors.push_back(e14);
	n_E->mp_localNeighbors.push_back(e15);

	//edge *e16 = new edge(n_F,n_A,3);
	//n_F->mp_localNeighbors.push_back(e16);

	//edge *e17 = new edge(n_G,n_B,4);
	edge *e18 = new edge(n_G,n_goal,3);
	//n_G->mp_localNeighbors.push_back(e17);
	n_G->mp_localNeighbors.push_back(e18);

	//edge *e19 = new edge(n_H,n_B,1);
	//n_H->mp_localNeighbors.push_back(e19);

	//edge *e20 = new edge(n_I,n_B,2);
	edge *e21 = new edge(n_I,n_goal,3);
	//n_I->mp_localNeighbors.push_back(e20);
	n_I->mp_localNeighbors.push_back(e21);

	//edge *e22 = new edge(n_J,n_C,1);
	//n_J->mp_localNeighbors.push_back(e22);

	//edge *e23 = new edge(n_K,n_C,1);
	edge *e24 = new edge(n_K,n_goal,2);
	//n_K->mp_localNeighbors.push_back(e23);
	n_K->mp_localNeighbors.push_back(e24);

	edge *e25 = new edge(n_L,n_C,1);
	n_L->mp_localNeighbors.push_back(e25);

	edge *e26 = new edge(n_goal,n_E,2);
	edge *e27 = new edge(n_goal,n_G,3);
	edge *e28 = new edge(n_goal,n_I,3);
	edge *e29 = new edge(n_goal,n_K,2);
	n_goal->mp_localNeighbors.push_back(e26);
	n_goal->mp_localNeighbors.push_back(e27);
	n_goal->mp_localNeighbors.push_back(e28);
	n_goal->mp_localNeighbors.push_back(e29);


	// ======== CONDUCT THE SEARCH ========

	int LOOP_ITERATIONS = 0;
	//bool USE_DIJKSTRA = false;
	bool USE_DIJKSTRA = true;
	// initialize the open list
	std::vector<node*> p_openList;
	std::vector<node *> p_processedNodes;

	p_openList.push_back(n_start);

	// While the open list is not empty
	while(!p_openList.empty())
	{
		LOOP_ITERATIONS++;

		// Get the lowest priority node from the list
		node *current_node = p_openList.at(0);
		for(int i = 0; i<int(p_openList.size()); i++)
		{
			if(current_node->m_priority > p_openList.at(i)->m_priority)
			{
				current_node = p_openList.at(i);
			}
		}


		// Remove this element from the open list set
		p_openList.erase(std::remove(p_openList.begin(), p_openList.end(), current_node), p_openList.end());

		// Mark the current node as explored (effectively adding it to the list of processed nodes)
		current_node->m_nodeExplored = true;

//		// Check if this node is the goal node
//		if (current_node == n_goal)
//		{
//			std::cout<< "THIS IS THE GOAL NODE" << std::endl;
//		}


		// Expand from the current node
		for (int ni = 0; ni<int(current_node->mp_localNeighbors.size()); ni++)
		{

			double potential_g = current_node->m_distFromStart + current_node->mp_localNeighbors[ni]->m_edgeLength;


			// If the neighbor has been explored, ignore it
			if (current_node->mp_localNeighbors[ni]->m_node2->m_nodeExplored)
			{
				continue;
			}

			// check if the neighbor (the node at the other end of the edge) has already been added to the queue
			bool neighbor_in_queue = false;
			for (int i = 0; i < int(p_openList.size()); i++)
			{
				if (current_node->mp_localNeighbors[ni]->m_node2 == p_openList[i])
				{
					//std::cout<<"NEIGHBOR IN QUEUE" <<std::endl;
					neighbor_in_queue = true;
					break;
				}
			}

			// Neighbor is not in the queue yet so initialize it
			if (!neighbor_in_queue)
			{
				// Update parent
				current_node->mp_localNeighbors[ni]->m_node2->mp_parent = current_node;

				// Create edge between neighbor and its parent
				edge *e = new edge(current_node->mp_localNeighbors[ni]->m_node2, current_node->mp_localNeighbors[ni]->m_node2->mp_parent, current_node->mp_localNeighbors[ni]->m_edgeLength);
				current_node->mp_localNeighbors[ni]->m_node2->mp_localNeighbors.push_back(e);

				// Update distance from start
				current_node->mp_localNeighbors[ni]->m_node2->m_distFromStart = potential_g;

				// Update priority
				if (USE_DIJKSTRA)
				{
					// Only consider path length
					current_node->mp_localNeighbors[ni]->m_node2->m_priority = current_node->mp_localNeighbors[ni]->m_node2->m_distFromStart;
				}
				else
				{
					current_node->mp_localNeighbors[ni]->m_node2->m_priority = current_node->mp_localNeighbors[ni]->m_node2->m_heuristic + current_node->mp_localNeighbors[ni]->m_node2->m_distFromStart;
				}

				// Add it to the queue
				p_openList.push_back(current_node->mp_localNeighbors[ni]->m_node2);
			}


			// Neighbor node has been initialized and added to the graph but check if this parent provides a better path
			else if (potential_g < current_node->mp_localNeighbors[ni]->m_node2->m_distFromStart)
			{
				// If it is better, then mark the current node as the parent of this node and update corresponding metrics
				// Update parent
				current_node->mp_localNeighbors[ni]->m_node2->mp_parent = current_node;

				// Update path length
				current_node->mp_localNeighbors[ni]->m_node2->m_distFromStart = potential_g;

				// Update priority
				if (USE_DIJKSTRA)
				{
					// Only consider path length
					current_node->mp_localNeighbors[ni]->m_node2->m_priority = current_node->mp_localNeighbors[ni]->m_node2->m_distFromStart;
				}
				else
				{
					current_node->mp_localNeighbors[ni]->m_node2->m_priority = current_node->mp_localNeighbors[ni]->m_node2->m_heuristic + current_node->mp_localNeighbors[ni]->m_node2->m_distFromStart;
				}
			}

		} // end for


	}	// End while


	// ======== GET THE PATH ========
	// goal_node.parent -> node.p_parent -> node.p_parent -> ... -> start_node
	// While a parent exists...
	std::vector<edge*> path;
	node* current_node = n_goal;
	double TOTAL_PATH_LENGTH = 0; 	// TODO how to compute this?
	while (current_node->mp_parent != NULL)
	{
		//std::cout << "Generating path from back pointers" << std::endl;

		// Add the edge between the current node and its parent
		//path.push_back(new edge(current_node->mp_parent, current_node));
		path.push_back(current_node->getEdge(current_node->mp_parent));

		TOTAL_PATH_LENGTH += current_node->getEdge(current_node->mp_parent)->m_edgeLength;

		// Update the current node to its parent
		current_node = current_node->mp_parent;
	}

	// Print the path:
	std::cout<<"======= PATH TO GOAL FOUND ======="<<std::endl;
	std::cout<<"=================================="<<std::endl;
	std::cout<<"USING DIJKSTRA?: "<< USE_DIJKSTRA << std::endl;
	std::cout<<"TOTAL LOOP ITERATIONS: "<< LOOP_ITERATIONS << std::endl;
	std::cout<<"TOTAL PATH LENGTH: " << TOTAL_PATH_LENGTH <<std::endl;
	std::cout<<"THE PATH (FROM GOAL TO START) IS:"<< std::endl;
	for (int e=0; e<int(path.size()); e++)
	{
		std::cout<<"NODE ID: " << path[e]->m_node1->m_Id << std::endl;
	}




	return 0;
}
