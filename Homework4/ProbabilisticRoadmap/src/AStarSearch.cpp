/*
 * AStarSearch.cpp
 *
 *  Created on: Nov 4, 2020
 *      Author: Joe
 */

#ifndef ASTARSEARCH_CPP_
#define ASTARSEARCH_CPP_

#include "AStarSearch.hpp"



#include <iostream>
#include <fstream>

std::vector<edge*> AStarSearch(node* n_init, node* n_goal)
//std::vector<node*> AStarSearch(node* n_init, node* n_goal)

{
	int LOOP_ITERATIONS = 0;
	bool USE_DIJKSTRA = false;
	//bool USE_DIJKSTRA = true;

	// initialize the open list
	std::vector<node*> p_openList;
	std::vector<node *> p_processedNodes;

	p_openList.push_back(n_init);


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

		p_processedNodes.push_back(current_node);

		//DEBUG:
//		if (current_node == n_goal)
//		{
//			std::cout<<"===GOAL HAS BEEN SELECTED FROM OPEN LIST==="<< std::endl;
//		}

		// Expand from the current node
		for (int ni = 0; ni<int(current_node->mp_localNeighbors.size()); ni++)
		{

//			// DEBUG: check if this neighbor is the goal
//			if(current_node->mp_localNeighbors[ni]->m_node2 == n_goal)
//			{
//				std::cout<< "==== NODE2 IS THE GOAL ====" << std::endl;
//				std::cout<< "GOAL EXPLORED? " << current_node->mp_localNeighbors[ni]->m_node2-> m_nodeExplored << std::endl;
//
//			}
//			else if (current_node->mp_localNeighbors[ni]->m_node1 == n_goal)
//			{
//				std::cout<< "==== NODE1 IS THE GOAL ====" << std::endl;
//				std::cout<< "GOAL EXPLORED? " << current_node->mp_localNeighbors[ni]->m_node1-> m_nodeExplored << std::endl;
//			}


			//std::cout<<"LENGTH OF EDGE TO NEIGHBOR: " << current_node->mp_localNeighbors[ni]->getEdgeLength() << std::endl;
			double potential_g = current_node->m_distFromStart + current_node->mp_localNeighbors[ni]->getEdgeLength();


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
//					// DEBUG:
//					if (current_node->mp_localNeighbors[ni]->m_node2 == n_goal)
//					{
//						std::cout<<"GOAL IS IN QUEUE" <<std::endl;
//						std::cout<<"GOAL DIST FROM START: " << current_node->mp_localNeighbors[ni]->m_node2->m_distFromStart << std::endl;
//					}

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
				edge *e = new edge(current_node->mp_localNeighbors[ni]->m_node2, current_node->mp_localNeighbors[ni]->m_node2->mp_parent);
				current_node->mp_localNeighbors[ni]->m_node2->mp_localNeighbors.push_back(e);

				// Set the heuristic for this node
				current_node->mp_localNeighbors[ni]->m_node2->m_heuristic = current_node->mp_localNeighbors[ni]->m_node2->distance(n_goal);

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

				// Update the heuristic of this node
				current_node->mp_localNeighbors[ni]->m_node2->m_heuristic = current_node->mp_localNeighbors[ni]->m_node2->distance(n_goal);

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
			}

		} // End for


		// DEBUG
		//std::cout<<"POS OF CURRENT: " << current_node->getX() << "," << current_node->getY() <<std::endl;
		// How far is node from the goal
		//std::cout<< "DIST FROM GOAL: " << current_node->distance(n_goal) <<std::endl;

	} // End while


	std::cout << "[NUMBER OF A* LOOPS] " << LOOP_ITERATIONS << std::endl;
	std::cout << "[A* USING DIJKSTRA?] " << USE_DIJKSTRA << std::endl;
	// DEBUG:
	if (n_goal->mp_parent == NULL)
	{
		std::cout << "[ERROR][PARENT OF N_GOAL == NULL]" <<std::endl;
		std::cout << "[******** NO PATH FOUND ********]" <<std::endl;
	}
//	else
//	{
//		std::cout << "[PARENT OF N_GOAL NOT NULL!]" <<std::endl;
//
//	}


	// ======== GET THE PATH ========
	std::vector<edge*> path;
	node* current_node = n_goal;
	double TOTAL_PATH_LENGTH = 0;

	// ====== LOG PATH ======
	std::ofstream PATH_LOG_FILE("PRMPath.csv");
	PATH_LOG_FILE << "X" << "," << "Y" << "," << "PATH LENGTH\n";

	while (current_node->mp_parent != NULL)
	{

		PATH_LOG_FILE<< current_node->getX() << "," << current_node->getY() << "," << TOTAL_PATH_LENGTH << "\n";



		// Add the edge between the current node and its parent
		//path.push_back(current_node->getEdge(current_node->mp_parent));

		edge *path_edge = new edge(current_node, current_node->mp_parent);
		// path.push_back(current_node->getEdge(current_node->mp_parent));
		path.push_back(path_edge);

		//TOTAL_PATH_LENGTH += current_node->getEdge(current_node->mp_parent)->m_edgeLength;
		TOTAL_PATH_LENGTH += path_edge->m_edgeLength;

		// Update the current node to its parent
		current_node = current_node->mp_parent;
	}

	// Plot the last node (this should be the start node)
	PATH_LOG_FILE<< current_node->getX() << "," << current_node->getY() << "," << TOTAL_PATH_LENGTH << "\n";


	PATH_LOG_FILE.close();

	std::cout << "[TOTAL PATH LENGTH] " << TOTAL_PATH_LENGTH << std::endl;


	return path;

	//return p_processedNodes;


}


///// The following block of code was the first cut at this function /////

//std::vector<edge*> AStarSearch(node* n_init, node* n_goal, graph* p_G)
////std::vector<edge*> AStarSearch(node* n_init, node* n_goal)
//
//{
//
//	// DEBUG:
//	// IDENTIFY IF THERE ARE ANY NODES WITH THE GOAL AS A NEIGHBOR
//	for (int i = 0; i<int(p_G->SetOfNodes.size()); i++)
//	{
//		node *curr = p_G->SetOfNodes[i];
//		for (int j = 0; j < int(curr->mp_localNeighbors.size()); j++)
//		{
//			if (curr->mp_localNeighbors[j] == n_goal)
//			{
//				std::cout<< "GOAL IS A NEIGHBOR OF SOME CELL" << std::endl;
//			}
//			if (curr->mp_localNeighbors[j] == n_init)
//			{
//				std::cout<< "INIT IS A NEIGHBOR OF SOME CELL" << std::endl;
//			}
//		}
//	}
//
//	// ======== CONDUCT THE SEARCH ========
//	// initialize the open list
//	std::vector<node*> p_openList;
//	std::vector<node *> p_processedNodes;
//	int NUM_ITERATIONS = 0;
//
//	// Set initial heuristic and priority for this node (by default, nodes are
//	// initialized with 0 path length, 999999999 priority, and NULL ptr to parent
//	//n_init->m_heuristic = n_init->distance(n_goal);
//	n_init->m_heuristic = 0;
//	//n_init->m_priority = n_init->m_distFromStart + n_init->m_heuristic;
//	n_init->m_priority = 999999999;
//
//	//std::cout<<"n_init priority: " <<n_init->m_priority<<std::endl;
//
//
//
//	p_openList.push_back(n_init);
//
//	// While the open list is not empty
//	while(!p_openList.empty())
//	{
//		NUM_ITERATIONS++;
//
//		// DEBUG: check if n_goal is in open list
//		for (int i=0; i<int(p_openList.size()); i++)
//		{
//			if (p_openList[i] == n_goal)
//			{
//				std::cout<< "====GOAL IS IN OPEN LIST===="<<std::endl;
//			}
//		}
//
//
//		// sort the open list to set the minimum priority node at the
//		// beginning of the list
//		//std::sort(p_openList.begin(), p_openList.end());
//
//		// Get the node with the lowest priority
//		// this will be the 1st element of the open list set
//		// Each element in p_openList is a node pointer
//		//node *current_node = p_openList.at(0);
//
//
//		// Get the lowest priority node from the list
//		node *current_node = p_openList.at(0);
//		for(int i = 0; i<int(p_openList.size()); i++)
//		{
//			if(current_node->m_priority > p_openList.at(i)->m_priority)
//			{
//				current_node = p_openList.at(i);
//			}
//		}
//
//
//		// Remove this element from the open list set
//		p_openList.erase(std::remove(p_openList.begin(), p_openList.end(), current_node), p_openList.end());
//
//		// Mark the current node as explored and add it to the
//		// list of processed nodes
//		current_node->m_nodeExplored = true;
//
//		// Remove the first element from the open list set
//		//p_openList.erase(p_openList.begin());
//
//		// DEBUG:
//		if(current_node->mp_parent != NULL)
//		{
//			std::cout<<"POS OF PARENT: " << current_node->mp_parent->getX() << "," << current_node->mp_parent->getY() <<std::endl;
//		}
//
//
//		// DEBUG:
//		if (n_goal->m_nodeExplored)
//		{
//			std::cout<< "====FOUND GOAL===="<<std::endl;
//		}
//		//DEBUG:
//		std::cout<<"POS: " << current_node->getX() << "," << current_node->getY() <<std::endl;
//
//		std::cout << "NUM NEIGHBORS: " << int(current_node->mp_localNeighbors.size()) <<std::endl;
//
//		for (int i=0; i<int(current_node->mp_localNeighbors.size()); i++)
//		{
//			if (current_node->mp_localNeighbors[i] == n_goal)
//			{
//				std::cout<< "====GOAL IS A NEIGHBOR OF THIS NODE===="<<std::endl;
//			}
//		}
//
//		// DEBUG:
//		if (int(current_node->mp_localNeighbors.size())==0)
//		{
//			std::cout << "!!!!!!! NODE HAS NO NEIGHBORS !!!!!!!!" <<std::endl;
//			std::cout << current_node->getX() << " " << current_node->getY() <<std::endl;
//		}
//
//
//		// Expand from the current nodes
//		for (int ni = 0; ni < int(current_node->mp_localNeighbors.size()); ni++)
//		{
//			double potential_g = current_node->m_distFromStart + current_node->distance(current_node->mp_localNeighbors[ni]);
//
//
//
//			// DEBUG:
//			if (current_node->mp_localNeighbors[ni] == n_goal)
//			{
//				std::cout<<"==== N_GOAL IS A NEIGHBOR! ====" << std::endl;
//			}
//
//
//			// If the neighbor has been explored, ignore it
//			if (current_node->mp_localNeighbors[ni]->m_nodeExplored)
//			{
//				continue;
//			}
//
//
//			// check if the neighbor has already been added to the queue
//			bool neighbor_in_queue = false;
//			for (int i = 0; i < int(p_openList.size()); i++)
//			{
//				if (current_node->mp_localNeighbors[ni] == p_openList[i])
//				{
//					//std::cout<<"NEIGHBOR IN QUEUE" <<std::endl;
//					neighbor_in_queue = true;
//					break;
//				}
//			}
//
//			// Neighbor is not in the queue yet so initialize it
//			if (!neighbor_in_queue)
//			{
//				//std::cout<<"NEIGHBOR NOT IN QUEUE" <<std::endl;
//
//				// Update parent
//				current_node->mp_localNeighbors[ni]->mp_parent = current_node;
//
//				// Update path length
//				current_node->mp_localNeighbors[ni]->m_distFromStart = potential_g;
//
//				// Update heuristic
//				current_node->mp_localNeighbors[ni]->m_heuristic = current_node->mp_localNeighbors[ni]->distance(n_goal);
//
//				// Update priority
//				current_node->mp_localNeighbors[ni]->m_priority = current_node->mp_localNeighbors[ni]->m_heuristic + current_node->mp_localNeighbors[ni]->m_distFromStart;
//
//				// Add it to the queue
//				p_openList.push_back(current_node->mp_localNeighbors[ni]);
//
//			}
//
//			// Neighbor is in the queue already but check if the path to this node is better than it's current path
//			else if (potential_g < current_node->mp_localNeighbors[ni]->m_distFromStart)
//			{
//				//std::cout<<"UPDATING THE PATH TO THIS NODE" << std::endl;
//				// Update parent
//				current_node->mp_localNeighbors[ni]->mp_parent = current_node;
//				// Update path length
//				current_node->mp_localNeighbors[ni]->m_distFromStart = potential_g;
//				// Update priority
//				current_node->mp_localNeighbors[ni]->m_heuristic = current_node->mp_localNeighbors[ni]->distance(n_goal);
//				// Update priority
//				current_node->mp_localNeighbors[ni]->m_priority = current_node->mp_localNeighbors[ni]->m_heuristic + current_node->mp_localNeighbors[ni]->m_distFromStart;
//
//				//std::cout<<"updated back ptr" << std::endl;
//			}
//
//
//		}	// end for
//
//
//	}// end while
//
//		// ======== GENERATE PATH FROM BACK POINTERS ========
//		// Path is from GOAL to START
//		std::vector<edge*> path;
//		node* current_node = n_goal;
//
////		if (current_node->mp_parent == NULL)
////		{
////			std::cout << "PARENT OF N_GOAL == NULL" <<std::endl;
////		}
//		if (n_goal->mp_parent == NULL)
//		{
//			std::cout << "PARENT OF N_GOAL == NULL" <<std::endl;
//		}
//		else
//		{
//			std::cout << "PARENT OF N_GOAL NOT NULL!" <<std::endl;
//
//		}
//		std::cout << "NUM LOOPS " << NUM_ITERATIONS << std::endl;
//
//
//		// While a parent exists...
//		while (current_node->mp_parent != NULL)
//		{
//			std::cout << "Generating path from back pointers" << std::endl;
//			// Add the edge between the current node and its parent
//			path.push_back(new edge(current_node->mp_parent, current_node));
//
//			// Update the current node to its parent
//			current_node = current_node->mp_parent;
//		}
//
//		return path;
//}
//////////////////////////////////////////////////////////////////////////////////////



#endif /* ASTARSEARCH_CPP_ */
