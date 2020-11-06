/*
 * AStarSearch.cpp
 *
 *  Created on: Nov 4, 2020
 *      Author: Joe
 */

#ifndef ASTARSEARCH_CPP_
#define ASTARSEARCH_CPP_

#include "AStarSearch.hpp"

std::vector<edge*> AStarSearch(node* n_init, node* n_goal)

{
	// ======== CONDUCT THE SEARCH ========
	// initialize the open list
	std::vector<node*> p_openList;
	std::vector<node *> p_processedNodes;

	// Set initial heuristic for this node (by defualt, nodes are
	// initialized with 0 path length, 0 priority, and NULL ptr to parent
	n_init->m_heuristic = n_init->distance(n_goal);

	//std::cout<<"n_init priority: " <<n_init->m_priority<<std::endl;



	p_openList.push_back(n_init);

	// While the open list is not empty
	while(!p_openList.empty())
	{

		//std::cout<<"size of open list " << p_openList.size()<<std::endl;


		// Get the node with the lowest priority
		// this will be the 1st element of the open list set
		// Each element in p_openList is a node pointer
		node *n = *p_openList.begin();

		std::cout<<"n priority: " <<n->m_priority<<std::endl;
		std::cout<<"n heuristic: " <<n->m_heuristic<<std::endl;



//		node *n_end = *p_openList.back();
//		std::cout<<"n_end priority: " <<n_end->m_priority<<std::endl;


		// Mark the current node as explored and add it to the
		// list of processed nodes
		n->m_nodeExplored = true;

		// update this node's heuristic
		n->m_heuristic = n->distance(n_goal);

		// Update this node's distance from start
		//n->m_distFromStart = n->mp_parent->m_distFromStart + n->distance(n->mp_parent);

		// Remove the first element from the open list set
		p_openList.erase(p_openList.begin());

		// Check if this node is the goal node
		// .... //
		if (n==n_goal)
		{
			std::cout<< "FOUND GOAL"<<std::endl;
		}

		// std::cout << "num neighbors: " << int(n->mp_localNeighbors.size()) <<std::endl;

		for (int ni = 0; ni < int(n->mp_localNeighbors.size()); ni++)
		{
			if(!n->mp_localNeighbors[ni]->m_nodeExplored)
			{

				// If g(nbest) + c(nbest, x) < g(x)
				if(n->m_distFromStart + n->distance(n->mp_localNeighbors[ni]) < n->mp_localNeighbors[ni]->m_distFromStart)
				{
					// Update the back pointer and related distance metric
					n->mp_localNeighbors[ni]->mp_parent = n;
					n->mp_localNeighbors[ni]->m_distFromStart = n->m_distFromStart + n->distance(n->mp_localNeighbors[ni]);
					n->mp_localNeighbors[ni]->m_heuristic = n->mp_localNeighbors[ni]->distance(n_goal);
					n->mp_localNeighbors[ni]->m_priority = n->mp_localNeighbors[ni]->m_heuristic + n->mp_localNeighbors[ni]->m_distFromStart;

					std::cout<<"updated back ptr" << std::endl;
				}

				p_openList.push_back(n->mp_localNeighbors[ni]);

			}

		}	// end for




		// sort the open list to set the minimum priority node at the
		// beginning of the list
		std::sort(p_openList.begin(), p_openList.end());


	}// end while



		// ======== GENERATE PATH FROM BACK POINTERS ========
		std::vector<edge*> path;
		node* current_node = n_goal;

		// While a parent exists...
		while (current_node->mp_parent != NULL)
		{
			std::cout << "Generating path from back pointers" << std::endl;
			// Add the edge between the current node and its parent
			path.push_back(new edge(current_node->mp_parent, current_node));

			// Update the current node to its parent
			current_node = current_node->mp_parent;
		}


		return path;


}




#endif /* ASTARSEARCH_CPP_ */
