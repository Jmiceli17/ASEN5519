/*
 * GenerateGraph.cpp
 *
 *  Created on: Nov 1, 2020
 *      Author: Joe
 */


#include "GenerateGraph.hpp"

graph* GenerateGraph(int num_nodes, double r_neighborhood, std::vector<RectangleObs> obs_vec, double X_MIN, double X_MAX, double Y_MIN, double Y_MAX)
{
	graph *p_Graph = new graph();
	int num_generated_nodes = 0;

	std::cout<< "GENERATING NODES" << std::endl;

 	// ==== Generate the set of nodes ====
	while (num_generated_nodes < num_nodes)
	{
		bool point_is_valid = true;
		node *n = new node();

		// Generate node by sampling a random number within the range of the space
		// rand()/RAND_MAX returns number between 0 and 1;
		//n->position[0] = (X_MAX - X_MIN) * ( (double)rand() / (double)RAND_MAX ) + X_MIN;
		//n->position[1] = (Y_MAX - Y_MIN) * ( (double)rand() / (double)RAND_MAX ) + Y_MIN;

		n->mp_position->m_Xpos = (X_MAX - X_MIN) * ( (double)rand() / (double)RAND_MAX ) + X_MIN;
		n->mp_position->m_Ypos = (Y_MAX - Y_MIN) * ( (double)rand() / (double)RAND_MAX ) + Y_MIN;

		//std::cout<< "TOTAL # OBSTACLES TO CHECK: " << obs_vec.size() << std::endl;

		//std::cout<< "CHECKING X VAL: " << n->mp_position->m_Xpos << std::endl;

		// Check if the new point is in collision
		for (int o = 0; o < int(obs_vec.size()); o++)
		{
			// If the new point is in collision with any obstacle,
			// we can't add it to the set of nodes on the graph
			if(obs_vec[o].InCollision( {n->mp_position->m_Xpos, n->mp_position->m_Ypos} ))
			{
				point_is_valid = false;
			}

		}

		// If the new node doesn't collide with any obstacles, add it to the set of nodes
		if (point_is_valid)
		{
			p_Graph->SetOfNodes.push_back(n);
		}

		// Increment the number of generated nodes
		num_generated_nodes++;

	} // End while




	std::cout<< "GENERATING EDGES IN LOCAL NEIGHBORHOODS" << std::endl;

	// ==== Generate the set of edges ====
	// Generate neighbors for each node
	for (int n = 0; n < int(p_Graph->SetOfNodes.size()); n++)
	{
		// initialize the list of neighbors
//		node *new_node = new node();
//		std::vector<node*> neighbors = {new_node};

		//std::vector<node*> neighbors = {p_Graph->SetOfNodes.at(n)};	// made this a member variable


		for (int m = 0; m < int(p_Graph->SetOfNodes.size()); m++)
		{


			// If the distance to node m is greater than 0 (i.e. m != n) AND distance is within the neighborhood radius
			if (p_Graph->SetOfNodes[n]->distance(p_Graph->SetOfNodes[m]) > 0 && p_Graph->SetOfNodes[n]->distance(p_Graph->SetOfNodes[m]) < r_neighborhood)
			{
				//std::cout<<"Distance to neighbor m: " << p_Graph->SetOfNodes[n]->distance(p_Graph->SetOfNodes[m])<<std::endl;

				// Add this node to the neighbors of node n
				p_Graph->SetOfNodes[n]->mp_localNeighbors.push_back(p_Graph->SetOfNodes[m]);
			}
		}

		//std::cout<<"Number neighbors to check: "<< p_Graph->SetOfNodes[n]->mp_localNeighbors.size() <<std::endl;

		// Attempt to connect edges between node n and all its neighbors
		//for (int m = 0; m < neighbors.size(); m++)
		for (int m = 0; m < int(p_Graph->SetOfNodes[n]->mp_localNeighbors.size()); m++)

		{
			// If the edge between node n and node m is collision free, add it to the set of edges
			if (PathCollisionFree(p_Graph->SetOfNodes[n], p_Graph->SetOfNodes[n]->mp_localNeighbors[m], obs_vec))
			{
				// Edge connecting node n to neighbor m
				edge *e = new edge(p_Graph->SetOfNodes[n], p_Graph->SetOfNodes[n]->mp_localNeighbors[m]);

				p_Graph->SetOfEdges.push_back(e);
			}
		}

	}	// end for


	return p_Graph;

}




