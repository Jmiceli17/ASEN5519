/*
 * PRMQuery.cpp
 *
 *  Created on: Nov 4, 2020
 *      Author: Joe
 */


#include "PRMQuery.hpp"


std::vector<edge*> PRMQuery(graph* p_graph, std::vector<RectangleObs> obs_vec, node* n_init, node* n_goal, double r_neighborhood)

//std::vector<node *> PRMQuery(graph* p_graph, std::vector<RectangleObs> obs_vec, node* n_init, node* n_goal, double r_neighborhood)
{
	std::vector<edge*> PRMPath;
	// Add initial and goal nodes to the graph
	p_graph->SetOfNodes.push_back(n_init);
	p_graph->SetOfNodes.push_back(n_goal);

	// ==== CONNECT Q_INIT TO GRAPH ====
//////////////////////////////////////////
//	// Generate neighbors of Q_init
//	for (int n = 0; n < int(p_graph->SetOfNodes.size()); n++)
//	{
//		// If the distance to node n is greater than 0 (i.e. n != n_init) AND distance is within the neighborhood radius
//		if ((n_init->distance(p_graph->SetOfNodes[n])) > 0 && (n_init->distance(p_graph->SetOfNodes[n]) < r_neighborhood))
//		{
//			// If the path to the neighbor is collision free
//			if (PathCollisionFree(n_init, p_graph->SetOfNodes[n], obs_vec))
//			{
//
//
//				// Add this node to the neighbors of node n_init
//				n_init->mp_localNeighbors.push_back(p_graph->SetOfNodes[n]);
//
//				// Edge connecting node n_init to neighbor n
//				edge *e = new edge(n_init, p_graph->SetOfNodes[n]);
//
//				p_graph->SetOfEdges.push_back(e);
//
//
//				//////// Add n_init to the neighbors of its neighbor
//				p_graph->SetOfNodes[n]->mp_localNeighbors.push_back(n_init);
//				////////
//
//
//			//	std::cout<<"CONNECTED Q_INIT TO GRAPH" << std::endl;
//
//			}
//		}
//	}
//////////////////////////////////////////

	// Get set of potential neighbors for n_init
	std::vector<node*> n_init_potl_neighbors;
	for (int n = 0; n < int(p_graph->SetOfNodes.size()); n++)
	{

		// If the distance to node n is greater than 0 (i.e. n != n_init) AND distance is within the neighborhood radius
		if ( (n_init->distance(p_graph->SetOfNodes[n])) > 0 && (n_init->distance(p_graph->SetOfNodes[n]) < r_neighborhood) )
		{
			//std::cout<<"NODE IN NEIGHBORHOOD OF Q_GOAL" <<std::endl;
			// If the path to the neighbor is collision free
			if (PathCollisionFree(n_init, p_graph->SetOfNodes[n], obs_vec))
			{
				n_init_potl_neighbors.push_back(p_graph->SetOfNodes[n]);
			}

		}
	}
	// Find the closest of all potential neighbors and use that to connect n_init to graph
	node *nearest_init = n_init_potl_neighbors[0];
	double dist_of_nearest_n_init = n_init->distance(n_init_potl_neighbors[0]);
	for (int pn = 0; pn<int(n_init_potl_neighbors.size()); pn++)
	{
		// Distance to the current neighbor candidate
		double dist_to_current_n = n_init->distance(n_init_potl_neighbors[pn]);
			if (dist_to_current_n < dist_of_nearest_n_init)
			{
				nearest_init = n_init_potl_neighbors[pn];
				dist_of_nearest_n_init = dist_to_current_n;
			}

	}

	// Add this node to the neighbors of node n_init
	// Edge connecting node n_goal to neighbor n
	edge *e_init = new edge(n_init, nearest_init);
	p_graph->SetOfEdges.push_back(e_init);
	n_init->mp_localNeighbors.push_back(e_init);



	// Add n_goal to the neighbors of its neighbor
	edge *e_n_to_init = new edge(nearest_init, n_init);
	p_graph->SetOfEdges.push_back(e_n_to_init);
	nearest_init->mp_localNeighbors.push_back(e_n_to_init);

	// DEBUG:
	// std::cout<<"[LENGTH OF EDGE CONENCTING INIT TO GRAPH] " << e_init->getEdgeLength() <<std::endl;
	// =================================





	// ==== CONNECT Q_GOAL TO GRAPH ====
////////////////////////////////////////
//	for (int n = 0; n < int(p_graph->SetOfNodes.size()); n++)
//	{
//
//		// If the distance to node n is greater than 0 (i.e. n != n_init) AND distance is within the neighborhood radius
//		if ( (n_goal->distance(p_graph->SetOfNodes[n])) > 0 && (n_goal->distance(p_graph->SetOfNodes[n]) < r_neighborhood) )
//		{
//			//std::cout<<"NODE IN NEIGHBORHOOD OF Q_GOAL" <<std::endl;
//			// If the path to the neighbor is collision free
//			if (PathCollisionFree(n_goal, p_graph->SetOfNodes[n], obs_vec))
//			{
//
//
//				// Add this node to the neighbors of node n_init
//				n_goal->mp_localNeighbors.push_back(p_graph->SetOfNodes[n]);
//
//				// Edge connecting node n_goal to neighbor n
//				edge *e = new edge(n_goal, p_graph->SetOfNodes[n]);
//
//				p_graph->SetOfEdges.push_back(e);
//
//
//				//////// Add n_goal to the neighbors of its neighbor
//				p_graph->SetOfNodes[n]->mp_localNeighbors.push_back(n_goal);
//				////////
//
//				//std::cout<<"CONNECTED Q_GOAL TO GRAPH" << std::endl;
//
//			}
//		}
//
//	}
////////////////////////////////////////

	// Set of potential neighbors for n_goal
	std::vector<node*> n_goal_potl_neighbors;
	for (int n = 0; n < int(p_graph->SetOfNodes.size()); n++)
	{

		// If the distance to node n is greater than 0 (i.e. n != n_init) AND distance is within the neighborhood radius
		if ( (n_goal->distance(p_graph->SetOfNodes[n])) > 0 && (n_goal->distance(p_graph->SetOfNodes[n]) < r_neighborhood) )
		{
			//std::cout<<"NODE IN NEIGHBORHOOD OF Q_GOAL" <<std::endl;
			// If the path to the neighbor is collision free
			if (PathCollisionFree(n_goal, p_graph->SetOfNodes[n], obs_vec))
			{
				n_goal_potl_neighbors.push_back(p_graph->SetOfNodes[n]);
			}

		}
	}
	// Find the closest neighbor
	node *nearest_goal = n_goal_potl_neighbors[0];
	double dist_of_nearest_n_goal = n_goal->distance(n_goal_potl_neighbors[0]);
	for (int pn = 0; pn<int(n_goal_potl_neighbors.size()); pn++)
	{
		double dist_to_current_n = n_goal->distance(n_goal_potl_neighbors[pn]);
			if (dist_to_current_n < dist_of_nearest_n_goal)
			{
				nearest_goal = n_goal_potl_neighbors[pn];
				dist_of_nearest_n_goal = dist_to_current_n;
			}

	}

	// Add this node to the neighbors of node n_init
	// Edge connecting node n_goal to neighbor n
	edge *e_goal = new edge(n_goal, nearest_goal);
	p_graph->SetOfEdges.push_back(e_goal);
	n_goal->mp_localNeighbors.push_back(e_goal);


	// Add n_goal to the neighbors of its neighbor
	// Edge connecting neighbor n to n_goal
	edge *e_n_to_goal = new edge(nearest_goal, n_goal);
	p_graph->SetOfEdges.push_back(e_n_to_goal);
	nearest_goal->mp_localNeighbors.push_back(e_n_to_goal);

	// DEBUG:
	// std::cout<<"num neighbors n_goal: " << n_goal->mp_localNeighbors.size() << std::endl;
	// std::cout<<"num neighbors of n_goal's neighbor: " << nearest_goal->mp_localNeighbors.size() << std::endl;

	// DEBUG:
	// std::cout<<"[LENGTH OF EDGE CONENCTING GOAL TO GRAPH] " << e_goal->getEdgeLength() <<std::endl;

	// =================================



//	// DEBUG:
//	for (int i = 0; i<int(p_graph->SetOfNodes.size()); i++)
//	{
//		if (p_graph->SetOfNodes[i]==n_init)
//		{
//			std::cout<<"====N_INIT IS IN GRAPH====" <<std::endl;
//		}
//		else if (p_graph->SetOfNodes[i]==n_goal)
//		{
//			std::cout<<"====N_GOAL IS IN GRAPH====" <<std::endl;
//		}
//
//	}

	// DEBUG:
//	// IDENTIFY IF THERE ARE ANY NODES WITH THE GOAL AS A NEIGHBOR
//	for (int i = 0; i<int(p_graph->SetOfNodes.size()); i++)
//	{
//		node *curr = p_graph->SetOfNodes[i];
//		for (int j = 0; j < int(curr->mp_localNeighbors.size()); j++)
//		{
//			if (curr->mp_localNeighbors[j]->m_node1 == n_goal | curr->mp_localNeighbors[j]->m_node2 == n_goal)
//			{
//				std::cout<< "GOAL IS A NEIGHBOR OF SOME CELL" << std::endl;
//			}
//			if (curr->mp_localNeighbors[j]->m_node1 == n_init | curr->mp_localNeighbors[j]->m_node2 == n_init)
//			{
//				std::cout<< "INIT IS A NEIGHBOR OF SOME CELL" << std::endl;
//			}
//		}
//	}


	std::cout<<"[CONDUCTING A* SEARCH...]"<<std::endl;

	PRMPath = AStarSearch(n_init, n_goal);
	return PRMPath;

	// Using this definition of AStarSearch because the above definition was not working
	// As a debug approach, I plotted the processed nodes from AStarSearch
	//std::vector<node*> processed_nodes = AStarSearch(n_init, n_goal);
	//return processed_nodes;

}
