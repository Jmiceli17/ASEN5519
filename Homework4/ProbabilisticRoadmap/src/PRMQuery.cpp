/*
 * PRMQuery.cpp
 *
 *  Created on: Nov 4, 2020
 *      Author: Joe
 */


#include "PRMQuery.hpp"

std::vector<edge*> PRMQuery(graph* p_graph, std::vector<RectangleObs> obs_vec, node* n_init, node* n_goal, double r_neighborhood)
{
	std::vector<edge*> PRMPath;
	// Add initial and goal nodes to the graph
	p_graph->SetOfNodes.push_back(n_init);
	p_graph->SetOfNodes.push_back(n_goal);


	// ==== CONNECT Q_INIT TO GRAPH ====
	// Generate neighbors of Q_init
	for (int n = 0; n < int(p_graph->SetOfNodes.size()); n++)
	{
		// If the distance to node n is greater than 0 (i.e. n != n_init) AND distance is within the neighborhood radius
		if (n_init->distance(p_graph->SetOfNodes[n]) > 0 && n_init->distance(p_graph->SetOfNodes[n]) < r_neighborhood)
		{
			//std::cout<<"Distance to neighbor m: " << p_Graph->SetOfNodes[n]->distance(p_Graph->SetOfNodes[m])<<std::endl;

			// Add this node to the neighbors of node n_init
			n_init->mp_localNeighbors.push_back(p_graph->SetOfNodes[n]);
		}
	}
	// Generate closest valid edge for q_init
	std::sort(n_init->mp_localNeighbors.begin(), n_init->mp_localNeighbors.end());

	// Now that the neighbors have been sorted, iterating over them will start with
	// the nearest neighbor
	for (int ni = 0; ni < int(n_init->mp_localNeighbors.size()); ni++)
	{
		if (PathCollisionFree(n_init, n_init->mp_localNeighbors[ni], obs_vec))
			{
				// Edge connecting node n_init to neighbor ni
				edge *e = new edge(n_init, n_init->mp_localNeighbors[ni]);

				p_graph->SetOfEdges.push_back(e);

				break;	// We only want to connect q_init with the closest node
			}
	}
	// =================================


	// ==== CONNECT Q_GOAL TO GRAPH ====
	// Generate neighbors of Q_goal
	for (int n = 0; n < int(p_graph->SetOfNodes.size()); n++)
	{
		// If the distance to node n is greater than 0 (i.e. n != n_goal) AND distance is within the neighborhood radius
		if (n_goal->distance(p_graph->SetOfNodes[n]) > 0 && n_goal->distance(p_graph->SetOfNodes[n]) < r_neighborhood)
		{
			//std::cout<<"Distance to neighbor m: " << p_Graph->SetOfNodes[n]->distance(p_Graph->SetOfNodes[m])<<std::endl;

			// Add this node to the neighbors of node n_goal
			n_goal->mp_localNeighbors.push_back(p_graph->SetOfNodes[n]);
		}
	}
	// Generate closest valid edge for n_goal
	std::sort(n_goal->mp_localNeighbors.begin(), n_goal->mp_localNeighbors.end());

	// Now that the neighbors have been sorted, iterating over them will start with
	// the nearest neighbor
	for (int ni = 0; ni < int(n_goal->mp_localNeighbors.size()); ni++)
	{
		if (PathCollisionFree(n_goal, n_goal->mp_localNeighbors[ni], obs_vec))
			{
				// Edge connecting n_goal to neighbor ni
				edge *e = new edge(n_goal, n_goal->mp_localNeighbors[ni]);

				p_graph->SetOfEdges.push_back(e);

				break;	// We only want to connect q_init with the closest node
			}
	}
	// =================================

	std::cout<<"CONNECTED Q_INIT AND Q_GOAL TO GRAPH"<<std::endl;

	std::cout<<"CONDUCTING A* SEARCH"<<std::endl;


	PRMPath = AStarSearch(n_init, n_goal);





	return PRMPath;
}
