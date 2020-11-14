/*
 * PRMQuery.hpp
 *
 *  Created on: Nov 4, 2020
 *      Author: Joe
 *
 *  Description: function that queries a graph in the form of
 *  			 G = (V, E) and returns a set of edges to take an
 *  			 input configuration to a goal configuration
 * 	Input: p_graph - pointer to the graph object
 * 		   obs_vec - vector of rectangular obstacles
 * 		   n_init - pointer to initial node
 * 		   n_goal - pointer to goal node
 * 		   r_neighborhood - Radius to use when attempting to identify neighbors of n_init and n_goal
 *
 * 	Output: The shortest path on the graph connecting n_init to n_goal
 *
 *
 *
 */

#ifndef PRMQUERY_HPP_
#define PRMQUERY_HPP_

#include <vector>
#include <algorithm>

#include "AStarSearch.hpp"
#include "PathCollisionFree.hpp"

#include "edge.hpp"
#include "graph.hpp"
#include "node.hpp"
#include "rectangle.hpp"

using namespace std;

//std::vector<node*> PRMQuery(graph* p_graph, std::vector<RectangleObs> obs_vec, node* n_init, node* n_goal, double r_neighborhood);
std::vector<edge*> PRMQuery(graph* p_graph, std::vector<RectangleObs> obs_vec, node* n_init, node* n_goal, double r_neighborhood);



#endif /* PRMQUERY_HPP_ */
