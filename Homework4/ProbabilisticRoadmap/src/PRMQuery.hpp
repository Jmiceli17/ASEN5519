/*
 * PRMQuery.hpp
 *
 *  Created on: Nov 4, 2020
 *      Author: Joe
 *
 *  Description: function that queries a graph in the form of
 *  			 G = (V, E) and returns a set of edges to take an
 *  			 input configuration to a goal configuration
 * 	Intput:
 *
 * 	Output:
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

std::vector<edge*> PRMQuery(graph* p_graph, std::vector<RectangleObs> obs_vec, node* n_init, node* n_goal, double r_neighborhood);



#endif /* PRMQUERY_HPP_ */
