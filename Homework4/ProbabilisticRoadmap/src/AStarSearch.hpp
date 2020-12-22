/*
 * AStarSearch.hpp
 *
 *  Created on: Nov 4, 2020
 *      Author: Joe
 */

#ifndef ASTARSEARCH_HPP_
#define ASTARSEARCH_HPP_

#include <algorithm>
#include <vector>

#include "edge.hpp"
#include "node.hpp"
#include "graph.hpp"

using namespace std;

//std::vector<node*> AStarSearch(node* n_init, node* n_goal);

std::vector<edge*> AStarSearch(node* n_init, node* n_goal);
//std::vector<edge*> AStarSearch(node* n_init, node* n_goal, graph* p_G);





#endif /* ASTARSEARCH_HPP_ */
