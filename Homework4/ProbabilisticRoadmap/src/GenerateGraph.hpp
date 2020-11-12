/*
 * GenerateGraph.hpp
 *
 *  Created on: Nov 1, 2020
 *      Author: Joe
 *
 *  Input:
 *  	num_nodes - number of nodes to sample
 *  	r_neighborhood - radius of neighborhood of each node to conduct local planning on
 *  	obs_vec - vector of obstacles of the workspace
 *  	X_MIN - lower x bound of space
 *  	X_MAX - upper x bound of space
 *  	Y_MIN - lower y bound of space
 *  	Y_MAX - upper y bound of space
 *
 *	Ouptut:
 *		p_Graph - pointer to a graph object
 *
 *
 */


#ifndef GENERATEGRAPH_HPP_
#define GENERATEGRAPH_HPP_

#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <random>


#include "edge.hpp"
#include "graph.hpp"
#include "node.hpp"
#include "rectangle.hpp"
#include "PathCollisionFree.hpp"

using namespace std;


// TODO: should obs_vec be a pointer???

graph* GenerateGraph(int num_nodes, double r_neighborhood, std::vector<RectangleObs> obs_vec, double X_MIN, double X_MAX, double Y_MIN, double Y_MAX);


#endif /* GENERATEGRAPH_HPP_ */
