/*
 * graph.hpp
 *
 *  Created on: Nov 1, 2020
 *      Author: Joe
 */

#ifndef GRAPH_HPP_
#define GRAPH_HPP_

#include <vector>

#include "edge.hpp"
#include "node.hpp"


using namespace std;



class graph
{
	public:
	// Constructor
	graph();

	// Destructor
	~graph();


	std::vector<node*> SetOfNodes;	// Set of nodes that define the graph
	std::vector<edge*> SetOfEdges;	// Set of edges that define the graph

};


#endif /* GRAPH_HPP_ */
