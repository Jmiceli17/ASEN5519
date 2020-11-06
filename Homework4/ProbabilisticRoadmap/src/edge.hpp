/*
 * edge.hpp
 *
 *  Created on: Nov 1, 2020
 *      Author: Joe
 */

#ifndef EDGE_HPP_
#define EDGE_HPP_

#include "GetDistance.hpp"
#include "node.hpp"

class node;


class edge
{
	public:

		// The two nodes that define this edge
		node *node1;
		node *node2;

		// The length of the edge
		double edge_length;

		// Constructor
		edge(node *n1, node *n2);

		// Destructor
		~edge();

};


#endif /* EDGE_HPP_ */
