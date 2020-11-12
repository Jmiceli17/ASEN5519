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
		// TODO: make these private
		// The two nodes that define this edge
		node *m_node1;
		node *m_node2;

		// The length of the edge
		double m_edgeLength;

		// Constructor
		edge(node *n1, node *n2);

		// Destructor
		~edge();

		double getEdgeLength();


};


#endif /* EDGE_HPP_ */
