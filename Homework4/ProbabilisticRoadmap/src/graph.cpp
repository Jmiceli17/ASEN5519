/*
 * graph.cpp
 *
 *  Created on: Nov 4, 2020
 *      Author: Joe
 */




#include "graph.hpp"


graph::graph()
{

}

graph::~graph()
{

}


bool graph::edgeExists(node* n1, node* n2)
{
	bool hasEdge = false;

	for (int i = 0; i < int(SetOfEdges.size()); i++)
	{
		edge *e = SetOfEdges.at(i);

		if (e->m_node1 == n1 && e->m_node2 == n2)
		{
			hasEdge = true;
			break;
		}
	}

	return hasEdge;
}
