/*
 * node.cpp
 *
 *  Created on: Oct 31, 2020
 *      Author: Joe
 */

#include "node.hpp"


// Constructor
node::node()
{
	m_Xpos = 0.0;
	m_Ypos = 0.0;

	m_heuristic = 0;
	m_distFromStart = 0;

	// Initialize priority with very large value
	m_priority = 999999999;

	mp_parent = NULL;		// This will be set during AStarSearch

	m_nodeExplored = false;
}


node::node(std::vector<double> pos)
{

	setX(pos[0]);
	setY(pos[1]);

	m_heuristic = 0;
	m_distFromStart = 0;

	// Initialize priority with very large value
	m_priority = 999999999;

	mp_parent = NULL;		// This will be set during AStarSearch

	m_nodeExplored = false;

}


// Destructor
node::~node()
{

}

void node::setX(double x)
{
	m_Xpos = x;
}

void node::setY(double y)
{
	m_Ypos = y;
}

double node::getX()
{
	double x = m_Xpos;
	return x;
}

double node::getY()
{
	double y = m_Ypos;
	return y;
}



double node::distance(node *dest_node)
{
	std::vector<double> p1 = {this->getX(), this->getY()};
	std::vector<double>	p2 = {dest_node->getX(), dest_node->getY()};


	return GetDistance(p1, p2);
}

edge* node::getEdge(node* n)
{
	// Check all the edges of this node to see if it's connected to n
	for (int i=0; i<int(mp_localNeighbors.size()); i++)
	{
		// if the neighbor is on either end of this edge
		if ((mp_localNeighbors[i]->m_node2 == n) | (mp_localNeighbors[i]->m_node1 == n))
		{
			// n is a neighbor, return the edge to this neighbor
			return mp_localNeighbors[i];
		}
//		else
//		{
//			std::cout<<"NO VALID EDGE TO NODE FROM: " << this->m_Id << " TO "<< n->m_Id << std::endl;
//		}
	}
}


// Define this such that calling std::sort will put the node with the lowest priority at the front
bool node::operator < (const node& n)const
{
	return (this->m_priority < n.m_priority);
}



