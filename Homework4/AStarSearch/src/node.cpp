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
	m_distFromNeighbor = 0;

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
	m_distFromNeighbor = 0;

	// Initialize priority with very large value
	m_priority = 999999999;

	mp_parent = NULL;		// This will be set during AStarSearch

	m_nodeExplored = false;

}

node::node(string node_id)
{

	m_Id = node_id;

	m_Xpos = 0.0;
	m_Ypos = 0.0;

	m_heuristic = 0;
	m_distFromStart = 0;
	m_distFromNeighbor = 0;

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

string node::getId()
{
	string id = m_Id;
	return id;
}


double node::distance(node *dest_node)
{
	std::vector<double> p1 = {this->getX(), this->getY()};
	std::vector<double>	p2 = {dest_node->getX(), dest_node->getY()};


	return GetDistance(p1, p2);
}

// Define this such that calling std::sort will put the node with the lowest priority at the front
bool node::operator < (const node& n)const
{
	return (this->m_priority < n.m_priority);
}



