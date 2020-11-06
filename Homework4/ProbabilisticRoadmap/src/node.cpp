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
	mp_position = new pos2D();
	m_heuristic = 0;
	m_distFromStart = 0;

	m_priority = m_heuristic + m_distFromStart;

	mp_parent = NULL;		// This will be set during AStarSearch

	m_nodeExplored = false;
}


node::node(std::vector<double> pos)
{
	mp_position = new pos2D();
	mp_position->m_Xpos = pos[0];
	mp_position->m_Ypos = pos[1];

	m_heuristic = 0;
	m_distFromStart = 0;

	m_priority = m_heuristic + m_distFromStart;

	mp_parent = NULL;		// This will be set during AStarSearch

	m_nodeExplored = false;

}


// Destructor
node::~node()
{

}

double node::distance(node *dest_node)
{
	std::vector<double> p1 = {this->mp_position->m_Xpos, this->mp_position->m_Ypos};
	std::vector<double>	p2 = {dest_node->mp_position->m_Xpos, dest_node->mp_position->m_Ypos};
	return GetDistance(p1, p2);
}

// Define this such that calling std::sort will put the node with the lowest priority at the front
bool node::operator < (const node& n)const
{
	return (this->m_priority < n.m_priority);
}



