/*
 * edge.cpp
 *
 *  Created on: Nov 1, 2020
 *      Author: Joe
 */


#include "edge.hpp"


edge::edge(node* n1, node* n2)
{
	// Set the member variables
	node1 = n1;
	node2 = n2;

//	/edge_length = GetDistance(n1->position, n2->position);

	edge_length = sqrt(pow(n1->mp_position->m_Xpos - n2->mp_position->m_Xpos,2)+pow(n1->mp_position->m_Ypos - n2->mp_position->m_Ypos,2));

}

edge::~edge()
{

}




