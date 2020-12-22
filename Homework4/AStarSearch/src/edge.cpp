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
	m_node1 = n1;
	m_node2 = n2;

	m_edgeLength = 0;

}

// For use in problem 1
edge::edge(node* n1, node* n2, double length)
{
	// Set the member variables
	m_node1 = n1;
	m_node2 = n2;
	m_edgeLength = length;


}


edge::~edge()
{

}

double edge::getEdgeLength()
{
	return sqrt(pow(m_node1->getX() - m_node2->getX(),2) + pow(m_node1->getY() - m_node2->getY(),2));

}




