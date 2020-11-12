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

	// m_edgeLength = sqrt(pow(n1->getX() - n2->getX(),2) + pow(n1->getY() - n2->getY(),2));
	m_edgeLength = m_node1->distance(m_node2);
}

edge::~edge()
{

}

double edge::getEdgeLength()
{
	double L = m_edgeLength;
	return L;
	//return sqrt(pow(m_node1->getX() - m_node2->getX(),2) + pow(m_node1->getY() - m_node2->getY(),2));

}


