/*
 * node.cpp
 *
 *  Created on: Oct 31, 2020
 *      Author: Joe
 */

#include "simpleNode.hpp"


// Constructor
simpleNode::simpleNode(double heuristic, double length_from_start)
{
	h = heuristic;
	g = length_from_start;

	f = h + g;

	node_explored = false;

}

// Constructor
simpleNode::simpleNode(double heuristic, double length_from_start, simpleNode parent)
{
	h = heuristic;
	g = length_from_start;

	f = h + g;

	node_explored = false;

	p_parent = &parent;		// set the pointer to the parent of this node

	// parent.AddChild(this);	// Add this node to the children of the parent

}

// Destructor
simpleNode::~simpleNode()
{

}

void simpleNode::AddChild(simpleNode *child)
{
	//node p_child = child;
	p_children.push_back(child);
}

bool simpleNode::operator < (const simpleNode& n)const
{
	return (this->f < n.f);
}



