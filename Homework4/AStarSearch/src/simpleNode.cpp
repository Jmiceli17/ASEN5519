/*
 * node.cpp
 *
 *  Created on: Oct 31, 2020
 *      Author: Joe
 */

#include "node.hpp"


// Constructor
node::node(double heuristic, double length_from_start)
{
	h = heuristic;
	g = length_from_start;

	f = h + g;

	node_explored = false;

}

// Constructor
node::node(double heuristic, double length_from_start, node parent)
{
	h = heuristic;
	g = length_from_start;

	f = h + g;

	node_explored = false;

	p_parent = &parent;		// set the pointer to the parent of this node

	// parent.AddChild(this);	// Add this node to the children of the parent

}

// Destructor
node::~node()
{

}

void node::AddChild(node *child)
{
	//node p_child = child;
	p_children.push_back(child);
}

bool node::operator < (const node& n)const
{
	return (this->f < n.f);
}



