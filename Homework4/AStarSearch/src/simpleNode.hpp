/*
 * node.hpp
 *
 *  Created on: Oct 31, 2020
 *      Author: Joe
 */

#ifndef NODE_HPP_
#define NODE_HPP_

#include <vector>

using namespace std;


class node
{
public:
	double h;							// heuristic value of this node
	double g;							// path length from q_start, NOTE this should be computed based on parents
	double f;							// f(n) = g(n) + h(n), the priority
	bool node_explored;					// bool indicating if this node has already been visited

	node *p_parent; 					// Pointer to the parent node of this node
	std::vector<node*> p_children; 	// Vector of pointers to children of this node

	// Constructors
	node(double heuristic, double length_from_start);					// Constructor for start node (i.e. no parent)
	node(double heuristic, double length_from_start, node p_parent);

	// Destructor
	~node();

	// Function for adding children to p_children
	void AddChild(node *child);

	// overload < operator to use "sort" method on nodes
	bool operator < (const node& n)const;







};


#endif /* NODE_HPP_ */
