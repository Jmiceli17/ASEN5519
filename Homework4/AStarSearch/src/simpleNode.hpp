/*
 * node.hpp
 *
 *  Created on: Oct 31, 2020
 *      Author: Joe
 */

#ifndef SIMPLENODE_HPP_
#define SIMPLENODE_HPP_

#include <vector>

using namespace std;


class simpleNode
{
public:
	double h;							// heuristic value of this node
	double g;							// path length from q_start, NOTE this should be computed based on parents
	double f;							// f(n) = g(n) + h(n), the priority
	bool node_explored;					// bool indicating if this node has already been visited

	simpleNode *p_parent; 					// Pointer to the parent node of this node
	std::vector<simpleNode*> p_children; 		// Vector of pointers to children of this node

	// Constructors
	simpleNode(double heuristic, double length_from_start);					// Constructor for start node (i.e. no parent)
	simpleNode(double heuristic, double length_from_start, simpleNode p_parent);

	// Destructor
	~simpleNode();

	// Function for adding children to p_children
	void AddChild(simpleNode *child);

	// overload < operator to use "sort" method on nodes
	bool operator < (const simpleNode& n)const;







};


#endif /* SIMPLENODE_HPP_ */
