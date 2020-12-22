/*
 * node.hpp
 *
 *  Created on: Oct 31, 2020
 *      Author: Joe
 */

#ifndef NODE_HPP_
#define NODE_HPP_

#include <vector>
#include <string>

#include "edge.hpp"
#include "GetDistance.hpp"

using namespace std;

class edge;

class node
{
public:
	string m_Id;						// NOT USED IN PROBLEM 2
	double m_Xpos;						// NOT USED IN PROBLEM 1
	double m_Ypos;						// NOT USED IN PROBLEM 1
	double m_heuristic;					// heuristic value of this node (distance to goal)
	double m_distFromStart;				// path length from q_start

	double m_distFromNeighbor;			// NOT USED IN PROBLEM 1

	double m_priority;					// f(n) = g(n) + h(n), the priority
	bool m_nodeExplored;				// bool indicating if this node has already been visited


	node *mp_parent; 					// Pointer to the parent node of this node (where it came from)
	std::vector<edge*> p_neighbors; 	// Vector of pointers to children of this node

	std::vector<edge*> mp_localNeighbors;


	// Constructors
	node();								// FOR PROBLEM 2
	node(std::vector<double> pos);		// FOR PROBLEM 2
	node(string node_id);				// FOR PROBLEM 1


	// Destructor
	~node();

	void setX(double x);
	void setY(double y);

	double getX();
	double getY();
	string getId();

	// Function for returning distance to another node
	double distance(node *dest_node);

	// Function for returning the edge connecting this node to another node
	edge* getEdge(node* n);

	// overload < operator to use "sort" method on nodes
	bool operator < (const node& n)const;



};


#endif /* NODE_HPP_ */
