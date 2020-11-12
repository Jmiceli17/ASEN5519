/*
 * node.hpp
 *
 *  Created on: Oct 31, 2020
 *      Author: Joe
 */

#ifndef NODE_HPP_
#define NODE_HPP_

#include <vector>

#include "edge.hpp"
#include "GetDistance.hpp"
#include "pos2D.hpp"

using namespace std;

class edge;

class node
{
public:
	//pos2D *mp_position;
	double m_Xpos;
	double m_Ypos;
	double m_heuristic;									// heuristic value of this node (distance to goal)
	double m_distFromStart;								// path length from q_start, NOTE this should be computed based on parents
	double m_priority;									// f(n) = g(n) + h(n), the priority
	bool m_nodeExplored;								// bool indicating if this node has already been visited


	node *mp_parent; 					// Pointer to the parent node of this node (where it came from)
	//std::vector<edge*> p_neighbors; 	// Vector of pointers to children of this node

	// std::vector<node*> mp_localNeighbors;
	std::vector<edge*> mp_localNeighbors;


	// Constructors
	node();
	node(std::vector<double> pos);


	// Destructor
	~node();

	void setX(double x);
	void setY(double y);

	double getX();
	double getY();

	// Function for returning distance to another node
	double distance(node *dest_node);

	// Function for returning the edge connecting this node to another node
	edge* getEdge(node* n);

	// overload < operator to use "sort" method on nodes
	bool operator < (const node& n)const;



};


#endif /* NODE_HPP_ */
