//============================================================================
// Name        : AStarSearch.cpp
// Author      : 
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <algorithm>
#include <iostream>


#include "node.hpp"

using namespace std;


int main() {

	// ======== CONFIGURE THE GRAPH ========
	// Nodes are initialized as (heuristic, path-length-from-start, parent-node)
	node n_start(0, 0);

	node n_A(3, 1, n_start);
	node n_B(2, 1, n_start);
	node n_C(3, 1, n_start);
	node n_D(3, 2, n_A);
	node n_E(1, 2, n_A);
	node n_F(3, 4, n_A);

	node n_G(2, 5, n_B);
	node n_H(1, 2, n_B);
	node n_I(2, 3, n_B);

	node n_J(3, 2, n_C);
	node n_K(2, 1, n_C);
	node n_L(3, 2, n_C);

	//node n_goal(3, 4, n_A);



	// Set the children of each node
	// TODO: call this function in the constructor of each node
	n_start.AddChild(&n_A);
	n_start.AddChild(&n_B);
	n_start.AddChild(&n_C);

	n_A.AddChild(&n_D);
	n_A.AddChild(&n_E);
	n_A.AddChild(&n_F);

	n_B.AddChild(&n_G);
	n_B.AddChild(&n_H);
	n_B.AddChild(&n_I);

	n_C.AddChild(&n_J);
	n_C.AddChild(&n_K);
	n_C.AddChild(&n_L);

	std::cout<< "# children of n_A: "<< n_A.p_children.size()<<std::endl;


	// ======== CONDUCT THE SEARCH ========
	// initialize the open list
	std::vector<node*> p_openList;
	std::vector<node *> p_processedNodes;

	p_openList.push_back(&n_start);

	// While the open list is not empty
	while(!p_openList.empty())
	{
		std::cout<<"size of open list " << p_openList.size()<<std::endl;

		// Get the node with the lowest priority
		// this will be the 1st element of the open list set
		// Each element in p_openList is a node pointer
		node *n = *p_openList.begin();
		std::cout<<"Priority of node = "<< n->f << std::endl;
		//std::cout<<"Number of children = "<< n->p_children.size() << std::endl;
		// Remove the first element from the open list set
		p_openList.erase(p_openList.begin());

		// Check if this node is the goal node


		// Expand from the current node and update the resulting nodes' back pointers
		for (int i = 0; i<int(n->p_children.size()); i++)
		{
			// If this node has not been explored yet, add it to open list
			if(! n->p_children[i]->node_explored)
			{

				// set the back pointer of this node's child to be this node
				// else if g(nbest) + c(nbest, x) < g(x)
				n->p_children[i]->p_parent = n;

				p_openList.push_back(n->p_children[i]);
			}
		}

		// Mark the current node as explored and add it to the
		// list of processed nodes
		n->node_explored = true;
		p_processedNodes.push_back(n);		// TODO: not convinced that this is necessary

		// sort the open list to set the minimum priority node at the
		// beginning of the list
		std::sort(p_openList.begin(), p_openList.end());


	}	// End while

	// Get the path
	// goal_node.parent -> node.p_parent -> node.p_parent -> ... -> start_node











	cout << "!!!Hello World!!!" << endl; // prints !!!Hello World!!!
	return 0;
}
