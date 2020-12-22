/*
 * GenerateGraph.cpp
 *
 *  Created on: Nov 1, 2020
 *      Author: Joe
 */


#include "GenerateGraph.hpp"

graph* GenerateGraph(int num_nodes, double r_neighborhood, std::vector<RectangleObs> obs_vec, double X_MIN, double X_MAX, double Y_MIN, double Y_MAX)
{
	// For random number generator
	std::random_device rand_dev;
	std::mt19937 generator(rand_dev());

	graph *p_Graph = new graph();
	int num_generated_nodes = 0;

	std::cout<< "[GENERATING NODES...]" << std::endl;

 	// ==== Generate the set of nodes ====
	while (num_generated_nodes < num_nodes)
	{
		bool point_is_valid = true;
		node *n = new node();

		// Generate node by sampling a random number within the range of the space
		// rand()/RAND_MAX returns number between 0 and 1;

		//n->setX( ((X_MAX - X_MIN) * ( (double)rand() / (double)RAND_MAX )) + X_MIN );
		//n->setY( ((Y_MAX - Y_MIN) * ( (double)rand() / (double)RAND_MAX )) + Y_MIN );

		std::uniform_real_distribution<double>  xunif(X_MIN, X_MAX);
		std::uniform_real_distribution<double>  yunif(Y_MIN, Y_MAX);

		double randX = xunif(generator);
	    double randY = yunif(generator);

		n->setX( randX );
		n->setY( randY );

		//std::cout<< "TOTAL # OBSTACLES TO CHECK: " << obs_vec.size() << std::endl;

		//std::cout<< "GENERATED POS: " << n->getX() << "," << n->getY() << std::endl;

		// Check if the new point is in collision
		for (int o = 0; o < int(obs_vec.size()); o++)
		{
			// If the new point is in collision with any obstacle,
			// we can't add it to the set of nodes on the graph
			if(obs_vec[o].InCollision( {n->getX(), n->getY()} ))
			{
				point_is_valid = false;
			}
		}

		// If the new node doesn't collide with any obstacles, add it to the set of nodes
		if (point_is_valid)
		{
			p_Graph->SetOfNodes.push_back(n);
		}

		// Increment the number of generated nodes (we are only sampling num_nodes times, they don't have to
		// produce successful additions to the graph)
		num_generated_nodes++;

	} // End while




	std::cout<< "[GENERATING EDGES IN LOCAL NEIGHBORHOODS...]" << std::endl;

	// ==== Generate the set of edges ====
	// Generate neighbors for each node
	for (int n = 0; n < int(p_Graph->SetOfNodes.size()); n++)
	{
		for (int m = 0; m < int(p_Graph->SetOfNodes.size()); m++)
		{
			// std::cout<<"DIST TO NODE M: " << (p_Graph->SetOfNodes[n]->distance(p_Graph->SetOfNodes[m])) <<std::endl;

			// If the distance to node m is greater than 0 (i.e. m != n) AND distance is within the neighborhood radius
			if (p_Graph->SetOfNodes[n]->distance(p_Graph->SetOfNodes[m]) > 0 && p_Graph->SetOfNodes[n]->distance(p_Graph->SetOfNodes[m]) < 79)
			{
				//std::cout<<"RANGE TO NODE M: " << (p_Graph->SetOfNodes[n]->distance(p_Graph->SetOfNodes[m])) <<std::endl;

				// If the path between node n and node m is collision free, connect them with an edge and add it to the set of edges
				if (PathCollisionFree(p_Graph->SetOfNodes[n], p_Graph->SetOfNodes[m], obs_vec))
				{
					// std:: cout<<"EDGE IS COLLISION FREE"<< std::endl;

					// If this pair of nodes does not already have an edge connecting them
					if ( !(p_Graph->edgeExists(p_Graph->SetOfNodes[n], p_Graph->SetOfNodes[m])) )
					{
						// Add this node to the neighbors of node n
						// p_Graph->SetOfNodes[n]->mp_localNeighbors.push_back(p_Graph->SetOfNodes[m]);

						// Add this node to the neighbors of node n
						// Edge connecting node n to neighbor m
						edge *e = new edge(p_Graph->SetOfNodes[n], p_Graph->SetOfNodes[m]);
						p_Graph->SetOfNodes[n]->mp_localNeighbors.push_back(e);

						p_Graph->SetOfEdges.push_back(e);


						// Add node n to the neighbors of node m
						// Edge connecting node m to neighbor n
						edge *e2 = new edge(p_Graph->SetOfNodes[m], p_Graph->SetOfNodes[n]);
						p_Graph->SetOfNodes[m]->mp_localNeighbors.push_back(e2);


					} //end if

				} // end if

			} // end if

		} // end for

	} // end for


	return p_Graph;

}




