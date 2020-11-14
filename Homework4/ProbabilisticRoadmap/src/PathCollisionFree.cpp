/*
 * PathCollisionFree.cpp
 *
 *  Created on: Nov 3, 2020
 *      Author: Joe
 */




#include "PathCollisionFree.hpp"

bool PathCollisionFree(node *n1, node *n2,  std::vector<RectangleObs> obs_vec)
{
	bool path_is_valid = true;	// Flag indicating if a collision is detected
	double step = 0.01; 				// Step size used to discretize the path

	std::vector<double> dir_vec(2);		// Direction vector from n1 to n2
	dir_vec[0] = n2->getX() - n1->getX();
	dir_vec[1] = n2->getY() - n1->getY();

	double dir_vec_mag = VectorNorm(dir_vec);		// Distance between end points of edge

	double theta = acos(dir_vec[0]/dir_vec_mag);	// Angle of direction vector (wrt. horizontal)

	// Discretize the path
	int i = 1;
	double mag = 0;
	double x_start = n1->getX();
	double y_start = n1->getY();

	std::vector<std::vector<double>> path_points;	// vector of discretized points on path

	//std::cout<<"DISCRETIZING LOCAL EDGE" << std::endl;

	while (mag < dir_vec_mag)
	{
		// Take a step from n1 towards n2
		double pt_x = x_start + i*step*cos(theta);
		double pt_y = y_start + i*step*sin(theta);

		std::vector<double> point = {pt_x,pt_y};

		// add the point to the vector of points
		path_points.push_back(point);

		mag = sqrt((pt_x-x_start)*(pt_x-x_start) + (pt_y-y_start)*(pt_y-y_start));
		//std::cout<<"DISCRETIZED MAG "<< mag << std::endl;

		i++;
	}

	//std::cout<<"CHECKING DISCRETIZED PTS FOR COLLISION" << std::endl;

	//std::cout<<"NUMBER OF PTS TO CHECK: " << path_points.size() <<std::endl;

	// Check all discretized points for collision
	for (int p = 0; p<int(path_points.size()); p++)
	{
		for (int o = 0; o<int(obs_vec.size()); o++)
		{
			if (obs_vec[o].InCollision(path_points[p]))
			{

				//std::cout<<"COLLISION DETECTED ON LOCAL PATH"<<std::endl;

				path_is_valid = false;
				break;
			}
		}
	}


	return path_is_valid;
}
