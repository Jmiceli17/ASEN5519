/*
 * GenerateVEctorField.cpp
 *
 *  Created on: Oct 21, 2020
 *      Author: Joe
 */



#include "GenerateVectorField.hpp"

void GenerateVectorField(std::vector<std::vector<double>> workspace_points, std::vector<RectangleObs> obstacle_vector, std::vector<double> q_goal, params_t PLANNER_PARAMS)
{

	std::cout<< "*** GENERATING VECTOR FIELD ***" << std::endl;

	// ======= Logging ========
	std::ofstream LOG_FILE("VectorField.csv");
	LOG_FILE << "X" << "," << "Y" << "," << "U" << "," << "V\n";	// Log file for writing x,y position and u,v gradient at each position

	// Compute and log the gradient for every point

	bool plot_it = true;

	for (int i =0; i < int(workspace_points.size()); i++)
	{

		for (int j = 0; j < int(obstacle_vector.size()); j++)
		{
			// Only consider points that are NOT in collision with an obstacle
			if(obstacle_vector[j].InCollision(workspace_points[i]))
			{
				//std::cout<< "POINT DISCARDED: "<< workspace_points[i][0] << "," << workspace_points[i][1] << std::endl;

				// Ignore this point from the workspace
				plot_it = false;
			}

		}

		if(plot_it)
		{
			std::vector<double> gradient = compute_gradient(workspace_points[i], q_goal, PLANNER_PARAMS, obstacle_vector);
			LOG_FILE<< workspace_points[i][0] << "," << workspace_points[i][1] << ","<< gradient[0] << "," << gradient[1] << "\n";
		}

		plot_it = true;


	}

	LOG_FILE.close();

}
