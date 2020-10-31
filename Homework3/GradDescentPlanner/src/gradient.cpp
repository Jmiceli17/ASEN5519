/*
 * gradient.cpp
 *
 *  Created on: Oct 18, 2020
 *      Author: Joe
 */


#include "gradient.hpp"

std::vector<double> compute_gradient(std::vector<double>q , std::vector<double> q_goal, params_t PLANNER_PARAMS, std::vector<RectangleObs> obs_vec)
{
	std::vector<double> gradient(q.size());				// total gradient
	std::vector<double> grad_U_att(q.size());			// attractive portion of gradient
	std::vector<double> grad_U_rep(q.size());			// repulsive portion of gradient
	std::vector<double> grad_U_rep_i(q.size());			// repulsive portion of gradient for a single obstacle
	std::vector<double> grad_U_rep_temp(q.size());

	double dist_to_goal = GetDistance(q, q_goal);
	std::vector<double> q_diff = Subtract2DVectors(q, q_goal);	// Difference between current configuration and goal


	// std::cout<<"dist from goal= "<< dist_to_goal << std::endl;



	// ==== Compute the attractive portion of the gradient ====
	for (int i=0; i<int(q_diff.size()); i++)
	{
		if (dist_to_goal <= PLANNER_PARAMS.D_STAR)
		{
			// Use the conic portion of the attractive gradient
			grad_U_att[i] = PLANNER_PARAMS.K_ATT*q_diff[i];
		}
		else
		{
			// Robot is close to goal so use the quadratic portion of the attractive gradient
			grad_U_att[i] = PLANNER_PARAMS.D_STAR*PLANNER_PARAMS.K_ATT*q_diff[i]/dist_to_goal;
		}
	}





	// ==== Compute the repulsive portion of the gradient ====
	// Loop over each obstacle
	for (int x = 0; x<int(obs_vec.size()); x++)
	{
		// Get the shortest distance to the current obstacle
		double dist_to_obs = obs_vec[x].GetDistanceToObstacle(q);

		// Get the distance portion of the obstacle gradient for the robot's current position
		std::vector<double> grad_d_i = obs_vec[x].GetObstacleGradient(q);

		for (int i = 0; i<int(grad_d_i.size()); i++)
		{
			// If our current position is within the Q* threshold, we need to get the gradient from this obstacle
			//if (dist_to_obs <= PLANNER_PARAMS.Q_STAR) // Global Q*
			if (dist_to_obs <= obs_vec[x].q_star)	// Check this obstacles distance threshold
			{
				// Close to an obstacle so consider it in the repulsive term
				grad_U_rep_i[i] = PLANNER_PARAMS.K_REP*(1/PLANNER_PARAMS.Q_STAR - 1/dist_to_obs)*grad_d_i[i]/(dist_to_obs*dist_to_obs);
			}

			else
			{
				// Not close enough to consider any obstacles
				grad_U_rep_i[i] = 0;
			}
			// Add the repulsive portion of this obstacle to the overall repulsive gradient term
			grad_U_rep_temp = Add2DVectors(grad_U_rep, grad_U_rep_i);

			grad_U_rep = grad_U_rep_temp;
		}
	}






	//std::cout<<"grad_U_att[0] = "<< grad_U_att[0] << std::endl;
	//std::cout<<"grad_U_rep[0] = "<< grad_U_rep[0] << std::endl;

	gradient = Add2DVectors(grad_U_att, grad_U_rep);



	return gradient;
}

