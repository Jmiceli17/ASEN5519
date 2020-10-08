/*
 * TraceObstacle.cpp
 *
 *  Created on: Sep 22, 2020
 *      Author: Joe
 *
 *      Function to trace an AlignedRectangle object and return the path to the leave point
 */


#include "TraceObstacle.hpp"


//std::vector<std::vector<double>> TraceObstacle(std::vector<double> &curr_pos, AlignedRectangle &obstacle, std::ofstream &log_file, double step_size, std::vector<double> &q_goal)
//void TraceObstacle(std::vector<double> &curr_pos, AlignedRectangle &obstacle, std::vector<std::vector<double>> &path_taken, std::vector<double> &leave_pos, std::ofstream &log_file, double step_size, std::vector<double> &q_goal)
std::vector<double> TraceObstacle(std::vector<double> &curr_pos, std::vector<double> &hit_pos, AlignedRectangle &obstacle, std::vector<std::vector<double>> &path_taken, std::ofstream &log_file, double step_size, std::vector<double> &q_goal)

{


	std::vector<double> leave_pos = curr_pos;
	std::vector<double> starting_pos = curr_pos;
	std::vector<double> dir_vec = {0,0};

	while (not obstacle.obstacle_traced)
	{
//		std::cout << "in while" << std::endl;
//
//		std::cout << "hit pos = " <<hit_pos[0] << std::endl;
//		std::cout << "hit pos size = " <<hit_pos.size() << std::endl;

		GetDirection(dir_vec, curr_pos, hit_pos);
//		std::cout << "got direction" << std::endl;

		TakeStep(curr_pos, dir_vec, step_size);

		// Log the new position
		log_file << curr_pos[0] << "," << curr_pos[1] << "\n";
//		std::cout << "logged points" << std::endl;

		// Update the leave position if we're closer to the goal
		if (GetDistance(curr_pos, q_goal) < GetDistance(leave_pos, q_goal))
		{
			leave_pos = curr_pos;
			std::cout << "leave pos = " <<leave_pos[0] << std::endl;

		}

		std::vector<std::vector<double>> sampled_points; // sampled points in circle around robot

		// Get a new set of sample points
		SamplePoints(curr_pos, sampled_points);


		// Update our hit position
		// Check if any of the sampled points are in collision with an object
		for(int i=0; i<int(sampled_points.size()); i++)
		{
			if (obstacle.InCollision(sampled_points[i]))
			{
				hit_pos = sampled_points[i];
			}
		}
		std::cout << "current pos = " <<curr_pos[0] << std::endl;
		std::cout << "starting_pos pos = " <<starting_pos[0] << std::endl;


		// Check if we've made it to the goal
		if (abs(GetDistance(curr_pos, starting_pos)) <= 1e-3)
		{
			obstacle.obstacle_traced = true;
		}


	}


	return leave_pos;



} // End TraceObstacle


