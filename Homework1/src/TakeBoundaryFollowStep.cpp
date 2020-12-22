///*
// * TakeBoundaryFollowStep.cpp
// *
// *  Created on: Sep 21, 2020
// *      Author: Joe
// */
//
//
//
//#include "TakeBoundaryFollowStep.hpp"
//
//
//
//// std::vector<double>  TakeStep(std::vector<double> init_position, std::vector<double> direction_vector, double step_size)
//void  TakeStep(std::vector<double> &position, std::vector<vector<double>> &previously_taken_path, double step_size)
//
//{
//	position = previously_taken_path;
//	std::vector<double> step_vector(direction_vector.size());
//	std::vector<double> new_position(direction_vector.size());
//	int dir_vec_size = direction_vector.size();
//
//	for (int i = 0; i<int(previously_taken_path.size()); i++)
//	{
//		step_vector[i] = step_size*direction_vector[i];
//	}
//
//	for (int i = 0; i<dir_vec_size; i++)
//	{
//		position[i] += step_vector[i];
//	}
//
//
//}
