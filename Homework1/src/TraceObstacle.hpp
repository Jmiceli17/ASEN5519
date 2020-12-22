/*
 * TraceObstacle.hpp
 *
 *  Created on: Sep 22, 2020
 *      Author: Joe
 */

#ifndef TRACEOBSTACLE_HPP_
#define TRACEOBSTACLE_HPP_

#include <iostream>
#include <vector>
#include <fstream>

#include "obstacle.hpp"
#include "GetDirection.hpp"
#include "TakeStep.hpp"
#include "GetDistance.hpp"
#include "SamplePoints.hpp"

using namespace std;




//std::vector<std::vector<double>> TraceObstacle(std::vector<double> &curr_pos, AlignedRectangle &obstacle, std::ofstream &log_file, double step_size, std::vector<double> &q_goal);

// std::vector<double> TraceObstacle(std::vector<double> &curr_pos, AlignedRectangle &obstacle, std::ofstream &log_file, double step_size, std::vector<double> &q_goal);

//void TraceObstacle(std::vector<double> &curr_pos, AlignedRectangle &obstacle, std::vector<std::vector<double>> &path_taken, std::vector<double> &leave_pos, std::ofstream &log_file, double step_size, std::vector<double> &q_goal);
std::vector<double> TraceObstacle(std::vector<double> &curr_pos, std::vector<double> &hit_pos, AlignedRectangle &obstacle, std::vector<std::vector<double>> &path_taken, std::ofstream &log_file, double step_size, std::vector<double> &q_goal);





#endif /* TRACEOBSTACLE_HPP_ */
