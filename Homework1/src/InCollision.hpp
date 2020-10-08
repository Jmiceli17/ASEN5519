/*
 * InCollision.hpp
 *
 *  Created on: Sep 18, 2020
 *      Author: Joe
 */

#ifndef INCOLLISION_HPP_
#define INCOLLISION_HPP_
#include <algorithm>
#include <vector>
#include <cmath>
#include <iostream>
#include <windows.h>
#include "rect_obstacle.hpp"
#include "obstacle.hpp"
using namespace std;



// bool InCollision(std::vector<double> position, rect_obstacle O, double obstacle_offset);
bool InCollision(std::vector<double> position, AlignedRectangle O, double obstacle_offset);


#endif /* INCOLLISION_HPP_ */
