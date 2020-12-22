/*
 * PathCollisionFree.hpp
 *
 *  Created on: Nov 3, 2020
 *      Author: Joe
 */

#ifndef PATHCOLLISIONFREE_HPP_
#define PATHCOLLISIONFREE_HPP_

#include <vector>

#include "node.hpp"
#include "rectangle.hpp"
#include "utilities.hpp"

using namespace std;

bool PathCollisionFree(node *n1, node *n2,  std::vector<RectangleObs> obs_vec);


#endif /* PATHCOLLISIONFREE_HPP_ */
