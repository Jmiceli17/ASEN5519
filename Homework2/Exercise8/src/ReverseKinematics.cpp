/*
 * ReverseKinematics.cpp
 *
 *  Created on: Oct 11, 2020
 *      Author: Joe
 */

#include "ReverseKinematics.hpp"


std::vector<double> ReverseKinematics_positive(std::vector<double> desired_pt, double l1, double l2)
{
	double t1;
	double t2;
	double xdes = desired_pt[0];
	double ydes = desired_pt[1];

	// Reverse kinematic equations
	double cosT2 = 1/(2*l1*l2)*( ((xdes*xdes)+(ydes*ydes)) - ((l1*l1)+(l2*l2)) );
	t2 = acos(cosT2); // Theta2 (rad)

	double cosT1 = 1/(l1+l2)*( (xdes*(l1+l2*cosT2)) + ydes*l2*sqrt(1-(cosT2*cosT2)) );
	t1 = acos(cosT1); // Theta1 (rad)


	std::vector<double> thetas = {t1, t2};
	return thetas;;


}

std::vector<double> ReverseKinematics_negative(std::vector<double> desired_pt, double l1, double l2)
{
	double t1;
	double t2;
	double xdes = desired_pt[0];
	double ydes = desired_pt[1];

	// Reverse kinematic equations
	double cosT2 = 1/(2*l1*l2)*( ((xdes*xdes)+(ydes*ydes)) - ((l1*l1)+(l2*l2)) );
	t2 = acos(cosT2); // Theta2 (rad)

	double cosT1 = 1/(l1+l2)*( (xdes*(l1+l2*cosT2)) - ydes*l2*sqrt(1-(cosT2*cosT2)) );
	t1 = acos(cosT1); // Theta1 (rad)


	std::vector<double> thetas = {t1, t2};
	return thetas;


}


