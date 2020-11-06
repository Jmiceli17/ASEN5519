/*
 * utilities.cpp
 *
 *  Created on: Oct 19, 2020
 *      Author: Joe
 */

#include "utilities.hpp"


std::vector<double> Subtract2DVectors(std::vector<double> v1, std::vector<double> v2)
{
	std::vector<double> difference_vec(2);

	if (int(v1.size()) != 2 | int(v2.size()) !=2 )
	{
		// Vectors must be 2D for this function
		std::cout<< "[ERROR] Vectors must both be 2D." <<std::endl;
		std::cout<< "Size of V1 is: " << int(v1.size()) << " Size of V2 is: "<< int(v2.size()) <<std::endl;
	}

	// compute the difference between v1 and v2
	for (int i=0; i < 2; i++)
	{
		difference_vec[i] = v1[i] - v2[i];
	}

	return difference_vec;
}


std::vector<double> Add2DVectors(std::vector<double> v1, std::vector<double> v2)
{
	std::vector<double> sum_vec(2);

	if (int(v1.size()) != 2 | int(v2.size()) !=2 )
	{
		// Vectors must be 2D for this function
		std::cout<< "[ERROR] Vectors must both be 2D." <<std::endl;
		std::cout<< "Size of V1 is: " << int(v1.size()) << " Size of V2 is: "<< int(v2.size()) <<std::endl;
	}

	// compute the difference between v1 and v2
	for (int i=0; i < 2; i++)
	{
		sum_vec[i] = v1[i] + v2[i];
	}

	return sum_vec;
}


double VectorNorm(std::vector<double> v)
{
	double norm;
	double square = 0;
	for (int i = 0; i<int(v.size()); i++ )
	{
		square += v[i]*v[i];
	}
	norm = sqrt(square);

	return norm;
}


std::vector<double> UnitVector(std::vector<double> v)
{
	std::vector<double> v_out(v.size());
	double norm = VectorNorm(v);

	for (int i = 0; i<int(v.size()); i++ )
	{
		v_out[i] = v[i]/norm;
	}

	return v_out;
}





