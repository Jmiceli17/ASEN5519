#include "GetDistance.hpp"



double GetDistance(std::vector<double> &curr_pos, std::vector<double> &destination)
{
	// create a difference vector
	std::vector<double> difference_vec(destination.size());
	int curr_pos_size = curr_pos.size();

	// compute the difference between current position and q_goal
	for (int i=0; i < curr_pos_size; i++)
	{
		difference_vec[i] = destination[i] - curr_pos[i];
	}


	double norm = 0.;
	double vector_mag_squared = 0.;
	int diff_vec_size = difference_vec.size();
	// Compute the sum of squares of each element in the difference vector
	for (int i=0; i < diff_vec_size; i++)
	{
		vector_mag_squared += difference_vec[i]*difference_vec[i];
	}
	norm = sqrt(vector_mag_squared);



	return norm;

}
