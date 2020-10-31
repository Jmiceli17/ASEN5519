#include "TakeStep.hpp"



// std::vector<double>  TakeStep(std::vector<double> init_position, std::vector<double> direction_vector, double step_size)
void  TakeStep(std::vector<double> &position, std::vector<double> direction_vector, double step_size)

{
	std::vector<double> step_vector(direction_vector.size());
	//std::vector<double> new_position(direction_vector.size());
	//std::vector<double> direction_vector_u = UnitVector(direction_vector);
	int dir_vec_size = direction_vector.size();

	for (int i = 0; i<dir_vec_size; i++)
	{
		step_vector[i] = step_size*direction_vector[i];
	}

	for (int i = 0; i<dir_vec_size; i++)
	{
		position[i] += step_vector[i];
	}


}
