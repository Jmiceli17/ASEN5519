#include "GetDirection.hpp"


//std::vector<double> GetDirectionToGoal(std::vector<double> position, std::vector<double> q_goal)
void GetDirection(std::vector<double> &DirectionVector_Unit, std::vector<double> position, std::vector<double> q_goal)

{
	//double x_start = q_start.x;
	//double y_start = q_start.y;

	//double x_goal = q_goal.x;
	//double y_goal = q_goal.y;

	//double x_start = q_start[0];
	//double y_start = q_start[1];

	//double x_goal = q_goal[0];
	//double y_goal = q_goal[1];

	std::vector<double> DirectionVector(position.size());
	int dir_vec_size = DirectionVector.size();
	for (int i=0; i< dir_vec_size; i++)
	{
		DirectionVector[i] = q_goal[i] - position[i];
	}
	// DirectionVector[0] = x_goal-x_start;
	// DirectionVector[1] = y_goal-y_start;

	// std::vector<double> DirectionVector_Unit;
	double norm = 0.;
	double vector_mag_squared = 0.;
	for (int i=0; i<dir_vec_size; i++)
	{
		vector_mag_squared += DirectionVector[i]*DirectionVector[i];
	}
	norm = sqrt(vector_mag_squared);


	for (int i=0; i<dir_vec_size; i++)
	{
		DirectionVector_Unit[i] = DirectionVector[i]/norm;
	}


}
