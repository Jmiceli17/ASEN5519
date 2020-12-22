/*
 * LogWaveFrontPath.cpp
 *
 *  Created on: Oct 28, 2020
 *      Author: Joe
 */

#include "LogWaveFrontPath.hpp"


//void LogWaveFrontPath(std::vector<WaveFrontCell> &path)
void LogWaveFrontPath(std::vector<std::vector<double>> path)

{
	std::ofstream LOG_FILE("WaveFrontPlanner.csv");
	//LOG_FILE << "X" << "," << "Y" << "," << "Value\n"; 	// Log file for writing x,y position and cell value at each position
	LOG_FILE << "X" << "," << "Y\n";
	for (int c = 0; c<int(path.size()); c++)
	{
		//std::vector<double> pos = path[c].position;
		std::vector<double> pos = path[c];

		//int val = path[c].value;

		//LOG_FILE<< pos[0] << "," << pos[1] << ","<< val << "\n";
		LOG_FILE<< pos[0] << "," << pos[1] << "\n";
	}

	// Close the file stream
	LOG_FILE.close();

}
