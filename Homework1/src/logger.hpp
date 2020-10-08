/*
 * logger.hpp
 *
 *  Created on: Sep 21, 2020
 *      Author: Joe
 */


#ifndef LOGGER_HPP_
#define LOGGER_HPP_
#include <vector>
#include <cmath>
#include <iostream>
#include <windows.h>
#include <fstream>

using namespace std;

class logger
{
public:
	std::ofstream output_stream;
	std::vector<double> pos_pointer;


	logger(std::string file_name, std::vector<double> &position);
	~logger();

	void initialize();

	void log_data();

};







#endif /* LOGGER_HPP_ */
