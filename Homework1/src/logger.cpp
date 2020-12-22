
# include "logger.hpp"

logger::logger(std::string file_name, std::vector<double> &position)
{
	std::ofstream output_stream(file_name);
	std::vector<double> pos_pointer = position;

	// write the column names to file

}

void logger::initialize()
{
	output_stream << "X_Position" << "," << "Y_Position" << "\n";
	output_stream.close();

}


void logger::log_data()
{
	output_stream << pos_pointer[0] << "," << pos_pointer[1] << "\n";
	output_stream.close();
}

logger::~logger()
{
	output_stream.close();
}
