#include "Iir.h"
#include "test_iir_data_path.h"
#include <stdio.h>
#include "assert_print.h"
#include "csv_parser.h"
#include "matplotlibcpp.hpp"
namespace plt = matplotlibcpp;
int main(int, char **)
{
	// create the filter structure for 3rd order
	Iir::Butterworth::LowPass<3> f;
	// filter parameters
	const float samplingrate = 50;	  // Hz
	const float cutoff_frequency = 1; // Hz
	// calc the coefficients
	f.setup(samplingrate, cutoff_frequency);

	std::string file_name = test_iir_data_path + "f_angle.csv";
	io::CSVReader<1> record_csv(file_name);
	record_csv.read_header(io::ignore_extra_column, "tire_f_angle");
	std::vector<double> raw_f_angle_vec;
	std::vector<double> filter_f_angle_vec;
	double f_angle = 0;
	while (record_csv.read_row(f_angle))
	{
		raw_f_angle_vec.push_back(f_angle);
		filter_f_angle_vec.push_back(f.filter(f_angle));
	}
	plt::plot(raw_f_angle_vec);
	plt::plot(filter_f_angle_vec);
	plt::show();
}
