#include "common_tool/csv_writer.h"
#include "common_tool/csv_parser.h"
#include "can_config_data_path.h"
#include "can_config_data_path.h"
#include <cmath>
int main()
{
	double speed = 10;
	io::CSVReader<3> in(can_config_data_path + "yifulou_around.csv");
	io::CSVWriter resample_writer(can_config_data_path + "no_closed_resample.csv");
	in.read_header(io::ignore_extra_column, "x", "y", "theta");
	double x, y, theta;
	double sm_delta_s = 0.5;
	double local_sum_s;
	double sum_s = 0;
	//variable to last point
	bool is_closed = false;
	double last_x, last_y, last_theta;
	int pop_num = 10;
	std::vector<double> resample_x, resample_y, resample_theta, resample_s;
	while (in.read_row(x, y, theta))
	{
		static int index = 0;

		if (index == 0)
		{
			local_sum_s = 0;
			last_x = x;
			last_y = y;
			last_theta = theta;
			resample_x.push_back(x);
			resample_y.push_back(y);
			resample_theta.push_back(theta);
			resample_s.push_back(sum_s);
		}
		else
		{
			local_sum_s += std::pow(std::pow(x - last_x, 2) + std::pow(y - last_y, 2), 0.5);
			sum_s += std::pow(std::pow(x - last_x, 2) + std::pow(y - last_y, 2), 0.5);
			if (local_sum_s >= sm_delta_s)
			{
				resample_x.push_back(x);
				resample_y.push_back(y);
				resample_theta.push_back(theta);
				resample_s.push_back(sum_s);
				local_sum_s = 0;
			}
			last_x = x;
			last_y = y;
		}
		index += 1;
	}
	for (int i = 0; i < pop_num; i++)
	{
		resample_x.pop_back();
		resample_y.pop_back();
		resample_theta.pop_back();
	}
	if (is_closed)
	{
		resample_x.push_back(resample_x[0]);
		resample_y.push_back(resample_y[0]);
		resample_theta.push_back(resample_theta[0]);
	}
	resample_writer.write_row<std::string>({"x", "y", "theta", "kappa", "v", "s", "a"});
	for (int i = 0; i < resample_x.size(); i++)
	{
		resample_writer.write_row<double>({resample_x[i], resample_y[i], resample_theta[i], 0, 10, resample_s[i], 0});
	}
}