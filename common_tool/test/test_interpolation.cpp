#include "interpolation_1d.h"
#include <iostream>
#include "interpolation_2d.h"
#include "test_data_path.h"

using namespace auto_ros::common_tool;
int main()
{

	Interpolation1D::DataType xy{{0, 0}, {15, 15}, {30, 30}, {45, 45}};
	Interpolation1D estimator;
	estimator.Init(xy);
	double inter_data = estimator.Interpolate(6.5);

	Interpolation2D::DataType xyz{std::make_tuple(0.3, 0.2, 0.6),
								  std::make_tuple(10.1, 15.2, 5.5),
								  std::make_tuple(20.2, 10.3, 30.5)};

	Interpolation2D estimator2d;
	estimator2d.Init(xyz);
	std::cout << estimator2d.Interpolate(std::make_pair(0.31, 0.2)) << std::endl;
}