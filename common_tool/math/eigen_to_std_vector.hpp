#pragma once
#include <Eigen/Dense>
namespace auto_ros
{
namespace common_tool
{

std::vector<double> mat_to_std_vec(const Eigen::MatrixXd &mat)
{
	std::vector<double> vec(mat.data(), mat.data() + mat.rows() * mat.cols());
	return vec;
}
} // namespace common_tool
} // namespace auto_ros