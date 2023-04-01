#pragma once
#include <Eigen/Dense>
namespace auto_ros
{
namespace common_tool
{
Eigen::MatrixXd vec_to_diag_matrix(const std::vector<double> &vec)
{
	int dim = vec.size();
	Eigen::MatrixXd mat = Eigen::MatrixXd::Zero(dim, dim);
	for (int i = 0; i < dim; i++)
	{
		mat(i, i) = vec[i];
	}

	return mat;
}
Eigen::VectorXd vec_to_vectorXd(const std::vector<double> &vec)
{
	int dim = vec.size();

	Eigen::VectorXd vecxd(dim);
	for (int i = 0; i < dim; i++)
	{
		vecxd(i) = vec[i];
	}
	return vecxd;
}
} // namespace common_tool
} // namespace auto_ros