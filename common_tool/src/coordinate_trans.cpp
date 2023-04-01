#include "coordinate_trans.h"
#include <cmath>
using namespace std;
Eigen::Matrix3d eular_zxy_to_rot_matrix(double ag_z, double ag_x, double ag_y)
{
	ag_z = ag_z / 180 * 3.1415;
	ag_x = ag_x / 180 * 3.1415;
	ag_y = ag_y / 180 * 3.1415;
	Eigen::Matrix3d rot_matrix;
	rot_matrix << cos(ag_z) * cos(ag_y) - sin(ag_z) * sin(ag_x) * sin(ag_y), -cos(ag_x) * sin(ag_z), cos(ag_z) * sin(ag_y) + cos(ag_y) * sin(ag_z) * sin(ag_x),
		cos(ag_y) * sin(ag_z) + cos(ag_z) * sin(ag_x) * sin(ag_y), cos(ag_z) * cos(ag_x), sin(ag_z) * sin(ag_y) - cos(ag_z) * cos(ag_y) * sin(ag_x),
		-cos(ag_x) * sin(ag_y), sin(ag_x), cos(ag_x) * cos(ag_y);
	return rot_matrix;
}
Eigen::Vector3d vector_trans_source_to_desti_eulr_zxy(double ag_z, double ag_x, double ag_y,
													  const Eigen::Vector3d &source_input_vector)
{
	return eular_zxy_to_rot_matrix(ag_z, ag_x, ag_y).transpose() * source_input_vector;
}
Eigen::Vector3d vector_trans_desti_to_source_eulr_zxy(double ag_z, double ag_x, double ag_y,
													  const Eigen::Vector3d &desti_input_vector)
{
	return eular_zxy_to_rot_matrix(ag_z, ag_x, ag_y) * desti_input_vector;
}
