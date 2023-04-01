#pragma once
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>
Eigen::Matrix3d eular_zxy_to_rot_matrix(double yaw, double pitch, double roll);
Eigen::Vector3d vector_trans_source_to_desti_eulr_zxy(double ag_z, double ag_x, double ag_y,
													  const Eigen::Vector3d &source_input_vector);
Eigen::Vector3d vector_trans_desti_to_source_eulr_zxy(double ag_z, double ag_x, double ag_y,
													  const Eigen::Vector3d &desti_input_vector);