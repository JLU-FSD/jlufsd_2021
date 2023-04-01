#ifndef EIGEN_CSV_H
#define EIGEN_CSV_H
#include </usr/include/eigen3/Eigen/Dense>

#include <iostream>  
#include <string>  
#include <vector>  
#include <fstream>  
#include <sstream> 

void showInCsv(Eigen::MatrixXd _matrix ,const char* _name);
void showInCsv(std::vector<double> _vector ,const char* _name);
void showInCsv(std::vector<int> _vector ,const char* _name);

#endif //EIGEN_CSV_H
