#pragma once
#include "planning_common.h"
#include "smoother_common.h"
#include <vector>
#include <math.h>
#include <numeric>

#include <Eigen/Sparse>
#include <Eigen/Cholesky>
#include <Eigen/Dense>
#include <Eigen/Core>

class IterativePolynomialSmoother : public Smoother
{
public:
    void smooth();
private: 
    const int               poly_order_ = 3;//该方法中这个值不能被改变
    int                     param_num_ = poly_order_ + 1;//这是因为在该方法下，由于曲线经过原点，0次系数一定为0
    int                     pieces_num_;
    int                     sum_param_num_;
    double                  precision_ = 0.5;//样条上点的间距
    Eigen::VectorXd         param_x_;
    Eigen::VectorXd         param_y_;

    std::vector<double>     distances_;
    std::vector<double>     new_distances_;
    std::vector<double>     difference_of_distances_;
    double                  threshold_ = precision_ / 2;
    int                     time_threshold_ = 1;
    std::vector<double>     factorial_ = {1,1,2,6,24,120,720,5040,40320,362880,3628800};

    //计算函数
    void calculateHyperparameter();
    void calculatePolynomial();
    void calculatePath();
    void updateDistances();
    planning::PathPoint getSplinePoint(const double & _s);
};