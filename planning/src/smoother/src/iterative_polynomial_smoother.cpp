#include "iterative_polynomial_smoother.h"

void IterativePolynomialSmoother::smooth()
{
    original_traj_.calculateS();
    original_traj_.calculateYaw();
    calculateHyperparameter();
    int time = 0;
    while(1)
    {
        calculatePolynomial();
        updateDistances();
        difference_of_distances_.clear();
        for(int i = 0; i < distances_.size(); i++)
        {
            difference_of_distances_.push_back( abs( new_distances_[i] - distances_[i] ) );
        }
        double max_difference =  *max_element(difference_of_distances_.begin(),difference_of_distances_.end());  
        std::cout<<max_difference<<std::endl;
        if(max_difference < threshold_)
        {
            break;
        }
        else
        {
            distances_ = new_distances_;
        } 
        time++;
        if(time > time_threshold_)
        {
            break;
        }
    }
    calculatePath();
    smoothed_traj_.calculateYaw();
    smoothed_traj_.calculateCur();

    std::vector<double> S;
    S.push_back(0);
    for(int i = 1; i < pieces_num_; i++)
    {
        S.push_back(S[i - 1] + distances_[i]);
    }
    // showInCsv(S, "/home/auto/fsd/src/planning/data/distance.csv");
}

void IterativePolynomialSmoother::calculateHyperparameter()
{
	pieces_num_ = original_traj_.path.size() - 1;
	sum_param_num_ = param_num_ * pieces_num_;
    //计算间距
	distances_.clear();
    for(int i = 0; i < pieces_num_; i++)
    {
        distances_.push_back(original_traj_.path[i+1].s - original_traj_.path[i].s);
    }


}

void IterativePolynomialSmoother::calculatePolynomial()
{
	//初始化函数
	//问题形式为 Ax = b，求解x
	// Eigen::MatrixXd A = Z
    Eigen::MatrixXd A   = Eigen::MatrixXd::Zero(sum_param_num_, sum_param_num_);
	Eigen::VectorXd b_x = Eigen::VectorXd::Zero(sum_param_num_);  
	Eigen::VectorXd b_y = Eigen::VectorXd::Zero(sum_param_num_);  
	double constrain_num = 0;
    //约束1：每段起点终点位置约束--(n-1)个约束
    //起点位置约束
    for(int i = 0; i < pieces_num_; i++)
	{
        A(i, i * param_num_) = 1;
		b_x[i] = original_traj_.path[i].x;
        b_y[i] = original_traj_.path[i].y;
	}
    constrain_num += pieces_num_;
    //终点位置约束
    for(int i = 0; i < pieces_num_; i++)
	{
		for (int j = 0; j <= poly_order_; j++)
		{
			A(constrain_num + i, j + i * param_num_) = pow(distances_[i], j);
		}
		b_x[constrain_num + i] = original_traj_.path[i + 1].x;
        b_y[constrain_num + i] = original_traj_.path[i + 1].y;
	}
	constrain_num += pieces_num_;
    //约束2：连接处一阶导数相等
    for(int k = 1; k < poly_order_; k++)
	{
        for(int i = 0; i < pieces_num_ - 1; i++)
        {
            for (int j = 0; j <= poly_order_; j++)
            {
                for (int j = 0; j <= poly_order_; j++)
                {
                    if(j-k >= 0)
                    {
                        A(constrain_num + i * (poly_order_ - 1) + k-1, j + i * param_num_) 
                            = factorial_[j] / factorial_[j-k] * pow(distances_[i], j-k);
                    }
                }
			    A(constrain_num + i * (poly_order_ - 1) + k-1, k + (i + 1) * param_num_) = -factorial_[k];
            }
        }
    }
    constrain_num += (poly_order_ - 1) * (pieces_num_ - 1);
    //约束3：采用自然边界，终点处一阶导数为0
    // //约束3：采用自然边界，端点处二阶导数为0,目前距离完整约束还差(poly_order_ - 1)个，这些自由度可以使用边界条件补充，也可以采用优化的方法
	// //从高阶到低阶导数，尾首尾首，进行边界条件的补充
	int constrain_order_ = poly_order_ - 1;
	while(1)
	{
		//先进行尾部添加约束
		for (int j = 0; j <= poly_order_; j++)
		{
			if(j - constrain_order_ >= 0)
			{
				A(constrain_num, j + (pieces_num_ - 1) * param_num_) 
					= factorial_[j] / factorial_[j-constrain_order_] * pow(distances_[pieces_num_ - 1], j-constrain_order_);
			}
		}
		constrain_num++;
		if(constrain_num >= sum_param_num_)break;

		//再进行首部添加约束
		A(constrain_num, constrain_order_) = factorial_[constrain_order_];
		constrain_num++;
		if(constrain_num >= sum_param_num_)break;

		//首尾均完成约束添加，进行更小阶数导数的约束添加
		constrain_order_--;
	}
    //对进行求解
	param_x_ = A.inverse() * b_x;
    param_y_ = A.inverse() * b_y;
    // showInCsv(param_x_,"/home/auto/fsd/src/planning/data/param_x_.csv");
    // showInCsv(param_y_,"/home/auto/fsd/src/planning/data/param_x_.csv");
}

void IterativePolynomialSmoother::updateDistances()
{
    new_distances_.clear();
    for(int i = 0; i < pieces_num_; i++)
    {
        planning::Path path;
        double gap = distances_[i] / 10;
        for(double s = 0; s <= distances_[i]; s += gap)
        {
            planning::PathPoint p;
            for(int j = 0; j <= poly_order_; j++)
            {
                p.x += pow(s, j) * param_x_[i * param_num_ + j];
                p.y += pow(s, j) * param_y_[i * param_num_ + j];
            }
            path.path.push_back(p);
        }
        path.calculateS();
        new_distances_.push_back(path.path.back().s);
    }
}

void IterativePolynomialSmoother::calculatePath()
{   
	smoothed_traj_.path.clear();
    double sum_distance = std::accumulate(std::begin(distances_), std::end(distances_), 0.0);
    for(double s = 0; s < sum_distance; s += precision_)
    {
        smoothed_traj_.path.push_back(getSplinePoint(s));
    }
    smoothed_traj_.precision = precision_;
    smoothed_traj_.has_s = true;
	// original_traj_.showInCsv("/home/auto/fsd/src/planning/data/original_traj_.csv");
	// smoothed_traj_.showInCsv("/home/auto/fsd/src/planning/data/smoothed_traj_.csv");
}

planning::PathPoint IterativePolynomialSmoother::getSplinePoint(const double & _s)
{
    //首先判断在哪一段上，并得到在具体段上的坐标
    double s = _s;
    int i = 0;
    for(; i < distances_.size(); i++)
    {
        if(s < distances_[i])
        {
            break;
        }
        else
        {
            s -= distances_[i];
        }
    }
    planning::PathPoint p;
    for(int j = 0; j <= poly_order_; j++)
    {
        p.x += pow(s, j) * param_x_[i * param_num_ + j];
        p.y += pow(s, j) * param_y_[i * param_num_ + j];
        
    }
    p.s = _s;
    return p;
}