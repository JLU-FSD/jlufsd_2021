#pragma once
namespace common_tool
{
    class Point2d
    {
        public:
            double x_ = 0;
            double y_ = 0;
        bool IsSame(const Point2d point2d)
        {
            if( (x_==point2d.x_)&&(y_==point2d.y_) )
                return true;
            else
            {
                return false;
            }
            return false;            
        }
        double InnerProduct(Point2d point2d)
        {
            return (x_*point2d.x_+y_*point2d.y_);
        }
        double operator*(const Point2d point2d)
        {
            return InnerProduct(point2d);
        }
        Point2d operator*(const double gain)
        {
            Point2d temp_point2d;
            temp_point2d.x_ = x_*gain;
            temp_point2d.y_ = y_*gain;
            return temp_point2d;
        }
        Point2d operator-(const Point2d point2d)
        {
            Point2d temp_point2d;
            temp_point2d.x_ = x_-point2d.x_;
            temp_point2d.y_ = y_-point2d.y_;
            return temp_point2d;
        }
        Point2d operator+(const Point2d point2d)
        {
            Point2d temp_point2d;
            temp_point2d.x_ = x_+point2d.x_;
            temp_point2d.y_ = y_+point2d.y_;
            return temp_point2d;
        }
    };
    class Point3d
    {
        public:
            double x_ = 0;
            double y_ = 0;
            double z_ = 0;
    };
    class Line2d
    {
        public:
            Point2d start_point_;
            Point2d end_point_;
            Line2d()=delete;
            Line2d(Point2d start_point,Point2d end_point):start_point_(start_point),end_point_(end_point)
            {
            
            }

            bool CalcPendicularPoint(Point2d ref_point,Point2d &pendicular_point,double &lambda)
            {
                if( start_point_.IsSame(end_point_) )
                    return false;
                else
                {
                    Point2d temp_point2d = start_point_ - end_point_;
                    Point2d temp_point2d_1 = start_point_ - ref_point;
                    double temp1 = temp_point2d_1*temp_point2d;
                    double temp2 = temp_point2d*temp_point2d;
                    lambda = temp1/temp2;
                    pendicular_point = (end_point_*lambda)+( start_point_*(1-lambda) );
                    return true;
                }
                return true;
                
            }
    };



}//end common_tool  namespace