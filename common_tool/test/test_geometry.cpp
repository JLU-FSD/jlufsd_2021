#include "geometry.hpp"
#include "gtest/gtest.h"
TEST(_geometry, geometry_pendicular_point_Test)
{
        common_tool::Point2d temp_point_start;
        temp_point_start.x_=1;
        temp_point_start.y_=0;
        common_tool::Point2d temp_point_end;
        temp_point_end.x_=0;
        temp_point_end.y_=1;
        common_tool::Point2d current_point;
        current_point.x_=0;
        current_point.y_=0;
        common_tool::Point2d pendicular_point;
        double lambda;
        common_tool::Line2d line2d(temp_point_start,temp_point_end);
        line2d.CalcPendicularPoint(current_point,pendicular_point,lambda);
        common_tool::Point2d expect_point;
        expect_point.x_=0.5;
        expect_point.y_=0.5;
        double expect_lambda =0.5;
        EXPECT_EQ(pendicular_point.x_,0.5);
}
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}