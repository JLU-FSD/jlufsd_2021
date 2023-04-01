#include "longitudinal_test.h"
using namespace planning;
int main()
{//测试ForwardBackwardVPlanner
    planning::Road road;
    planning::Path optimized_path;
    readFromCsv(road.center_line, 
      "/home/auto/Experiment/auto_ros/src/planning/road/centerline/racecourse.csv");

    ForwardBackwardVPlanner velocity_planner;
    velocity_planner.setCurrentV(10);
    velocity_planner.process(&road.center_line);
    road.center_line.showInCsv("/home/auto/Experiment/auto_ros/src/planning/debug/result1.csv");

    // ForwardBackwardVPlanner velocity_planner1;
    // velocity_planner1.setCurrentV(10);
    // velocity_planner1.process(&road.center_line);
    // road.center_line.showInCsv("/home/auto/Experiment/auto_ros/src/planning/debug/result2.csv");
    std::cout<< "test over"<<std::endl;
    return 0;
}