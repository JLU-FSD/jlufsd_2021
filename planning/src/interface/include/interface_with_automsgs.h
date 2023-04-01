#include "auto_msgs/planning_point.h"
#include "auto_msgs/planning.h"
#include "planning_common.h"

namespace planning {
auto_msgs::planning toAutoMsgs(planning::Path _path);
auto_msgs::planning_point toAutoMsgs(planning::PathPoint _p);

planning::Path fromAutoMsgs(auto_msgs::planning _path);
planning::PathPoint fromAutoMsgs(auto_msgs::planning_point _p);

double distance(auto_msgs::planning_point _point1,auto_msgs::planning_point _point2);
void readFromCsv(auto_msgs::planning & _path, const char* _filename);
void showInCsv(auto_msgs::planning & _path ,const char* _name);
}