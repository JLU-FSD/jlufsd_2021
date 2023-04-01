#include <nav_msgs/Path.h>

#include "trajectory.h"

namespace planning
{
    nav_msgs::Path   generatePathVisFromPath(const Path & _path);
} // namespace planning