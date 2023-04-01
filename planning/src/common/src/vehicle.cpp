#include "vehicle.h"

double planning::distance(const planning::Vehicle & v1, const planning::Vehicle & v2)
{
    return sqrt(pow(v1.x - v2.x, 2) + pow(v1.y - v2.y, 2));
}