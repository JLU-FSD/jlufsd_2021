#pragma once
#include "smoother_common.h"

class PolygonalLine : public Smoother
{
public:          
    void smooth();
    double                                  pieces_num_;
    double                                  precision_ = 0.2;
};