#pragma once
#include "fsa_planning.h"
#include "longitudinal_module.h"
#include "smoother_module.h"

#include <string>


class FSAStraightPlanning : public FSAPlanning
{
    public:
        bool initalize();
        void runAlgorithm();
        void switchStage();

        void constructLine();
        planning::PathPoint findNearestPointInLine();
        void generateTrajectoryFromLine();

        planning::Road                  local_road_;
        IterativePolynomialSmoother     lateral_planner_;
        ForwardBackwardVPlanner         longitudinal_planner_;

        std::vector<planning::Vehicle>  states_recording_;
        double                          line_k_;
        double                          line_b_;
        double                          line_theta_;//弧度
        const double                    unit_length_ = 0.5;
        bool                            has_constructed_line_ = false;
        const int                       points_num_ = 50;
};