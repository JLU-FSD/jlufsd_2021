#pragma once
#include "fsa_planning.h"
#include "longitudinal_module.h"
#include "smoother_module.h"

class FSARacetrackPlanning : public FSAPlanning
{
    public:
        bool initalize();
        void runAlgorithm();
        void switchStage();

        planning::Road                  local_road_;
        planning::Road                  global_road_;
        const bool                      is_adopting_global_method_ = false;
        const int                       local_cone_num_ = 6;
        
    private:
        void generateLocalMapFromGlobalMap();
        int                             lap_num_;
        bool                            is_near_start_;
        int                             no_yellow_high_cone_count_;

        IterativePolynomialSmoother     lateral_planner_;
        ForwardBackwardVPlanner         longitudinal_planner_;  
};