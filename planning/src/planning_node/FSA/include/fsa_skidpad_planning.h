#pragma once
#include "fsa_planning.h"
#include "longitudinal_module.h"
#include "smoother_module.h"

#include <string>

class FSASkidpadPlanning : public FSAPlanning
{
    public:
        bool initalize();
        void runAlgorithm();
        void switchStage();

        planning::Road                  local_road_;
        IterativePolynomialSmoother     lateral_planner_;
        ForwardBackwardVPlanner         longitudinal_planner_;
        const double                    cone_radius_ = 15.25 / 2;//半径m
        const double                    trajectory_radius_ = 18.25 / 2;//半径m
        bool                            stage_need_to_be_updated_ = false;
        
    private:
        int                             lap_num_;
        planning::PathPoint             left_center;
        planning::PathPoint             right_center;
        bool                            is_near_high_yellow_cone_;
        int                             no_yellow_high_cone_count_ = 0;
        bool stageNeedToBeUpdated();
        void findCircleCenter();
        
        //find yellow cones as a referenece of entrance path or export path
        bool                            has_recieved_yellow_cones_ = false;
        const double                    yellow_cones_distance_ = 3;
        const double                    yellow_cones_distance_error_ = 3 * 0.5;
        std::vector<auto_msgs::Cone>    yellow_cones_;
        bool getYellowCones();

        //Entrance Part
        const double                    entrance_path_length_ = 50;
        const double                    entrance_path_precision_ = 0.5;
        planning::Path                  entrance_path_;
        void getEntranceLine();
        void enter8Shape();

        //8 Shape Part
        std::vector<auto_msgs::Cone>    reference_cones_;
        planning::Path                  circle_trajectory_;
        planning::PathPoint             circle_center_;
        const double                    skidpad_precision_ = 0.5;
        double                          skidpad_angle_precision_ = skidpad_precision_ / trajectory_radius_;
        bool getReferenceCones();
        bool getCircleCenter();
        bool generateCircleTrajectory();
        void run8Shape();

        //Export Part
        const double                    export_path_precision_ = 0.5;
        planning::Path                  export_path_;
        void getExportLine();
        void leave8Shape();

        void generateTrajectoryFrom(planning::Path path);
        void generateTrajectoryFrom(planning::Path current_path, planning::Path next_path);
};