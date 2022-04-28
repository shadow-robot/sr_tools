#include "ros/ros.h"
#include <std_msgs/Float64MultiArray.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_interaction/kinematic_options_map.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <geometry_msgs/Pose.h>
#include <bio_ik/bio_ik.h>
#include <yaml-cpp/yaml.h>
#include <Eigen/Core>

#include <math.h>

struct FingerGroupParams{
    std::string name;
    tf2::Vector3 position_target;
    tf2::Quaternion orientation_target;
    double position_weight;
    double orientation_weight;
    double regularisation_weight;
};



class SrAntropomorphicIndex{
    public:
        SrAntropomorphicIndex();
        bool init();
        bool check_reachability(tf2::Vector3 position_target, tf2::Quaternion orientation_target);
        bool check_all_reachability(std::vector<FingerGroupParams> finger_groups);

        robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
        std::vector<robot_model::JointModelGroup*> robots_joint_groups_;
        
        moveit::core::RobotModelPtr kinematic_model_;
        moveit::core::RobotStatePtr kinematic_state_;        
        std::vector<std::string> move_group_list_;

        bio_ik::BioIKKinematicsQueryOptions kinematic_options_;
        moveit::core::GroupStateValidityCallbackFn constraint_;  

    private:
        bool found_ik = false;
        std::string side_prefix_ = "rh_";
        double ik_timeout_ = 0.006;
        double position_weight = 0.00;
        double orientation_weight = 1;
        double regularisation_weight = 1;
        
        std::vector<srdf::Model::EndEffector> model_eef_list_;

};
