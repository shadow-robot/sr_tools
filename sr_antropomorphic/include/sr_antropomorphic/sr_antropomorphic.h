#include "ros/ros.h"
#include <std_msgs/Float64MultiArray.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <bio_ik/bio_ik.h>
#include <Eigen/Core>

class SrAntropomorphicIndex{
    public:
        SrAntropomorphicIndex();
        bool init();
        bool check_reachability(tf2::Vector3 position_target, tf2::Quaternion orientation_target);

        robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
        std::vector<robot_model::JointModelGroup*> robots_joint_model_;
        std::vector<std::string> allowed_names_;
        
        moveit::core::RobotModelPtr kinematic_model_;
        moveit::core::RobotStatePtr kinematic_state_;
        
        std::vector<moveit::core::LinkModel*> kinematic_chain;
        std::vector<std::string> move_group_list_;

        std::shared_ptr<const srdf::Model> srdf_model;
};
