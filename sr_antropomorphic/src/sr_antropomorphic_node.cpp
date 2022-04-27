#include "sr_antropomorphic/sr_antropomorphic.h"
#include <iostream>

SrAntropomorphicIndex::SrAntropomorphicIndex(){};

bool SrAntropomorphicIndex::init(){    
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    ROS_INFO("Initializing kinematic models");

    robot_model_loader_.reset(new robot_model_loader::RobotModelLoader("/robot_description"));
    kinematic_model_ = robot_model_loader_->getModel();

    if (kinematic_model_ == NULL){
        ROS_ERROR("No robot description found");
        return false;
    }
    
    if (private_nh.getParam("move_group_list", move_group_list_)){
        ROS_INFO("Got move_group_list param");
    }else{
        ROS_ERROR("Failed to get param move_group_list");
    }        

    for (auto group : kinematic_model_ -> getJointModelGroups()){
        if (std::find(move_group_list_.begin(), move_group_list_.end(), group -> getName()) != move_group_list_.end()){
            robots_joint_groups_.push_back(group);
        }        
    }
    
    kinematic_state_.reset(new robot_state::RobotState(kinematic_model_));
    srdf_model = kinematic_model_->getSRDF();

    return true;
};

bool SrAntropomorphicIndex::check_reachability(tf2::Vector3 position_target, tf2::Quaternion orientation_target){
    
    double position_weight = 0.99;
    double orientation_weight = 0.01;
    double regularisation_weight = 1.0;
    double ik_timeout_  = 0.007;

    //bio_ik::BioIKKinematicsQueryOptions kinematic_options_;
    kinematic_options_.replace = true;
    kinematic_options_.return_approximate_solution = false;

    moveit::core::GroupStateValidityCallbackFn constraint_;    
    std::string group_tip_link_name;
    
    int nb_ik_solutions_found = 0;
    auto model_eef_list_ = srdf_model->getEndEffectors();

    for (auto model : robots_joint_groups_){
        kinematic_options_.goals.clear();      
        for (int j=0; j < model_eef_list_.size(); j++){
            if (model_eef_list_[j].parent_group_ == model->getName()){
                group_tip_link_name = model_eef_list_[j].parent_link_;
            }
        }

        auto* position_goal = new bio_ik::PositionGoal();
        position_goal->setLinkName(group_tip_link_name);
        position_goal->setWeight(position_weight);
        position_goal->setPosition(tf2::Vector3(position_target));
        kinematic_options_.goals.emplace_back(position_goal);

        auto* orientation_goal = new bio_ik::OrientationGoal();
        orientation_goal->setLinkName(group_tip_link_name);
        orientation_goal->setWeight(orientation_weight);
        orientation_goal->setOrientation(tf2::Quaternion(orientation_target));
        kinematic_options_.goals.emplace_back(orientation_goal);

        auto* regularization_goal = new bio_ik::MinimalDisplacementGoal();
        regularization_goal->setWeight(regularisation_weight);
        kinematic_options_.goals.emplace_back(regularization_goal);       
        
        found_ik = kinematic_state_->setFromIK(model, 
                                               EigenSTL::vector_Isometry3d(),
                                               std::vector<std::string>(), 
                                               ik_timeout_,
                                               moveit::core::GroupStateValidityCallbackFn(),
                                               kinematic_options_);

        const Eigen::Affine3d& ik_position = kinematic_state_->getGlobalLinkTransform(group_tip_link_name);

        auto x = position_target.getX()-ik_position.translation().x();
        auto y = position_target.getY()-ik_position.translation().y();
        auto z = position_target.getZ()-ik_position.translation().z();

        std::cout << "Solution for " << group_tip_link_name << " found(" << ((int)found_ik) << ")" << std::endl;
        if (found_ik){
            std::cout << "-Distance:" << sqrt(x*x+y*y+z*z) << std::endl;    
            nb_ik_solutions_found++;
            for (auto joint_model : model->getJointModels()){
                std::cout << "--" << joint_model -> getName() << " " << 180*(*(kinematic_state_->getJointPositions(joint_model)))/3.1415 << std::endl;
            }
        }
    }  

    if (nb_ik_solutions_found == robots_joint_groups_.size()){
        return true;
    }    
    return false;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "sr_antropomorphic_index_node");
    SrAntropomorphicIndex ai = SrAntropomorphicIndex();
    ai.init();

    // straight wrist (0, 0) and bent finger
    // - Translation: [0.032, -0.056, 0.402]
    // - Rotation: in Quaternion [0.666, -0.005, 0.004, 0.746]

    tf2::Vector3 pos_goal = tf2::Vector3(0.032, -0.056, 0.402);    
    tf2::Quaternion orient_goal = tf2::Quaternion(0.666, -0.005, 0.004, 0.746);

    ai.check_reachability(pos_goal, orient_goal);
    
    return 0;
}
