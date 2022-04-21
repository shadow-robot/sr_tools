#include "sr_antropomorphic/sr_antropomorphic.h"
#include <iostream>

SrAntropomorphicIndex::SrAntropomorphicIndex(){};

bool SrAntropomorphicIndex::init(){
    
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    ROS_INFO("Initializing kinematic models");

    robot_model_loader_.reset(new robot_model_loader::RobotModelLoader("/robot_description"));
    kinematic_model_ = robot_model_loader_->getModel();
    kinematic_urdf_model_ = robot_model_loader_->getURDF();
    kinematic_srdf_model_ = robot_model_loader_->getSRDF();
    moveit::core::RobotModel robot_model_ = moveit::core::RobotModel(kinematic_urdf_model_, kinematic_srdf_model_);

    if (kinematic_model_ == NULL){
        ROS_ERROR("No robot description found");
        return false;
    }

    if (private_nh.getParam("move_group_list", move_group_list_)){
        ROS_INFO("Got move_group_list param");
    }else{
        ROS_ERROR("Failed to get param move_group_list");
    }

    /*
    for (auto joint : robot_model_.getLinkModels()){
        if (joint -> getParentLinkModel() != 0){
            std::cout << joint -> getName() << "-" << joint -> getParentLinkModel() -> getName() << std::endl;            
        }else{
            std::cout << joint -> getName() << "-" << "No parent" << std::endl;
        }
    }
    */

    bio_ik::BioIKKinematicsQueryOptions kinematic_options_;
    moveit::core::GroupStateValidityCallbackFn constraint_;
    kinematic_options_.goals.clear();

    kinematic_state_.reset(new robot_state::RobotState(kinematic_model_));
    kinematic_state_->setToDefaultValues();
    
    bool found_ik = false;
    std_msgs::Float64MultiArray start_data;
    std_msgs::Float64MultiArray stop_data;

     for (size_t i = 0; i < move_group_list_.size(); i++){
        
        robots_joint_model_.push_back(kinematic_model_->getJointModelGroup(move_group_list_[i]));     

        kinematic_state_->copyJointGroupPositions(robots_joint_model_[i], start_data.data);
        std::cout << start_data << std::endl;
        std::cout << robots_joint_model_[i] << std::endl;

        found_ik = kinematic_state_->setFromIK(robots_joint_model_[i], EigenSTL::vector_Isometry3d(),
                                               std::vector<std::string>(), 0.01, constraint_,
                                               kinematic_options_);

        kinematic_state_->copyJointGroupPositions(robots_joint_model_[i], stop_data.data);
        std::cout << "----------------------" << std::endl;
        std::cout << stop_data << std::endl;
        std::cout << robots_joint_model_[i] << std::endl;
        /*        
        for (auto joint : joint_group -> getJointModelNames()){
            std::cout << "--" << joint << std::endl;
        }*/
    }    

    //kinematic_state_->printStateInfo();

    
    return true;
};

int main(int argc, char **argv){
    ros::init(argc, argv, "sr_antropomorphic_index_node");
    SrAntropomorphicIndex ai = SrAntropomorphicIndex();
    ai.init();
    //int a;
    //std::cin >> a;
    return 0;
}