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

    kinematic_state_.reset(new robot_state::RobotState(kinematic_model_));
    //kinematic_state_->setToDefaultValues();
    srdf_model = kinematic_model_->getSRDF();

    return true;
};

bool SrAntropomorphicIndex::check_reachability(tf2::Vector3 position_target, tf2::Quaternion orientation_target){
    
    for (auto group : kinematic_model_ -> getJointModelGroups()){
        if (std::find(move_group_list_.begin(), move_group_list_.end(), group -> getName()) != move_group_list_.end()){
            robots_joint_model_.push_back(group);
        }        
    }

    double position_weight = 1.0;
    double orientation_weight = 0.05;
    double regularisation_weight = 1.0;
    double ik_timeout_  = 0.007;

    bio_ik::BioIKKinematicsQueryOptions kinematic_options_;
    moveit::core::GroupStateValidityCallbackFn constraint_;    

    bool found_ik = false;
    int nb_ik_solutions_found = 0;
    std_msgs::Float64MultiArray current_pos;
    std_msgs::Float64MultiArray ik_pos;

    for (auto model : robots_joint_model_){

        kinematic_options_.goals.clear();
        auto model_eef_list_ = srdf_model->getEndEffectors();
        std::string current_group_tip_link_name;
        for (int j=0; j < model_eef_list_.size(); j++){
            if (model_eef_list_[j].parent_group_ == model->getName()){
                current_group_tip_link_name = model_eef_list_[j].parent_link_;
                continue;
            }
        }

        auto* position_goal = new bio_ik::PositionGoal();
        position_goal->setLinkName(current_group_tip_link_name);
        position_goal->setWeight(position_weight);
        position_goal->setPosition(tf2::Vector3(position_target));
        kinematic_options_.goals.emplace_back(position_goal);

        auto* orientation_goal = new bio_ik::OrientationGoal();
        orientation_goal->setLinkName(current_group_tip_link_name);
        orientation_goal->setWeight(orientation_weight);
        orientation_goal->setOrientation(tf2::Quaternion(orientation_target));
        kinematic_options_.goals.emplace_back(orientation_goal);

        auto* regularization_goal = new bio_ik::MinimalDisplacementGoal();
        regularization_goal->setWeight(regularisation_weight);
        kinematic_options_.goals.emplace_back(regularization_goal);       

        kinematic_options_.replace = true;
        kinematic_options_.return_approximate_solution = true;
        
        // get initial joint state
        kinematic_state_->copyJointGroupPositions(model, current_pos.data);
        
        found_ik = kinematic_state_->setFromIK(model, 
                                               EigenSTL::vector_Isometry3d(),
                                               std::vector<std::string>(), 
                                               ik_timeout_,
                                               moveit::core::GroupStateValidityCallbackFn(),
                                               kinematic_options_);

        // get final joint state
        kinematic_state_->copyJointGroupPositions(model, ik_pos.data);

        std::cout << std::endl << std::endl;
        std::cout << "----- For " << model->getName() << " found IK - " << ((int)found_ik) << std::endl;
        std::cout << "LinkName for PositionGoal: " << current_group_tip_link_name << std::endl;
        std::cout << "Joints in group: ";
        for (auto joint_model : model->getJointModels()){
            std::cout << joint_model -> getName() << " " << *(kinematic_state_->getJointPositions(joint_model)) << std::endl;
        }
        std::cout << std::endl << std::endl;
        std::cout << current_pos << std::endl;
        std::cout << ik_pos << std::endl;
        //kinematic_state_ -> printStateInfo(std::cout);
        
        //kinematic_model_ -> printModelInfo(std::cout);
        if (found_ik){
            nb_ik_solutions_found++;
        }
        break; // just stop after first group - this is only for now to test 
    }  

    if (nb_ik_solutions_found == robots_joint_model_.size()){
        return true;
    }
    
    return false;

}

int main(int argc, char **argv){
    ros::init(argc, argv, "sr_antropomorphic_index_node");
    SrAntropomorphicIndex ai = SrAntropomorphicIndex();
    ai.init();

    /* rh_fftip to rh_palm
    # at top (finger open)
        [0.033, -0.000, 0.191]
        [0.000, 0.000, -0.000, 1.000]

    # bent (halway down))
        - Translation: [0.033, -0.046, 0.146]
        - Rotation: in Quaternion [0.777, 0.001, -0.001, 0.629]
    */

    //tf2::Vector3 pos_goal = tf2::Vector3(0.033, -0.000, 0.191);    
    //tf2::Quaternion orient_goal = tf2::Quaternion(0.000, 0.000, -0.000, 1.000);

    tf2::Vector3 pos_goal = tf2::Vector3(0.033, -0.000, 0.191);    
    tf2::Quaternion orient_goal = tf2::Quaternion(0.000, 0.000, -0.000, 1.000);

    ai.check_reachability(pos_goal, orient_goal);
    
    
    return 0;
}

//   rh_rftip: T.xyz = [-0.011, -0.01, 0.43801], Q.xyzw = [0, 0, 0, 1]
