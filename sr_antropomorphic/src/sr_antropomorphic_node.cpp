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
        for (auto group : kinematic_model_ -> getJointModelGroups()){
            if (std::find(move_group_list_.begin(), move_group_list_.end(), group -> getName()) != move_group_list_.end()){
                robots_joint_groups_.push_back(group);
            }        
        }
    }else{
        ROS_ERROR("Failed to get param move_group_list");
    }        

    model_eef_list_ = kinematic_model_->getSRDF()->getEndEffectors();
    kinematic_state_.reset(new robot_state::RobotState(kinematic_model_));    
    kinematic_options_.replace = true;
    kinematic_options_.return_approximate_solution = false;

    return true;
};

bool SrAntropomorphicIndex::check_all_reachability(std::vector<FingerGroupParams> finger_groups){
    int nb_ik_solutions_found = 0;
    for (FingerGroupParams group : finger_groups){
        //std::cout << "Group name:" << group.name << std::endl;
        //std::cout << "-" << "position target:[" << group.position_target.getX() << "," << group.position_target.getY() << "," << group.position_target.getZ() << "]" << std::endl;
        //std::cout << "-" << "orientation target:[" << group.orientation_target.getX() << "," << group.orientation_target.getY() << "," << group.orientation_target.getZ() << "," << group.orientation_target.getW() << "]" << std::endl;

        std::string group_tip_link_name;
        kinematic_options_.goals.clear();      
        for (int j=0; j < model_eef_list_.size(); j++){
            if (model_eef_list_[j].parent_group_ == group.name){
                group_tip_link_name = model_eef_list_[j].parent_link_;
            }
        }

        if (group.position_weight > 0){
            auto* position_goal = new bio_ik::PositionGoal();
            position_goal->setLinkName(group_tip_link_name);
            position_goal->setWeight(group.position_weight);
            position_goal->setPosition(tf2::Vector3(group.position_target));
            kinematic_options_.goals.emplace_back(position_goal);
        }
        if (group.orientation_weight > 0){
            auto* orientation_goal = new bio_ik::OrientationGoal();
            orientation_goal->setLinkName(group_tip_link_name);
            orientation_goal->setWeight(group.orientation_weight);
            orientation_goal->setOrientation(tf2::Quaternion(group.orientation_target));
            kinematic_options_.goals.emplace_back(orientation_goal);
        }
        if (group.regularisation_weight > 0){
            auto* regularization_goal = new bio_ik::MinimalDisplacementGoal();
            regularization_goal->setWeight(group.regularisation_weight);
            kinematic_options_.goals.emplace_back(regularization_goal);     
        }  

        bool found_ik = false;
        found_ik = kinematic_state_->setFromIK(kinematic_model_->getJointModelGroup(group.name), 
                                               EigenSTL::vector_Isometry3d(),
                                               std::vector<std::string>(), 
                                               ik_timeout_,
                                               moveit::core::GroupStateValidityCallbackFn(),
                                               kinematic_options_);

        if(found_ik){
            nb_ik_solutions_found++;
            std::cout << "Found solution" << std::endl;
        }else{
            std::cout << "Not found!" << std::endl;
        }
    }
    return nb_ik_solutions_found == finger_groups.size();
}

int main(int argc, char **argv){
    ros::init(argc, argv, "sr_antropomorphic_index_node");
    SrAntropomorphicIndex ai;
    ai.init();

    std::vector<FingerGroupParams> finger_group_goals;
    YAML::Node config = YAML::LoadFile("/home/user/projects/shadow_robot/base/src/sr_tools/sr_antropomorphic/config/test.yaml");

    if (config["grasps"]) {
        for (std::string finger_group : ai.move_group_list_){
            auto position = config["grasps"]["large_diameter"][finger_group]["position"].as<std::vector<double>>();;
            auto orientation = config["grasps"]["large_diameter"][finger_group]["orientation"].as<std::vector<double>>();;
            tf2::Vector3 pos = tf2::Vector3(position[0], position[1], position[2]);
            tf2::Quaternion orient = tf2::Quaternion(orientation[0], orientation[1], orientation[2], orientation[3]);
            FingerGroupParams finger_group_param = {finger_group, pos, orient, 1, 0.01, 0.01};
            finger_group_goals.push_back(finger_group_param);            
        }
        ai.check_all_reachability(finger_group_goals);
    } 
    
    return 0;
}


/*
    // straight wrist (0, 0) and bent finger
    // - Translation: [0.032, -0.056, 0.402]
    // - Rotation: in Quaternion [0.666, -0.005, 0.004, 0.746]

    tf2::Matrix3x3 matrix = tf2::Matrix3x3(0.98807, 0.0018493, -0.15398, -0.059824, 0.92599, -0.37277, 0.14189, 0.37753, 0.91506);           
    tf2::Quaternion quat;
    matrix.getRotation(quat);
    std::cout<< quat.getX() << " " << quat.getY() << " " << quat.getZ() << " " << quat.getW();
    tf2::Vector3 pos_goal = tf2::Vector3(0.032, -0.056, 0.402);    
    tf2::Quaternion orient_goal = tf2::Quaternion(0.666, -0.005, 0.004, 0.746);

    //ai.check_reachability(pos_goal, orient_goal);
    //pos_goal = tf2::Vector3(0.032, -0.056, 0.403);    
    //orient_goal = tf2::Quaternion(0.666, -0.005, 0.004, 0.746);
    //ai.check_reachability(pos_goal, orient_goal);
*/