//
// Created by jialonglong on 23-3-29.
//
#include "rotation_controller/rotation_controller.h"

namespace rotation_controller {
    bool Controller::init(hardware_interface::EffortJointInterface *robot_hw, ros::NodeHandle &nh) {
        std::string joint_name;
        if (!nh.getParam("joint_name", joint_name)) {
            ROS_ERROR("Failed to get joint name");
            return false;
        }
        joint_ = robot_hw->getHandle(joint_name);
        command_sub_ = nh.subscribe<std_msgs::Float64>("command", 1, &Controller::CommandCallback, this);
        return true;
    }

    void Controller::update(const ros::Time &time, const ros::Duration &period) {
        joint_.setCommand(command_);
    }

    void Controller::starting(const ros::Time &time) {
        command_ = 0.0;
    }

    void Controller::stopping(const ros::Time &time) {
        command_ = 0.0;
    }

    void Controller::CommandCallback(const std_msgs::Float64::ConstPtr &msg) {
        command_ = msg->data;
    }

}//namespace rotation_controller
