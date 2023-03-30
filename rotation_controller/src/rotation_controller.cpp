//
// Created by jialonglong on 23-3-29.
//
#include "rotation_controller/rotation_controller.h"

namespace rotation_controller {
    bool Controller::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &nh) {
        if (!joint_velocity_ctrl_.init(robot_hw->get<hardware_interface::VelocityJointInterface>(), nh)) {
            ROS_WARN("joint_velocity_joint is failed");
            return false;
        }
        command_sub_ = nh.subscribe<std_msgs::Float64>("command", 1, &Controller::CommandCallback, this);
        return true;
    }

    void Controller::update(const ros::Time &time, const ros::Duration &period) {
       joint_velocity_ctrl_.update(time, period);
    }
    
    void Controller::CommandCallback(const std_msgs::Float64::ConstPtr &msg) {
        command_ = msg->data;
        joint_velocity_ctrl_.command_buffer_.writeFromNonRT(command_);
    }

}//namespace rotation_controller