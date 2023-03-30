//
// Created by jialonglong on 23-3-29.
//

#pragma once
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_state_interface.h>
#include "velocity_controllers/joint_velocity_controller.h"
#include "hardware_interface/joint_command_interface.h"
#include "controller_interface/controller.h"
#include "pluginlib/class_list_macros.h"
#include <ros/ros.h>
#include <std_msgs/Float64.h>

namespace rotation_controller {
class Controller : public controller_interface::MultiInterfaceController<hardware_interface::VelocityJointInterface,
                                                                        hardware_interface::JointStateInterface> {
    public:
        bool init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &nh) override;
        void update(const ros::Time &time, const ros::Duration &period) override;

    private:
        void CommandCallback(const std_msgs::Float64ConstPtr &msg);
        
        velocity_controllers::JointVelocityController joint_velocity_ctrl_;
        ros::Subscriber command_sub_;
        double command_;
    };

    PLUGINLIB_EXPORT_CLASS(rotation_controller::Controller, controller_interface::ControllerBase)
}//namespace rotation_controller