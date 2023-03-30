//
// Created by jialonglong on 23-3-29.
//

#pragma once

#include "hardware_interface/joint_command_interface.h"
#include "controller_interface/controller.h"
#include "pluginlib/class_list_macros.h"
#include <ros/ros.h>
#include <std_msgs/Float64.h>

namespace rotation_controller {
    class Controller : public controller_interface::Controller<hardware_interface::EffortJointInterface> {
    public:
        bool init(hardware_interface::EffortJointInterface *robot_hw, ros::NodeHandle &nh) override;

        void update(const ros::Time &time, const ros::Duration &period) override;

        void starting(const ros::Time &time) override;

        void stopping(const ros::Time &time) override;

    private:
        void CommandCallback(const std_msgs::Float64::ConstPtr &msg);

        hardware_interface::JointHandle joint_;
        ros::Subscriber command_sub_;
        double command_;
    };

    PLUGINLIB_EXPORT_CLASS(rotation_controller::Controller, controller_interface::ControllerBase)
}//namespace rotation_controller