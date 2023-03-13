/*
 * @Author: JIAlonglong
 * @Date: 2023-03-08 17:57:32
 * @LastEditors: JIAlonglong 2495477531@qq.com
 * @LastEditTime: 2023-03-11 11:57:21
 * @FilePath: /rc_ws/src/rc_controllers/gimbal_controller/include/gimbal_controller/gimbal_base.h
 * @Description:
 *
 * Copyright (c) 2023 by ${git_name_email}, All Rights Reserved.
 */
#pragma once
#include <effort_controllers/joint_position_controller.h>
#include <controller_manager/controller_manager.h>
#include <controller_interface/controller.h>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <realtime_tools/realtime_publisher.h>
#include <rc_common/hardware_interface/robot_state_interface.h>
#include <rc_common/ros_utilities.h>
#include <rc_common/ori_tool.h>
#include <tf2_eigen/tf2_eigen.h>
#include <dynamic_reconfigure/server.h>
#include <rc_msgs/GimbalCmd.h>
#include <rc_msgs/GimbalDesError.h>
#include <rc_msgs/TrackData.h>
#include <Eigen/Eigen>
#include <boost/scoped_ptr.hpp>

namespace gimbal_controller
{
class Controller : public controller_interface::MultiInterfaceController<rc_control::RobotStateInterface,
                                                                         hardware_interface::ImuSensorInterface,
                                                                         hardware_interface::EffortJointInterface>
{
private:
  // Functions
  bool setDesIntoLimit(double& real_des, double current_des, double base2gimbal_current_des,
                       const urdf::JointConstSharedPtr& joint_urdf);
  void moveJoint(const ros::Time& time, const ros::Duration& period);
  void commandCB(const rc_msgs::GimbalCmdConstPtr& msg);
  void trackCB(const rc_msgs::TrackDataConstPtr& msg);

  std::vector<hardware_interface::JointHandle> joint_handles_;
  effort_controllers::JointPositionController ctrl_yaw_, ctrl_pitch_;
  hardware_interface::ImuSensorHandle imu_sensor_handle_;
  bool has_imu_ = true;
  rc_control::RobotStateHandle robot_state_handle_;
  std::string gimbal_des_frame_id_{}, imu_name_{};

  // Transform
  geometry_msgs::TransformStamped odom2gimbal_des_, odom2pitch_, odom2base_, odom2roll_, last_odom2base_;

  std::shared_ptr<realtime_tools::RealtimePublisher<rc_msgs::GimbalDesError>> error_pub_;

  double publish_rate_{};
  bool state_changed_{};

  // rc_msgs
  rc_msgs::GimbalCmd cmd_gimbal_;
  rc_msgs::TrackData data_track_;

  // ROS interface
  realtime_tools::RealtimeBuffer<rc_msgs::GimbalCmd> cmd_rt_buffer_;
  realtime_tools::RealtimeBuffer<rc_msgs::TrackData> track_rt_buffer_;
  ros::Subscriber cmd_gimbal_sub_;
  ros::Subscriber data_track_sub_;

  enum
  {
    NORMAL,
    DIRECT
  };
  int state_ = NORMAL;

  enum
  {
    Two_Degrees,
    Three_Degrees
  };
  int angle_ = Two_Degrees;

public:
  Controller() = default;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void setDes(const ros::Time& time, double yaw_des, double pitch_des);
  void direct(const ros::Time& time);
  void normal(const ros::Time& time);
  void starting(const ros::Time& time) override;
  void stopping(const ros::Time& time) override;
};

}  // namespace gimbal_controller
