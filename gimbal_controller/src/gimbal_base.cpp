/*
 * @Author: JIAlonglong
 * @Date: 2023-03-08 17:57:44
 * @LastEditors: JIAlonglong 2495477531@qq.com
 * @LastEditTime: 2023-03-11 12:03:56
 * @FilePath: /rc_ws/src/rc_controllers/gimbal_controller/src/gimbal_base.cpp
 * @Description:
 *
 * Copyright (c) 2023 by ${git_name_email}, All Rights Reserved.
 */
#include "gimbal_controller/gimbal_base.h"
#include <angles/angles.h>
#include <pluginlib/class_list_macros.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace gimbal_controller
{
bool Controller::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
  // get yaw joint and pitch joint or roll joint
  ros::NodeHandle nh_yaw = ros::NodeHandle(controller_nh, "yaw");
  ros::NodeHandle nh_pitch = ros::NodeHandle(controller_nh, "pitch");
  hardware_interface::EffortJointInterface* effort_joint_interface =
      robot_hw->get<hardware_interface::EffortJointInterface>();
  // Two degrees of freedom
  if (!ctrl_yaw_.init(effort_joint_interface, nh_yaw) || !ctrl_pitch_.init(effort_joint_interface, nh_pitch))
    return false;
  robot_state_handle_ = robot_hw->get<rc_control::RobotStateInterface>()->getHandle("robot_state");
  if (!controller_nh.hasParam("imu_name"))
    has_imu_ = false;
  if (has_imu_)
  {
    imu_name_ = getParam(controller_nh, "imu_name", static_cast<std::string>("gimbal_imu"));
    hardware_interface::ImuSensorInterface* imu_sensor_interface =
        robot_hw->get<hardware_interface::ImuSensorInterface>();
    imu_sensor_handle_ = imu_sensor_interface->getHandle(imu_name_);
  }
  else
  {
    ROS_INFO("Param imu_name has not set, use motors' data instead of imu.");
  }

  // Coordinate transformation of the head itself
  gimbal_des_frame_id_ = ctrl_pitch_.joint_urdf_->child_link_name + "_des";
  odom2gimbal_des_.header.frame_id = "odom";
  odom2gimbal_des_.child_frame_id = gimbal_des_frame_id_;
  odom2gimbal_des_.transform.rotation.w = 1.;
  odom2pitch_.header.frame_id = "odom";
  odom2pitch_.child_frame_id = ctrl_pitch_.joint_urdf_->child_link_name;
  odom2pitch_.transform.rotation.w = 1.;
  odom2base_.header.frame_id = "odom";
  odom2base_.child_frame_id = ctrl_yaw_.joint_urdf_->parent_link_name;
  odom2base_.transform.rotation.w = 1.;

  cmd_gimbal_sub_ = controller_nh.subscribe<rc_msgs::GimbalCmd>("command", 1, &Controller::commandCB, this);
  data_track_sub_ = controller_nh.subscribe<rc_msgs::TrackData>("track", 1, &Controller::trackCB, this);
  publish_rate_ = getParam(controller_nh, "publish_rate", 100.);
  error_pub_.reset(new realtime_tools::RealtimePublisher<rc_msgs::GimbalDesError>(controller_nh, "error", 100));
  return true;
}

void Controller::starting(const ros::Time& /*unused*/)
{
  state_ = NORMAL;
  state_changed_ = true;
}

void Controller::update(const ros::Time& time, const ros::Duration& period)
{
  cmd_gimbal_ = *cmd_rt_buffer_.readFromRT();
  data_track_ = *track_rt_buffer_.readFromNonRT();
  try
  {
    odom2pitch_ = robot_state_handle_.lookupTransform("odom", ctrl_pitch_.joint_urdf_->child_link_name, time);
    odom2base_ = robot_state_handle_.lookupTransform("odom", ctrl_yaw_.joint_urdf_->parent_link_name, time);
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
    return;
  }
  if (state_ != cmd_gimbal_.mode)
  {
    state_ = cmd_gimbal_.mode;
    state_changed_ = true;
  }
  switch (state_)
  {
    case NORMAL:
      normal(time);
      break;
    case DIRECT:
      direct(time);
      break;
  }
  moveJoint(time, period);
}

void Controller::moveJoint(const ros::Time& time, const ros::Duration& period)
{
  geometry_msgs::Vector3 gyro, angular_vel_pitch, angular_vel_yaw;
  if (has_imu_)
  {
    gyro.x = imu_sensor_handle_.getAngularVelocity()[0];
    gyro.y = imu_sensor_handle_.getAngularVelocity()[1];
    gyro.z = imu_sensor_handle_.getAngularVelocity()[2];
    try
    {
      tf2::doTransform(gyro, angular_vel_pitch,
                       robot_state_handle_.lookupTransform(ctrl_pitch_.joint_urdf_->child_link_name,
                                                           imu_sensor_handle_.getFrameId(), time));
      tf2::doTransform(gyro, angular_vel_yaw,
                       robot_state_handle_.lookupTransform(ctrl_yaw_.joint_urdf_->child_link_name,
                                                           imu_sensor_handle_.getFrameId(), time));
    }
    catch (tf2::TransformException& e)
    {
      ROS_WARN("%s", e.what());
      return;
    }
  }
  else
  {
    angular_vel_yaw.z = ctrl_yaw_.joint_.getVelocity();
    angular_vel_pitch.y = ctrl_pitch_.joint_.getVelocity();
  }
  geometry_msgs::TransformStamped base_frame2des;
  base_frame2des =
      robot_state_handle_.lookupTransform(ctrl_yaw_.joint_urdf_->parent_link_name, gimbal_des_frame_id_, time);
  double roll_des, pitch_des, yaw_des;  // desired position
  quatToRPY(base_frame2des.transform.rotation, roll_des, pitch_des, yaw_des);

  double yaw_vel_des = 0., pitch_vel_des = 0.;

  geometry_msgs::Point target_pos = data_track_.target_pos;
  geometry_msgs::Vector3 target_vel = data_track_.target_vel;
  tf2::Vector3 target_pos_tf, target_vel_tf;

  try
  {
    geometry_msgs::TransformStamped transform = robot_state_handle_.lookupTransform(
        ctrl_yaw_.joint_urdf_->parent_link_name, data_track_.header.frame_id, data_track_.header.stamp);
    tf2::doTransform(target_pos, target_pos, transform);
    tf2::doTransform(target_vel, target_vel, transform);
    tf2::fromMsg(target_pos, target_pos_tf);
    tf2::fromMsg(target_vel, target_vel_tf);

    yaw_vel_des = target_vel_tf.cross(target_pos_tf).z() / std::pow((target_pos_tf.length()), 2);
    transform = robot_state_handle_.lookupTransform(ctrl_pitch_.joint_urdf_->parent_link_name,
                                                    data_track_.header.frame_id, data_track_.header.stamp);
    tf2::doTransform(target_pos, target_pos, transform);
    tf2::doTransform(target_vel, target_vel, transform);
    tf2::fromMsg(target_pos, target_pos_tf);
    tf2::fromMsg(target_vel, target_vel_tf);
    pitch_vel_des = target_vel_tf.cross(target_pos_tf).y() / std::pow((target_pos_tf.length()), 2);
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
  }

  ctrl_yaw_.setCommand(yaw_des, yaw_vel_des + ctrl_yaw_.joint_.getVelocity() - angular_vel_yaw.z);
  ctrl_pitch_.setCommand(pitch_des, pitch_vel_des + ctrl_pitch_.joint_.getVelocity() - angular_vel_pitch.y);
  ctrl_yaw_.update(time, period);
  ctrl_pitch_.update(time, period);
  ctrl_yaw_.joint_.setCommand(ctrl_yaw_.joint_.getCommand());
  ctrl_pitch_.joint_.setCommand(ctrl_pitch_.joint_.getCommand());
}

void Controller::setDes(const ros::Time& time, double yaw_des, double pitch_des)
{
  tf2::Quaternion odom2base, odom2gimbal_des;
  tf2::Quaternion base2gimbal_des;
  tf2::fromMsg(odom2base_.transform.rotation, odom2base);
  odom2gimbal_des.setRPY(0, pitch_des, yaw_des);
  base2gimbal_des = odom2base.inverse() * odom2gimbal_des;
  double roll_temp, base2gimbal_current_des_pitch, base2gimbal_current_des_yaw;
  quatToRPY(toMsg(base2gimbal_des), roll_temp, base2gimbal_current_des_pitch, base2gimbal_current_des_yaw);
  double pitch_real_des, yaw_real_des;

  if (!setDesIntoLimit(pitch_real_des, pitch_des, base2gimbal_current_des_pitch, ctrl_pitch_.joint_urdf_))
  {
    double yaw_temp;
    tf2::Quaternion base2new_des;
    double upper_limit, lower_limit;
    upper_limit = ctrl_pitch_.joint_urdf_->limits ? ctrl_pitch_.joint_urdf_->limits->upper : 1e16;
    lower_limit = ctrl_pitch_.joint_urdf_->limits ? ctrl_pitch_.joint_urdf_->limits->lower : -1e16;
    base2new_des.setRPY(0,
                        std::abs(angles::shortest_angular_distance(base2gimbal_current_des_pitch, upper_limit)) <
                                std::abs(angles::shortest_angular_distance(base2gimbal_current_des_pitch, lower_limit)) ?
                            upper_limit :
                            lower_limit,
                        base2gimbal_current_des_yaw);
    quatToRPY(toMsg(odom2base * base2new_des), roll_temp, pitch_real_des, yaw_temp);
  }

  if (!setDesIntoLimit(yaw_real_des, yaw_des, base2gimbal_current_des_yaw, ctrl_yaw_.joint_urdf_))
  {
    double pitch_temp;
    tf2::Quaternion base2new_des;
    double upper_limit, lower_limit;
    upper_limit = ctrl_yaw_.joint_urdf_->limits ? ctrl_yaw_.joint_urdf_->limits->upper : 1e16;
    lower_limit = ctrl_yaw_.joint_urdf_->limits ? ctrl_yaw_.joint_urdf_->limits->lower : -1e16;
    base2new_des.setRPY(0, base2gimbal_current_des_pitch,
                        std::abs(angles::shortest_angular_distance(base2gimbal_current_des_yaw, upper_limit)) <
                                std::abs(angles::shortest_angular_distance(base2gimbal_current_des_yaw, lower_limit)) ?
                            upper_limit :
                            lower_limit);
    quatToRPY(toMsg(odom2base * base2new_des), roll_temp, pitch_temp, yaw_real_des);
  }

  odom2gimbal_des_.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(0., pitch_real_des, yaw_real_des);
  odom2gimbal_des_.header.stamp = time;
  robot_state_handle_.setTransform(odom2gimbal_des_, "gimbal_controllers");
}

bool Controller::setDesIntoLimit(double& real_des, double current_des, double base2gimbal_current_des,
                                 const urdf::JointConstSharedPtr& joint_urdf)
{
  double upper_limit, lower_limit;
  upper_limit = joint_urdf->limits ? joint_urdf->limits->upper : 1e16;
  lower_limit = joint_urdf->limits ? joint_urdf->limits->lower : -1e16;
  if ((base2gimbal_current_des <= upper_limit && base2gimbal_current_des >= lower_limit) ||
      (angles::two_pi_complement(base2gimbal_current_des) <= upper_limit &&
       angles::two_pi_complement(base2gimbal_current_des) >= lower_limit))
    real_des = current_des;
  else
    return false;
  return true;
}
void Controller::direct(const ros::Time& time)
{
  geometry_msgs::Point aim_point_odom = cmd_gimbal_.target_pos.point;
  try
  {
    if (!cmd_gimbal_.target_pos.header.frame_id.empty())
      tf2::doTransform(aim_point_odom, aim_point_odom,
                       robot_state_handle_.lookupTransform("odom", cmd_gimbal_.target_pos.header.frame_id,
                                                           cmd_gimbal_.target_pos.header.stamp));
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
  }
  double yaw = std::atan2(aim_point_odom.y - odom2pitch_.transform.translation.y,
                          aim_point_odom.x - odom2pitch_.transform.translation.x);
  double pitch = -std::atan2(aim_point_odom.z - odom2pitch_.transform.translation.z,
                             std::sqrt(std::pow(aim_point_odom.x - odom2pitch_.transform.translation.x, 2) +
                                       std::pow(aim_point_odom.y - odom2pitch_.transform.translation.y, 2)));
  setDes(time, yaw, pitch);
}

void Controller::normal(const ros::Time& time)
{
  double yaw = cmd_gimbal_.yaw_target_pos;
  double pitch = cmd_gimbal_.pitch_target_pos;
  setDes(time, yaw, pitch);
}

void Controller::commandCB(const rc_msgs::GimbalCmdConstPtr& msg)
{
  cmd_rt_buffer_.writeFromNonRT(*msg);
}

void Controller::trackCB(const rc_msgs::TrackDataConstPtr& msg)
{
  if (msg->id == 0)
    return;
  track_rt_buffer_.writeFromNonRT(*msg);
}

}  // namespace gimbal_controller
PLUGINLIB_EXPORT_CLASS(gimbal_controller::Controller, controller_interface::ControllerBase)
