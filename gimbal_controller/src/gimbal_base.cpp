//
// Created by jialonglong on 2023/3/15.
//
// ref:https://github.com/rm-controls
#include "gimbal_controller/gimbal_base.h"
#include <angles/angles.h>
#include <pluginlib/class_list_macros.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


namespace gimbal_controller {
    bool
    Controller::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) {
        // create tf buffer and listener
        tf_buffer_.reset(new tf2_ros::Buffer());
        tf_listener_.reset(new tf2_ros::TransformListener(*tf_buffer_));
        // get yaw joint and pitch joint or roll joint
        ros::NodeHandle nh_yaw = ros::NodeHandle(controller_nh, "yaw");
        ros::NodeHandle nh_pitch = ros::NodeHandle(controller_nh, "pitch");
        hardware_interface::EffortJointInterface *effort_joint_interface =
                robot_hw->get<hardware_interface::EffortJointInterface>();
        // Two degrees of freedom
        if (!ctrl_yaw_.init(effort_joint_interface, nh_yaw) || !ctrl_pitch_.init(effort_joint_interface, nh_pitch))
            return false;
//  robot_state_handle_ = robot_hw->get<rc_control::RobotStateInterface>()->getHandle("robot_state");

        if (!controller_nh.hasParam("imu_name"))
            has_imu_ = false;
        if (has_imu_) {
            imu_name_ = getParam(controller_nh, "imu_name", static_cast<std::string>("gimbal_imu"));
            hardware_interface::ImuSensorInterface *imu_sensor_interface =
                    robot_hw->get<hardware_interface::ImuSensorInterface>();
            imu_sensor_handle_ = imu_sensor_interface->getHandle(imu_name_);
        } else {
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
        publish_rate_ = getParam(controller_nh, "publish_rate", 100.);
        error_pub_.reset(new realtime_tools::RealtimePublisher<rc_msgs::GimbalDesError>(controller_nh, "error", 100));
        return true;
    }

    void Controller::starting(const ros::Time & /*unused*/) {
        state_ = RATE;
        state_changed_ = true;
    }

    void Controller::update(const ros::Time &time, const ros::Duration &period) {
        cmd_gimbal_ = *cmd_rt_buffer_.readFromRT();
        data_track_ = *track_rt_buffer_.readFromNonRT();
        try {
//      odom2pitch_ = robot_state_handle_.lookupTransform("base_link", ctrl_pitch_.joint_urdf_->child_link_name, time);//pitch
//    odom2base_ = robot_state_handle_.lookupTransform("base_link", ctrl_yaw_.joint_urdf_->parent_link_name, time);//base_link
            odom2pitch_ = tf_buffer_->lookupTransform("base_link", ctrl_pitch_.joint_urdf_->child_link_name, time,
                                                      ros::Duration(0.1));
            odom2base_ = tf_buffer_->lookupTransform("base_link", ctrl_yaw_.joint_urdf_->parent_link_name, time,
                                                     ros::Duration(0.1));
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            return;
        }
//        bool success = false;
//        while (!success && ros::ok()) {
//            try {
//                odom2pitch_ = tf_buffer_->lookupTransform("odom", ctrl_pitch_.joint_urdf_->child_link_name, time);
//                odom2base_ = tf_buffer_->lookupTransform("odom", ctrl_yaw_.joint_urdf_->parent_link_name, time);
//                success = true;
//            }
//            catch (tf2::TransformException &ex) {
//                ROS_WARN("%s", ex.what());
//            }
//
//            ros::Duration(0.01).sleep();
//        }

//        if (!ros::ok()) {
//            ROS_WARN("ROS is not ok, stop controller");
//            return;
//        }
        updateChassisVel();
        if (state_ != cmd_gimbal_.mode) {
            state_ = cmd_gimbal_.mode;
            state_changed_ = true;
        }
        switch (state_) {
            case RATE:
                rate(time, period);
                break;
            case NORMAL:
                normal(time);
                break;
            case DIRECT:
                direct(time);
                break;
        }
        moveJoint(time, period);

    }

    void Controller::updateChassisVel() {
        double tf_period = odom2base_.header.stamp.toSec() - last_odom2base_.header.stamp.toSec();
        double linear_x = (odom2base_.transform.translation.x - last_odom2base_.transform.translation.x) / tf_period;
        double linear_y = (odom2base_.transform.translation.y - last_odom2base_.transform.translation.y) / tf_period;
        double linear_z = (odom2base_.transform.translation.z - last_odom2base_.transform.translation.z) / tf_period;
        double last_angular_position_x, last_angular_position_y, last_angular_position_z, angular_position_x,
                angular_position_y, angular_position_z;
        quatToRPY(odom2base_.transform.rotation, angular_position_x, angular_position_y, angular_position_z);
        quatToRPY(last_odom2base_.transform.rotation, last_angular_position_x, last_angular_position_y,
                  last_angular_position_z);
        double angular_x = angles::shortest_angular_distance(last_angular_position_x, angular_position_x) / tf_period;
        double angular_y = angles::shortest_angular_distance(last_angular_position_y, angular_position_y) / tf_period;
        double angular_z = angles::shortest_angular_distance(last_angular_position_z, angular_position_z) / tf_period;
        double linear_vel[3]{linear_x, linear_y, linear_z};
        double angular_vel[3]{angular_x, angular_y, angular_z};
        chassis_vel_->update(linear_vel, angular_vel, tf_period);
        last_odom2base_ = odom2base_;
    }

    void Controller::setDes(const ros::Time &time, double yaw_des, double pitch_des) {
        tf2::Quaternion odom2base, odom2gimbal_des;
        tf2::Quaternion base2gimbal_des;
        tf2::fromMsg(odom2base_.transform.rotation, odom2base);
        odom2gimbal_des.setRPY(0, pitch_des, yaw_des);
        base2gimbal_des = odom2base.inverse() * odom2gimbal_des;
        double roll_temp, base2gimbal_current_des_pitch, base2gimbal_current_des_yaw;
        quatToRPY(toMsg(base2gimbal_des), roll_temp, base2gimbal_current_des_pitch, base2gimbal_current_des_yaw);
        double pitch_real_des, yaw_real_des;

        if (!setDesIntoLimit(pitch_real_des, pitch_des, base2gimbal_current_des_pitch, ctrl_pitch_.joint_urdf_)) {
            double yaw_temp;
            tf2::Quaternion base2new_des;
            double upper_limit, lower_limit;
            upper_limit = ctrl_pitch_.joint_urdf_->limits ? ctrl_pitch_.joint_urdf_->limits->upper : 1e16;
            lower_limit = ctrl_pitch_.joint_urdf_->limits ? ctrl_pitch_.joint_urdf_->limits->lower : -1e16;
            base2new_des.setRPY(0,
                                std::abs(
                                        angles::shortest_angular_distance(base2gimbal_current_des_pitch, upper_limit)) <
                                std::abs(angles::shortest_angular_distance(base2gimbal_current_des_pitch, lower_limit))
                                ?
                                upper_limit :
                                lower_limit,
                                base2gimbal_current_des_yaw);
            quatToRPY(toMsg(odom2base * base2new_des), roll_temp, pitch_real_des, yaw_temp);
        }

        if (!setDesIntoLimit(yaw_real_des, yaw_des, base2gimbal_current_des_yaw, ctrl_yaw_.joint_urdf_)) {
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
//        robot_state_handle_.setTransform(odom2gimbal_des_, "gimbal_controller");
        tf_buffer_->setTransform(odom2gimbal_des_, "gimbal_controller", false);

    }

    void Controller::rate(const ros::Time &time, const ros::Duration &period) {
        if (state_changed_) {  // on enter
            state_changed_ = false;
            ROS_INFO("[Gimbal] Enter RATE");
            odom2gimbal_des_.transform.rotation = odom2pitch_.transform.rotation;
            odom2gimbal_des_.header.stamp = time;
//            robot_state_handle_.setTransform(odom2gimbal_des_, "gimbal_controller");
            tf_buffer_->setTransform(odom2gimbal_des_, "gimbal_controller", false);
        } else {
            double roll{}, pitch{}, yaw{};
            quatToRPY(odom2gimbal_des_.transform.rotation, roll, pitch, yaw);
            setDes(time, yaw + period.toSec() * cmd_gimbal_.rate_yaw, pitch + period.toSec() * cmd_gimbal_.rate_pitch);
        }
    }

    void Controller::direct(const ros::Time &time) {
        if (state_changed_) {
            state_changed_ = false;
            ROS_INFO("[Gimbal] Enter DIRECT");
        }
        geometry_msgs::Point aim_point_odom = cmd_gimbal_.target_pos.point;
        try {
            if (!cmd_gimbal_.target_pos.header.frame_id.empty() || ros::Time::waitForValid())
//                tf2::doTransform(aim_point_odom, aim_point_odom,
//                                 robot_state_handle_.lookupTransform("odom", cmd_gimbal_.target_pos.header.frame_id,
//                                                                     cmd_gimbal_.target_pos.header.stamp));
                tf2::doTransform(aim_point_odom, aim_point_odom,
                                 tf_buffer_->lookupTransform("base_link", cmd_gimbal_.target_pos.header.frame_id,
                                                             cmd_gimbal_.target_pos.header.stamp, ros::Duration(0.01)));
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
        }
        double yaw = std::atan2(aim_point_odom.y - odom2pitch_.transform.translation.y,
                                aim_point_odom.x - odom2pitch_.transform.translation.x);
        double pitch = -std::atan2(aim_point_odom.z - odom2pitch_.transform.translation.z,
                                   std::sqrt(std::pow(aim_point_odom.x - odom2pitch_.transform.translation.x, 2) +
                                             std::pow(aim_point_odom.y - odom2pitch_.transform.translation.y, 2)));
        setDes(time, yaw, pitch);
    }

    void Controller::normal(const ros::Time &time) {
        if (state_changed_) {
            state_changed_ = false;
            ROS_INFO("[Gimbal] Enter NORMAL");
        }
        ROS_INFO("[Gimbal] Enter NORMAL");
        double yaw = cmd_gimbal_.yaw_target_pos;
        double pitch = cmd_gimbal_.pitch_target_pos;
        setDes(time, yaw, pitch);
    }

    bool Controller::setDesIntoLimit(double &real_des, double current_des, double base2gimbal_current_des,
                                     const urdf::JointConstSharedPtr &joint_urdf) {
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

    void Controller::moveJoint(const ros::Time &time, const ros::Duration &period) {
        geometry_msgs::Vector3 gyro, angular_vel_pitch, angular_vel_yaw;
        if (has_imu_) {
            gyro.x = imu_sensor_handle_.getAngularVelocity()[0];
            gyro.y = imu_sensor_handle_.getAngularVelocity()[1];
            gyro.z = imu_sensor_handle_.getAngularVelocity()[2];
            try {
//            tf2::doTransform(gyro, angular_vel_pitch,
//                             robot_state_handle_.lookupTransform(ctrl_pitch_.joint_urdf_->child_link_name,
//                                                                 imu_sensor_handle_.getFrameId(), time));
//            tf2::doTransform(gyro, angular_vel_yaw,
//                             robot_state_handle_.lookupTransform(ctrl_yaw_.joint_urdf_->child_link_name,
//                                                                 imu_sensor_handle_.getFrameId(), time));
                tf2::doTransform(gyro, angular_vel_pitch,
                                 tf_buffer_->lookupTransform(ctrl_pitch_.joint_urdf_->child_link_name,
                                                             imu_sensor_handle_.getFrameId(), time,
                                                             ros::Duration(0.01)));
                tf2::doTransform(gyro, angular_vel_yaw,
                                 tf_buffer_->lookupTransform(ctrl_yaw_.joint_urdf_->child_link_name,
                                                             imu_sensor_handle_.getFrameId(), time,
                                                             ros::Duration(0.01)));


            }
            catch (tf2::TransformException &e) {
                ROS_WARN("%s", e.what());
                return;
            }
        } else {
            angular_vel_yaw.z = ctrl_yaw_.joint_.getVelocity();
            angular_vel_pitch.y = ctrl_pitch_.joint_.getVelocity();
        }
        geometry_msgs::TransformStamped base_frame2des;
//    base_frame2des =
//            robot_state_handle_.lookupTransform(ctrl_yaw_.joint_urdf_->parent_link_name, gimbal_des_frame_id_, time);
        base_frame2des =
                tf_buffer_->lookupTransform(ctrl_yaw_.joint_urdf_->parent_link_name, gimbal_des_frame_id_, time,
                                            ros::Duration(0.01));
        double roll_des, pitch_des, yaw_des;  // desired position
        quatToRPY(base_frame2des.transform.rotation, roll_des, pitch_des, yaw_des);

        double yaw_vel_des = 0., pitch_vel_des = 0.;
        if (state_ == RATE) {
            yaw_vel_des = cmd_gimbal_.rate_yaw;
            pitch_vel_des = cmd_gimbal_.rate_pitch;
        } else {
            geometry_msgs::Point target_pos = data_track_.target_pos;
            geometry_msgs::Vector3 target_vel = data_track_.target_vel;
            tf2::Vector3 target_pos_tf, target_vel_tf;

            try {
//            geometry_msgs::TransformStamped transform = robot_state_handle_.lookupTransform(
//                    ctrl_yaw_.joint_urdf_->parent_link_name, data_track_.header.frame_id, data_track_.header.stamp);
                geometry_msgs::TransformStamped transform = tf_buffer_->lookupTransform(
                        ctrl_yaw_.joint_urdf_->parent_link_name, data_track_.header.frame_id, data_track_.header.stamp,
                        ros::Duration(0.01));
                tf2::doTransform(target_pos, target_pos, transform);
                tf2::doTransform(target_vel, target_vel, transform);
                tf2::fromMsg(target_pos, target_pos_tf);
                tf2::fromMsg(target_vel, target_vel_tf);

                yaw_vel_des = target_vel_tf.cross(target_pos_tf).z() / std::pow((target_pos_tf.length()), 2);
//            transform = robot_state_handle_.lookupTransform(ctrl_pitch_.joint_urdf_->parent_link_name,
//                                                            data_track_.header.frame_id, data_track_.header.stamp);
                transform = tf_buffer_->lookupTransform(ctrl_pitch_.joint_urdf_->parent_link_name,
                                                        data_track_.header.frame_id, data_track_.header.stamp,
                                                        ros::Duration(0.01));
                tf2::doTransform(target_pos, target_pos, transform);
                tf2::doTransform(target_vel, target_vel, transform);
                tf2::fromMsg(target_pos, target_pos_tf);
                tf2::fromMsg(target_vel, target_vel_tf);
                pitch_vel_des = target_vel_tf.cross(target_pos_tf).y() / std::pow((target_pos_tf.length()), 2);
            }
            catch (tf2::TransformException &ex) {
                ROS_WARN("%s", ex.what());
            }
        }

        ctrl_yaw_.setCommand(yaw_des, yaw_vel_des + ctrl_yaw_.joint_.getVelocity() - angular_vel_yaw.z);
        ctrl_pitch_.setCommand(pitch_des, pitch_vel_des + ctrl_pitch_.joint_.getVelocity() - angular_vel_pitch.y);
        ctrl_yaw_.update(time, period);
        ctrl_pitch_.update(time, period);
        ctrl_yaw_.joint_.setCommand(ctrl_yaw_.joint_.getCommand());
        ctrl_pitch_.joint_.setCommand(ctrl_pitch_.joint_.getCommand());
    }

    void Controller::commandCB(const rc_msgs::GimbalCmdConstPtr &msg) {
        cmd_rt_buffer_.writeFromNonRT(*msg);
    }

}  // namespace gimbal_controller

PLUGINLIB_EXPORT_CLASS(gimbal_controller::Controller, controller_interface::ControllerBase)
