//
// Created by jialonglong on 23/3/15
//
//ref:https://github.com/rm-controls
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
#include <rc_common/filters/filters.h>
#include <tf2_eigen/tf2_eigen.h>
#include <dynamic_reconfigure/server.h>
#include <rc_msgs/GimbalCmd.h>
#include <rc_msgs/GimbalDesError.h>
#include <rc_msgs/TrackData.h>
#include <Eigen/Eigen>
#include <boost/scoped_ptr.hpp>

namespace gimbal_controller {
    class Vector3WithFilter {
    public:
        Vector3WithFilter(int num_data) {
            for (int i = 0; i < 3; i++)
                filter_vector_.push_back(std::make_shared<MovingAverageFilter<double>>(num_data));
        }

        void input(double vector[3], double period) {
            for (int i = 0; i < 3; i++) {
                if (period < 0)
                    return;
                if (period > 0.1)
                    filter_vector_[i]->clear();
                filter_vector_[i]->input(vector[i]);
            }
        }

        double x() {
            return filter_vector_[0]->output();
        }

        double y() {
            return filter_vector_[1]->output();
        }

        double z() {
            return filter_vector_[2]->output();
        }

    private:
        std::vector<std::shared_ptr<MovingAverageFilter<double>>> filter_vector_;
    };

    class ChassisVel {
    public:
        ChassisVel(const ros::NodeHandle &nh) {
            double num_data;
            nh.param("num_data", num_data, 20.0);
            nh.param("debug", is_debug_, true);
            linear_ = std::make_shared<Vector3WithFilter>(num_data);
            angular_ = std::make_shared<Vector3WithFilter>(num_data);
            if (is_debug_) {
                real_pub_.reset(new realtime_tools::RealtimePublisher<geometry_msgs::Twist>(nh, "real", 1));
                filtered_pub_.reset(new realtime_tools::RealtimePublisher<geometry_msgs::Twist>(nh, "filtered", 1));
            }
        }

        std::shared_ptr<Vector3WithFilter> linear_;
        std::shared_ptr<Vector3WithFilter> angular_;

        void update(double linear_vel[3], double angular_vel[3], double period) {
            linear_->input(linear_vel, period);
            angular_->input(angular_vel, period);
            if (is_debug_ && loop_count_ % 10 == 0) {
                if (real_pub_->trylock()) {
                    real_pub_->msg_.linear.x = linear_vel[0];
                    real_pub_->msg_.linear.y = linear_vel[1];
                    real_pub_->msg_.linear.z = linear_vel[2];
                    real_pub_->msg_.angular.x = angular_vel[0];
                    real_pub_->msg_.angular.y = angular_vel[1];
                    real_pub_->msg_.angular.z = angular_vel[2];

                    real_pub_->unlockAndPublish();
                }
                if (filtered_pub_->trylock()) {
                    filtered_pub_->msg_.linear.x = linear_->x();
                    filtered_pub_->msg_.linear.y = linear_->y();
                    filtered_pub_->msg_.linear.z = linear_->z();
                    filtered_pub_->msg_.angular.x = angular_->x();
                    filtered_pub_->msg_.angular.y = angular_->y();
                    filtered_pub_->msg_.angular.z = angular_->z();

                    filtered_pub_->unlockAndPublish();
                }
            }
            loop_count_++;
        }

    private:
        bool is_debug_;
        int loop_count_;
        std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::Twist>> real_pub_{}, filtered_pub_{};
    };

    class Controller : public controller_interface::MultiInterfaceController<hardware_interface::ImuSensorInterface,
            hardware_interface::EffortJointInterface> {
    private:
        // Functions
        void direct(const ros::Time &time);

        void normal(const ros::Time &time);

        void rate(const ros::Time &time, const ros::Duration &period);

        bool setDesIntoLimit(double &real_des, double current_des, double base2gimbal_current_des,
                             const urdf::JointConstSharedPtr &joint_urdf);

        void moveJoint(const ros::Time &time, const ros::Duration &period);

        void commandCB(const rc_msgs::GimbalCmdConstPtr &msg);

        void updateChassisVel();

        std::vector<hardware_interface::JointHandle> joint_handles_;
        effort_controllers::JointPositionController ctrl_yaw_, ctrl_pitch_;
        hardware_interface::ImuSensorHandle imu_sensor_handle_;
        bool has_imu_ = true;
        rc_control::RobotStateHandle robot_state_handle_;
        tf2_ros::BufferInterface *tf_buffer_handle_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
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

        //chassis
        std::shared_ptr<ChassisVel> chassis_vel_;

        enum {
            RATE,
            NORMAL,
            DIRECT
        };
        int state_ = RATE;

    public:
        Controller() = default;

        bool
        init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;

        void update(const ros::Time &time, const ros::Duration &period) override;

        void setDes(const ros::Time &time, double yaw_des, double pitch_des);

        void starting(const ros::Time &time) override;
    };

}  // namespace gimbal_controller
