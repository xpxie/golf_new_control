//
// Created by parallels on 5/23/18.
//

#ifndef GOLF_NEW_CONTROL_ENCODER_VELOCITY_TO_ODOM_H
#define GOLF_NEW_CONTROL_ENCODER_VELOCITY_TO_ODOM_H



#include <ros/ros.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "unramlab_msgs.h"
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <memory>
#include <can_msgs/Frame.h>

#include "ramlab_msgs.h"
#ifndef RAMLAB_RELEASE
#include <nav_msgs/Odometry.h>

#endif

using std::shared_ptr;

namespace golfcar
{

    class EncoderVelocityToOdom
    {
    public:
        EncoderVelocityToOdom(ros::NodeHandle nh);
        ~EncoderVelocityToOdom();
        void ProcessRawData(const can_msgs::Frame frame);

    private:
        ros::NodeHandle nh_;
        ros::Publisher odom_2d_;

        ros::Time last_time_, current_time_;

        bool initialized_ = false;

        const double diameter_ = 0.4359;
        const double distance_between_wheels_ = 0.9815;
        const int counter_num_per_round_ = 128;

        nav_msgs::Odometry odom_2d_msg_;
        Eigen::Matrix3d rotation_matrix_;
        Eigen::Vector3d global_pos_;
        Eigen::Quaterniond global_orientation_;

        void Odom2dOutput(const can_msgs::Frame frame);

        ros::Timer serial_reader_;

// tf publisher

        tf2_ros::TransformBroadcaster tf_br_;
#pragma push_macro("geometry_msgs")
#pragma push_macro("TransformStamped")
#undef geometry_msgs
#undef TransformStamped
        geometry_msgs::TransformStamped odom_tf_msg_;
#pragma pop_macro("geometry_msgs")
#pragma pop_macro("TransformStamped")
    };

}


#endif //GOLF_NEW_CONTROL_ENCODER_VELOCITY_TO_ODOM_H
