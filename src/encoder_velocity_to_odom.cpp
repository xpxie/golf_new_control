//
// Created by parallels on 5/23/18.
//

#include "encoder_velocity_to_odom.h"

namespace golfcar {

    EncoderVelocityToOdom::EncoderVelocityToOdom(ros::NodeHandle nh) : nh_(nh) {
        odom_2d_ = nh.advertise<nav_msgs::Odometry>("/golfcar/odom", 10);
        odom_tf_msg_.header.frame_id = "odom";
        odom_tf_msg_.child_frame_id = "base_link";
    }

	// void EncoderVelocityToOdom::OnInit(ros::NodeHandle nh) {
	// 	nh_ = nh;
	// 	odom_2d_ = nh.advertise<nav_msgs::Odometry>("/golfcar/odom", 10);
    //     odom_tf_msg_.header.frame_id = "odom";
    //     odom_tf_msg_.child_frame_id = "base_link";
	// }


    void EncoderVelocityToOdom::ProcessRawData(const can_msgs::Frame frame) {

            current_time_ = frame.header.stamp;

            if (!initialized_) {
                last_time_ = current_time_;

                odom_2d_msg_.child_frame_id = "base_link";
                odom_2d_msg_.header.frame_id = "odom";

                rotation_matrix_ = Eigen::Matrix3d::Identity();
                global_pos_ = Eigen::Vector3d::Zero();

                odom_2d_msg_.pose.pose.position.z = 0;

                odom_2d_msg_.twist.twist.linear.y = 0;
                odom_2d_msg_.twist.twist.linear.z = 0;
                odom_2d_msg_.twist.twist.angular.x = 0;
                odom_2d_msg_.twist.twist.angular.y = 0;

                initialized_ = true;
            } else {
                Odom2dOutput(frame);
            }

    }

    void EncoderVelocityToOdom::Odom2dOutput(const can_msgs::Frame frame) {
        static double period;
        period = (current_time_ - last_time_).toSec();

        if (period < 0) {
            ROS_ERROR_STREAM("Refer To A Future Frame");
            initialized_ = false;
            return;
        }

        // TODO: reconfigure "counter number this period"
        static double lfront;
        // lfront = static_cast<int16_t >(frame.data[2] | frame.data[3] << 8) / 2.0;
		lfront = static_cast<int8_t>(frame.data[1] - 127);
        static double rfront;
        // rfront = static_cast<int16_t >(frame.data[0] | frame.data[1] << 8) / 2.0;
		rfront = static_cast<int8_t>(frame.data[0] - 127);
        static double lrear;
        // lrear = static_cast<int16_t >(frame.data[4] | frame.data[5] << 8) / 2.0;
		lrear = static_cast<int8_t>(frame.data[2] - 127);
        static double rrear;
        // rrear = static_cast<int16_t >(frame.data[6] | frame.data[7] << 8) / 2.0;
		rrear = static_cast<int8_t>(frame.data[3] - 127);

	//  ROS_ERROR_STREAM("right rear is:" << rrear);
	//  ROS_ERROR_STREAM("left front is:" << lfront);
	//  ROS_ERROR_STREAM("left rear is:" << lrear);
	//  ROS_ERROR_STREAM("right front is:" << rfront);

        // ROS_INFO_STREAM("period: " << period << "\n"
        //                            << "lfront: " << lfront <<"\n"
        //                            << "rfront: " << rfront << "\n"
        //                            << "lrear: " << lrear << "\n"
        //                            << "rrear: " << rrear);

        static double coefficient;
        // coefficient = diameter_ * M_PI / period / counter_num_per_round_;
		coefficient =( 20 / 128.0 * 2.0 * M_PI )* ( diameter_ / 2.0);
		static double lfront_linear_vel;
        lfront_linear_vel = coefficient * lfront;
        static double rfront_linear_vel;
        rfront_linear_vel = coefficient * rfront;
        static double lrear_linear_vel;
        lrear_linear_vel = coefficient * lrear;
        static double rrear_linear_vel;
        rrear_linear_vel = coefficient * rrear;

        // ROS_INFO_STREAM(	"\n"
        //                         	<< "lfront: " << lfront_linear_vel <<"\n"
        //                            << "rfront: " << rfront_linear_vel << "\n"
        //                            << "lrear: " << lrear_linear_vel << "\n"
        //                            << "rrear: " << rrear_linear_vel);


        static Eigen::Vector3d local_vel;
	static Eigen::Vector3d last_local_vel;

	
	local_vel(0) = (lrear_linear_vel + rrear_linear_vel) / 2.0;
	local_vel(2) = (rrear_linear_vel - lrear_linear_vel) / distance_between_wheels_;

	if ( fabs(last_local_vel(0) - local_vel(0)) > 10.0) {
		ROS_ERROR_STREAM("Drop one unreliable value of encoder");
		local_vel = last_local_vel;
	}

	last_local_vel = local_vel;

	odom_2d_msg_.twist.twist.linear.x = local_vel(0);
	odom_2d_msg_.twist.twist.angular.z = local_vel(2);

	static Eigen::Vector3d global_vel;
	global_vel = rotation_matrix_ * local_vel;
	global_pos_ += period * global_vel;

	if (global_pos_(2) > M_PI) {
		global_pos_(2) -= 2 * M_PI;
	} else if (global_pos_(2) < -M_PI) {
		global_pos_(2) += 2 * M_PI;
	}

	rotation_matrix_ = Eigen::AngleAxisd(global_pos_(2), Eigen::Vector3d::UnitZ());
	global_orientation_ = Eigen::Quaterniond(rotation_matrix_);

	odom_2d_msg_.header.stamp = frame.header.stamp;

	last_time_ = odom_2d_msg_.header.stamp;

	odom_2d_msg_.pose.pose.orientation.w = global_orientation_.w();
	odom_2d_msg_.pose.pose.orientation.x = global_orientation_.x();
	odom_2d_msg_.pose.pose.orientation.y = global_orientation_.y();
	odom_2d_msg_.pose.pose.orientation.z = global_orientation_.z();

	odom_2d_msg_.pose.pose.position.x = global_pos_(0);
	odom_2d_msg_.pose.pose.position.y = global_pos_(1);

	odom_2d_msg_.pose.covariance[0] = 0.1;
	odom_2d_msg_.pose.covariance[7] = 0.1;
	odom_2d_msg_.pose.covariance[14] = 1e-6;
	odom_2d_msg_.pose.covariance[21] = 1e-6;
	odom_2d_msg_.pose.covariance[28] = 1e-6;
	odom_2d_msg_.pose.covariance[35] = 0.5;

	odom_2d_msg_.twist.covariance[0] = 0.001;
	odom_2d_msg_.twist.covariance[7] = 1e-6;
	odom_2d_msg_.twist.covariance[14] = 1e-6;
	odom_2d_msg_.twist.covariance[21] = 1e-6;
	odom_2d_msg_.twist.covariance[28] = 1e-6;
	odom_2d_msg_.twist.covariance[35] = 0.05;


	odom_2d_.publish(odom_2d_msg_);

	// publish tf
	odom_tf_msg_.header.stamp = frame.header.stamp;
	odom_tf_msg_.transform.translation.x = global_pos_(0);
	odom_tf_msg_.transform.translation.y = global_pos_(1);

	odom_tf_msg_.transform.rotation.w = global_orientation_.w();
	odom_tf_msg_.transform.rotation.x = global_orientation_.x();
	odom_tf_msg_.transform.rotation.y = global_orientation_.y();
	odom_tf_msg_.transform.rotation.z = global_orientation_.z();

	tf_br_.sendTransform(odom_tf_msg_);
    }

	double EncoderVelocityToOdom::RCfiler(double now_v) {
		double v_filted = K_ * now_v + (1 - K_) * last_v_;
		return v_filted;
	}
}
