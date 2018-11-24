/**
 ******************************************************************************
 * Copyright (C) 2018 Joey.Liu <lty2226262@gmail.com>
 * Distributed under terms of the MIT license.
 ******************************************************************************
 */

#include <can_msgs/Frame.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <bitset>
#include <memory>
#include "can_bridge.h"

#undef UINT32
#include "encoder_velocity_to_odom.h"
#include "ramlab_msgs.h"

#ifndef RAMLAB_RELEASE
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt8.h>
#endif

namespace golfmiddleware {
class GolfCanParser : public nodelet::Nodelet {
 public:
  GolfCanParser(){};

 private:
  virtual void onInit();
  ros::NodeHandle nh_;
  // ros::Subscriber honk_sub_, highlight_sub_;
  // ros::Subscriber left_signal_light_sub_, controller_sub_;
  // ros::Subscriber right_signal_light_sub_, brake_light_sub_;
  ros::Subscriber cmd_vel_sub_, brake_sub_;

  ros::Publisher battery_pub_, error_state_pub_;
  ros::Publisher angle_pub_;
  ros::Publisher sas_pub_;

  //	typedef enum{
  //		HONK = 0,
  //		HIGHLIGHT,
  //		LEFT_SIGNAL_LIGHT,
  //		RIGHT_SIGNAL_LIGHT,
  //		BRAKE_LIGHT,
  //		CONTROLLER,
  //		REAR_LEFT_SIGNAL_LIGHT,
  //		REAR_RIGHT_SIGNAL_LIGHT,
  //		WIDTH_LIGHT
  //	} IoType;

  // std::string io_names_[9] = {
  // 		"HONK",
  // 		"HIGHLIGHT",
  // 		"LEFT_SIGNAL_LIGHT",
  // 		"RIGHT_SIGNAL_LIGHT",
  // 		"BRAKE_LIGHT",
  // 		"CONTROLLER",
  // 		"REAR_LEFT_SIGNAL_LIGHT",
  // 		"REAR_RIGHT_SIGNAL_LIGHT",
  // 		"WIDTH_LIGHT"
  // };

  // int io_byte_dot_bit_[9][2] = {
  // 		{0,1}, //HONK
  // 		{1,7}, //HIGHLIGHT
  // 		{1,5}, //LEFT_SIGNAL_LIGHT
  // 		{1,6}, //RIGHT_SIGNAL_LIGHT
  // 		{1,0}, //BRAKE_LIGHT
  // 		{0,1}, //CONTROLLER
  // 		{0,5}, //REAR_LEFT_SIGNAL_LIGHT,
  // 		{0,6}, //REAR_RIGHT_SIGNAL_LIGHT
  // 		{0,7}, //WIDTH_LIGHT
  // };

  // const int io_id_ = 0x610;
  const int sas_id_ = 0x380;
  const int speed_id_ = 0x1F1;

  int car_max_motor_speed_ = 5500;
  int car_max_motor_speed_reverse_ = 5500;

  double heartbeat_cycle_ = 0.05;
  double trans_receive_cycle_ = 0.005;
  // adjust finished

  // void GeneralHandler(std_msgs::BoolConstPtr msg, const IoType& io_type);
  void CmdVelHandler(geometry_msgs::TwistConstPtr msg);
  void BrakeHandler(std_msgs::BoolConstPtr msg);

  // void SetBit(const IoType& io_type);
  // void ResetBit(const IoType& io_type);

  // can_msgs::Frame io_frame_;
  can_msgs::Frame sas_frame_;
  can_msgs::Frame speed_frame_;

  double max_speed_reciprocal_ = 0.0;
  double max_speed_reciprocal_reverse_ = 0.0;
  bool brake_state_ = false;

  void SetForceBrake();
  std::vector<can_msgs::Frame> can_to_send_buffer_;
  std::mutex can_buffer_mutex_;
  ros::Timer heartbeat_timer_, transmit_timer_, receive_timer_;
  void HeartBeatCb(const ros::TimerEvent &);
  void TransmitCb(const ros::TimerEvent &);
  void ReceiveCb(const ros::TimerEvent &);

  std::shared_ptr<cantools::CanBridge> can_bridge_ptr_;
  vector<can_msgs::Frame> to_read_buffer_;

  bool initialize_finished_ = false;
  // void ComputeCrcCheck(void);
  double last_linear_x_;
  int acc_flag_ = 0;

  // add for new golf car

  std::shared_ptr<golfcar::EncoderVelocityToOdom> encoder_calculator_ptr_;
};
}  // namespace golfmiddleware

PLUGINLIB_EXPORT_CLASS(golfmiddleware::GolfCanParser, nodelet::Nodelet);

namespace golfmiddleware {
void GolfCanParser::onInit() {
  nh_ = getPrivateNodeHandle();
  // honk_sub_ = nh_.subscribe<std_msgs::Bool>("/decision/honk", 1,
  //										  boost::bind(&GolfCanParser::GeneralHandler,
  //this, _1, HONK)); highlight_sub_  =
  // nh_.subscribe<std_msgs::Bool>("/decision/highlight", 1,
  //												boost::bind(&GolfCanParser::GeneralHandler,
  //this, _1, HIGHLIGHT)); left_signal_light_sub_ =
  // nh_.subscribe<std_msgs::Bool>("/decision/left_signal_light", 1,
  //													   boost::bind(&GolfCanParser::GeneralHandler,
  //this, _1, LEFT_SIGNAL_LIGHT)); right_signal_light_sub_ =
  // nh_.subscribe<std_msgs::Bool>("/decision/right_signal_light",1,
  //														boost::bind(&GolfCanParser::GeneralHandler,
  //this, _1, RIGHT_SIGNAL_LIGHT)); brake_light_sub_ =
  // nh_.subscribe<std_msgs::Bool>("/decision/brake_light",1,
  //												 boost::bind(&GolfCanParser::GeneralHandler,
  //this, _1, BRAKE_LIGHT)); controller_sub_ =
  // nh_.subscribe<std_msgs::Bool>("/decision/controller",1,
  //												boost::bind(&GolfCanParser::GeneralHandler,
  //this, _1, CONTROLLER));
  cmd_vel_sub_ = nh_.subscribe("/decision/cmd_vel", 1,
                               &GolfCanParser::CmdVelHandler, this);
  brake_sub_ =
      nh_.subscribe("/decision/brake", 1, &GolfCanParser::BrakeHandler, this);
  battery_pub_ = nh_.advertise<std_msgs::Float32>("/battery", 1);
  error_state_pub_ = nh_.advertise<std_msgs::UInt8>("/error_state", 1);
  angle_pub_ =
      nh_.advertise<geometry_msgs::Vector3Stamped>("/steering_wheel_angle", 1);
  sas_pub_ = nh_.advertise<std_msgs::Float32>("/steering_wheel_angle_sas", 1);
  nh_.param<int>("car_max_motor_speed", car_max_motor_speed_, 5500);
  nh_.param<int>("car_max_motor_speed_reverse", car_max_motor_speed_reverse_,
                 5500);

  //		io_frame_.id = io_id_;
  //		io_frame_.header.frame_id = "io_control";
  //		for (auto& i:io_frame_.data)
  //		{
  //			i = 0;
  //		}
  //        ComputeCrcCheck();

  speed_frame_.id = speed_id_;
  speed_frame_.header.frame_id = "speed_control";
  speed_frame_.data = {0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

  // sas frame init
  sas_frame_.id = sas_id_;
  sas_frame_.header.frame_id = "sas_control";
  sas_frame_.data = {1, 1, 0, 0, 0, 0, 0, 0};

  // io_frame init end

  max_speed_reciprocal_ =
      1.0 / (static_cast<double>(car_max_motor_speed_) / 3000.0 * 6.481481481) *
      100.0;
  max_speed_reciprocal_reverse_ =
      -1.0 /
      (static_cast<double>(car_max_motor_speed_reverse_) / 3000.0 *
       6.481481481) *
      100.0;
  {
    std::unique_lock<std::mutex> lock(can_buffer_mutex_);
    can_to_send_buffer_.clear();
  }

  heartbeat_timer_ = nh_.createTimer(ros::Duration(heartbeat_cycle_),
                                     &GolfCanParser::HeartBeatCb, this);
  receive_timer_ = nh_.createTimer(ros::Duration(trans_receive_cycle_),
                                   &GolfCanParser::ReceiveCb, this);
  transmit_timer_ = nh_.createTimer(ros::Duration(trans_receive_cycle_),
                                    &GolfCanParser::TransmitCb, this);

  can_bridge_ptr_ =
      std::shared_ptr<cantools::CanBridge>(new cantools::CanBridge());
  can_bridge_ptr_->CanOpen();

  encoder_calculator_ptr_ = shared_ptr<golfcar::EncoderVelocityToOdom>(
      new golfcar::EncoderVelocityToOdom(nh_));

  initialize_finished_ = true;
}

// void GolfCanParser::GeneralHandler(std_msgs::BoolConstPtr msg, const IoType&
// io_type)
// {
// 			static int left_count = 0;
// 			static int right_count = 0;
// 	if (true == msg->data)
// 	{
// 			if (io_type != LEFT_SIGNAL_LIGHT && io_type !=
// RIGHT_SIGNAL_LIGHT)
// 			{
// 				SetBit(io_type);
// 				}
// 			if (io_type == LEFT_SIGNAL_LIGHT)
// 			{
// 					if (++left_count == 1)
// 					{
// 							SetBit(REAR_LEFT_SIGNAL_LIGHT);
// 				SetBit(io_type);
// 					} else if (left_count == 4)
// 					{
// 							ResetBit(REAR_LEFT_SIGNAL_LIGHT);
// 				ResetBit(io_type);
// 					} else if (left_count == 5)
// 					{
// 							left_count = 0;
// 					}
// 			}
// 			if (io_type == RIGHT_SIGNAL_LIGHT)
// 			{
// 					if (++right_count == 1)
// 					{
// 				SetBit(io_type);
// 							SetBit(REAR_RIGHT_SIGNAL_LIGHT);
// 					} else if (right_count == 4)
// 					{
// 				ResetBit(io_type);
// 							ResetBit(REAR_RIGHT_SIGNAL_LIGHT);
// 					} else if (right_count == 5)
// 					{
// 							right_count = 0;
// 					}
// 			}
// 			if (io_type == HIGHLIGHT)
// 			{
// 					SetBit(WIDTH_LIGHT);
// 			}
// 	}
// 	else if (false == msg->data)
// 	{
// 			if (io_type != LEFT_SIGNAL_LIGHT && io_type !=
// RIGHT_SIGNAL_LIGHT)
// 			{
// 					ResetBit(io_type);
// 			}
// 			if (io_type == RIGHT_SIGNAL_LIGHT)
// 			{
// 					right_count = 0;
// 					ResetBit(REAR_RIGHT_SIGNAL_LIGHT);
// 					ResetBit(io_type);
// 			}
// 			if (io_type == LEFT_SIGNAL_LIGHT)
// 			{
// 					left_count = 0;
// 					ResetBit(REAR_LEFT_SIGNAL_LIGHT);
// 					ResetBit(io_type);
// 			}
// 			if (io_type == HIGHLIGHT)
// 			{
// 					ResetBit(WIDTH_LIGHT);
// 			}
// 	}
// }

// void GolfCanParser::SetBit(const IoType &io_type)
// {
// 		//ROS_INFO_STREAM(io_names_[io_type] << " ON");
// 		io_frame_.data[io_byte_dot_bit_[io_type][0]] |= (1 <<
// io_byte_dot_bit_[io_type][1]); 		ComputeCrcCheck();
// 		//		v{
// 		//			std::unique_lock<std::mutex>
// lock(can_buffer_mutex_);
// 		// can_to_send_buffer_.push_back(io_frame_);
// 		//		}
// }

// void GolfCanParser::ResetBit(const IoType &io_type)
// {
// 		//ROS_INFO_STREAM(io_names_[io_type] << "OFF");
// 		io_frame_.data[io_byte_dot_bit_[io_type][0]] &= ~(1 <<
// io_byte_dot_bit_[io_type][1]); 		ComputeCrcCheck();
// 		//		{
// 		//			std::unique_lock<std::mutex>
// lock(can_buffer_mutex_);
// 		// can_to_send_buffer_.push_back(io_frame_);
// 		//		}
//}

void GolfCanParser::CmdVelHandler(geometry_msgs::TwistConstPtr msg) {
  static float linear_x, angular_z;
  if (!brake_state_) {
    linear_x = msg->linear.x;
    angular_z = msg->angular.z / 35 * 700;

    if (linear_x >= 0) {
      if ((linear_x - last_linear_x_ > 0) && acc_flag_ != 1) {
        acc_flag_ = 1;
        // ResetBit(BRAKE_LIGHT);
      } else if ((linear_x - last_linear_x_ < 0) && acc_flag_ != -1) {
        acc_flag_ = -1;
        // SetBit(BRAKE_LIGHT);
      } else if ((linear_x - last_linear_x_ == 0) && linear_x == 0 &&
                 acc_flag_ != -1) {
        acc_flag_ = -1;
        // SetBit(BRAKE_LIGHT);
      } else if ((linear_x - last_linear_x_ == 0) && linear_x > 0 &&
                 acc_flag_ != 1) {
        acc_flag_ = 1;
        // ResetBit(BRAKE_LIGHT);
      }
    } else {
      if (-1 != acc_flag_) {
        // SetBit(BRAKE_LIGHT);
      }
    }

    last_linear_x_ = linear_x;

    static double min_tolerance = 1.0 / max_speed_reciprocal_;
    static double min_walk = 0.12;
    // ROS_INFO_STREAM("min_tole:" << min_tolerance);

    if (linear_x > min_tolerance) {
      linear_x = (linear_x > min_walk) ? linear_x : min_walk;
      speed_frame_.data[0] = 0x05;
      speed_frame_.data[1] = static_cast<uint8_t>(
          std::min((linear_x * max_speed_reciprocal_), 100.0));
      //	ROS_INFO_STREAM("linear speed: " << linear_x);
      //	ROS_INFO_STREAM("parameter is " << max_speed_reciprocal_);
      // ROS_INFO_STREAM("speed_is_" << static_cast<int>(static_cast<uint8_t
      // >(std::min((linear_x * max_speed_reciprocal_), 100.0))));
      // ROS_INFO_STREAM("max_speed_is_" << max_speed_reciprocal_);
      speed_frame_.data[2] = 0x00;
      speed_frame_.data[3] = 0x00;
      speed_frame_.data[4] = 0x00;
      speed_frame_.data[5] = 0x00;
      // end accelerate time, decelerate time, steering angle
      speed_frame_.data[6] = 0;
      speed_frame_.data[7] = 0;
    } else if (fabs(linear_x) <= min_tolerance) {
      speed_frame_.data[0] = 0x08;
      speed_frame_.data[1] = 0x00;
      speed_frame_.data[2] = 0x00;
      speed_frame_.data[3] = 0x00;
      speed_frame_.data[4] = 0x00;
      speed_frame_.data[5] = 0x00;
      speed_frame_.data[6] = 0x00;
      speed_frame_.data[7] = 0x00;

    } else {
      linear_x = (linear_x < -min_walk) ? linear_x : (-min_walk);
      speed_frame_.data[0] = 0x06;
      speed_frame_.data[1] = static_cast<uint8_t>(
          std::min(linear_x * max_speed_reciprocal_reverse_, 100.0));
      speed_frame_.data[2] = 0;
      speed_frame_.data[3] = 0;
      speed_frame_.data[4] = 0;
      speed_frame_.data[5] = 0;
      speed_frame_.data[6] = 0;
      speed_frame_.data[7] = 0;
    }

    static int16_t angle_to_send = 0;
    angle_to_send = static_cast<uint16_t>(angular_z);
    static uint8_t angle_low_byte, angle_high_byte;
    angle_low_byte = angle_to_send & (0xFF);
    angle_high_byte = (angle_to_send >> 8) & (0xFF);

    sas_frame_.data = {0x01, 0x01, angle_low_byte, angle_high_byte, 0, 0, 0, 0};

    {
      std::unique_lock<std::mutex> lock(can_buffer_mutex_);
      can_to_send_buffer_.push_back(speed_frame_);
    }
  } else {
    SetForceBrake();
  }
}

void GolfCanParser::BrakeHandler(std_msgs::BoolConstPtr msg) {
  brake_state_ = msg->data;
  if (brake_state_) {
    SetForceBrake();
  }
}

void GolfCanParser::SetForceBrake() {
  ROS_WARN_STREAM("Force brake on");
  speed_frame_.data[0] = (1 << 3);
  speed_frame_.data[1] = 0;
  {
    std::unique_lock<std::mutex> lock(can_buffer_mutex_);
    can_to_send_buffer_.push_back(speed_frame_);
  }
}

void GolfCanParser::HeartBeatCb(const ros::TimerEvent &) {
  if (!initialize_finished_) {
    return;
  }
  // ROS_WARN_STREAM("Heartbeat test!");
  {
    std::unique_lock<std::mutex> lock(can_buffer_mutex_);
    can_to_send_buffer_.push_back(speed_frame_);
    can_to_send_buffer_.push_back(sas_frame_);
    // Dealing with IO can
  }
}

void GolfCanParser::TransmitCb(const ros::TimerEvent &) {
  if (!initialize_finished_) {
    return;
  }
  if (!can_to_send_buffer_.empty()) {
    std::unique_lock<std::mutex> lock(can_buffer_mutex_);
    // for (auto& i : can_to_send_buffer_)
    // {
    // 	                //if (i.data[7] != 0)
    // 	                //{
    // 	                //    continue;
    // 	                //}
    // 	 std::cout.setf ( std::ios::hex,std::ios::basefield);;
    // 	 std::cout << "ID: " << static_cast<int>(i.id) << ", ";
    // 	 for (int ii = 0; ii < i.data.size(); ii++)
    // 	 {
    // 	     std::cout << static_cast<int>(i.data[ii]) << ", ";
    // 	 }
    // 	 std::cout.unsetf ( std::ios::hex);
    // 	 std::cout << std::endl;
    // }
    if (static_cast<int>(can_to_send_buffer_.size()) > 0) {
      can_bridge_ptr_->Write(can_to_send_buffer_);
      can_to_send_buffer_.clear();
    }
  } else {
    // Nothing to transmit
  }
}

void GolfCanParser::ReceiveCb(const ros::TimerEvent &) {
  if (!initialize_finished_) {
    return;
  }
  //	ROS_WARN_STREAM("read buffer is: " << to_read_buffer_.size());
  if (can_bridge_ptr_->Read(to_read_buffer_) > 0) {
    for (auto &it : to_read_buffer_) {
      //	ROS_INFO_STREAM(static_cast<int>(it.id));
      if (it.id == 0x2F2) {
        // Battery publish
        static int battery_cnt = 98;
        if (battery_cnt++ >= 50) {
          static std_msgs::Float32 battery_msg;
          battery_msg.data = static_cast<double>(it.data[5]) / 255.0;
          battery_pub_.publish(battery_msg);
          battery_cnt = 0;
        }
        // Error state publish
        static std_msgs::UInt8 error_state_msg;
        error_state_msg.data = it.data[6];
        error_state_pub_.publish(error_state_msg);

        static geometry_msgs::Vector3Stamped msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "steering_wheel";
        msg.vector.z = it.data[3] | (it.data[4] << 8);
        angle_pub_.publish(msg);
      } else if (0x18F01D48 == it.id) {
        // ROS_INFO_STREAM("Have received msg from id" << it.id);
        uint16_t uint_angle = it.data[0] | it.data[1] << 8;
        static std_msgs::Float32 angle_msg;
        if (uint_angle <= 32767) {
          angle_msg.data = 0.1 * uint_angle;
        } else {
          angle_msg.data = 0.1 * static_cast<int16_t>(uint_angle - 65535);
        }
        sas_pub_.publish(angle_msg);
      }

      else if (0x780 == it.id) {
        // ROS_INFO_STREAM("Have receive encoder");
        encoder_calculator_ptr_->ProcessRawData(it);
      }
    }

    // ROS_WARN_STREAM("The size of can read buffer is: " <<
    // to_read_buffer_.size());
    // TODO: parser
    // for (auto& it: to_read_buffer_)
    // {
    // 	//ROS_INFO_STREAM("Have Receive from id" << it.id);
    // 	//for (auto &i: it.data)
    // 	//{
    // 	//	std::cout.setf ( std::ios::hex,std::ios::basefield);;
    // 	//	std::cout << static_cast<int>(i) << ", ";
    // 	//}
    // 	//std::cout.unsetf ( std::ios::hex);
    // 	//std::cout << std::endl;
    // }
  }
}

// void GolfCanParser::ComputeCrcCheck()
//{
//		uint16_t check_reg = 0xFFFF;
//		uint16_t current_value;

//		for (int i = 0; i < 6; i++)
//		{
//				current_value = io_frame_.data[i] << 8;

//				for (int j = 0; j < 8; j++)
//				{
//						if ((short)(check_reg ^ current_value)
//< 0)
//						{
//								check_reg = (check_reg << 1)
//^ 0x8005;
//						}
//						else
//						{
//								check_reg <<= 1;
//						}
//						current_value <<= 1;
//				}

//		}

// io_frame_.data[6] = check_reg & 0xFF;
// io_frame_.data[7] = (check_reg >> 8) & 0xFF;

//}
}  // namespace golfmiddleware
