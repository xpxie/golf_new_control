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
#include "ramlab_msgs.h"
#include "encoder_velocity_to_odom.h"

#ifndef RAMLAB_RELEASE
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt8.h>
#endif

namespace golfmiddleware {
class CanParser : public nodelet::Nodelet {
 public:
  CanParser(){};

 private:
  virtual void onInit();
  ros::NodeHandle nh_;
  ros::Subscriber honk_sub_, highlight_sub_;
  ros::Subscriber left_signal_light_sub_, controller_sub_;
  ros::Subscriber right_signal_light_sub_, brake_light_sub_;
  ros::Subscriber cmd_vel_sub_, brake_sub_;

  ros::Publisher battery_pub_, error_state_pub_;
  ros::Publisher angle_pub_;
  ros::Publisher sas_pub_;
  ros::Publisher swa_steer_pub_;

  ros::Publisher sonar_pub_;

  std::shared_ptr<golfcar::EncoderVelocityToOdom> encoder_calculator_ptr_;
  // golfcar::EncoderVelocityToOdom encoder_calculator_;

  typedef enum {
    HONK = 0,
    HIGHLIGHT,
    LEFT_SIGNAL_LIGHT,
    RIGHT_SIGNAL_LIGHT,
    BRAKE_LIGHT,
    CONTROLLER,
    REAR_LEFT_SIGNAL_LIGHT,
    REAR_RIGHT_SIGNAL_LIGHT,
    WIDTH_LIGHT
  } IoType;

  std::string io_names_[9] = {"HONK",
                              "HIGHLIGHT",
                              "LEFT_SIGNAL_LIGHT",
                              "RIGHT_SIGNAL_LIGHT",
                              "BRAKE_LIGHT",
                              "CONTROLLER",
                              "REAR_LEFT_SIGNAL_LIGHT",
                              "REAR_RIGHT_SIGNAL_LIGHT",
                              "WIDTH_LIGHT"};

  int io_byte_dot_bit_[9][2] = {
      {0, 1},  // HONK
      {1, 7},  // HIGHLIGHT
      {1, 5},  // LEFT_SIGNAL_LIGHT
      {1, 6},  // RIGHT_SIGNAL_LIGHT
      {1, 0},  // BRAKE_LIGHT
      {0, 1},  // CONTROLLER
      {0, 5},  // REAR_LEFT_SIGNAL_LIGHT,
      {0, 6},  // REAR_RIGHT_SIGNAL_LIGHT
      {0, 7},  // WIDTH_LIGHT
  };

  const int io_id_ = 0x610;
  const int speed_id_ = 0x1F1;  // origin 0x1F1, no change

  int car_max_motor_speed_ = 5500;
  int car_max_motor_speed_reverse_ = 5500;

  double heartbeat_cycle_ = 0.04;
  double trans_receive_cycle_ = 0.005;
  // adjust finished

  void GeneralHandler(std_msgs::BoolConstPtr msg, const IoType& io_type);
  void CmdVelHandler(geometry_msgs::TwistConstPtr msg);
  void BrakeHandler(std_msgs::BoolConstPtr msg);

  void SetBit(const IoType& io_type);
  void ResetBit(const IoType& io_type);

  can_msgs::Frame io_frame_;
  can_msgs::Frame speed_frame_;

  double max_speed_reciprocal_ = 0.0;
  double max_speed_reciprocal_reverse_ = 0.0;
  bool brake_state_ = false;

  void SetForceBrake();
  std::vector<can_msgs::Frame> can_to_send_buffer_;
  std::mutex can_buffer_mutex_;
  ros::Timer heartbeat_timer_, transmit_timer_, receive_timer_;
  void HeartBeatCb(const ros::TimerEvent&);
  void TransmitCb(const ros::TimerEvent&);
  void ReceiveCb(const ros::TimerEvent&);

  void TransmitSonar(void);

  std::shared_ptr<cantools::CanBridge> can_bridge_ptr_;
  vector<can_msgs::Frame> to_read_buffer_;

  bool initialize_finished_ = false;
  void ComputeCrcCheck(void);
  double last_linear_x_;
  int acc_flag_ = 0;

  // static const int front_sonar_num_ = 4;
  // static const int rear_sonar_num_ = 4;
  // static const int left_sonar_num_ = 2;
  // static const int right_sonar_num_ = 2;
  static const int front_sonar_num_ = 2;
  static const int rear_sonar_num_ = 2;
  static const int left_sonar_num_ = 4;
  static const int right_sonar_num_ = 4;

  std::array<std::list<double>, front_sonar_num_> front_distance_array_;
  std::array<std::list<double>, rear_sonar_num_> rear_distance_array_;
  std::array<std::list<double>, left_sonar_num_> left_distance_array_;
  std::array<std::list<double>, right_sonar_num_> right_distance_array_;
  const int sonar_cache_length_ = 10;
};
}  // namespace golfmiddleware

PLUGINLIB_EXPORT_CLASS(golfmiddleware::CanParser, nodelet::Nodelet);

namespace golfmiddleware {
void CanParser::onInit() {
  nh_ = getPrivateNodeHandle();
  honk_sub_ = nh_.subscribe<std_msgs::Bool>(
      "/decision/honk", 1,
      boost::bind(&CanParser::GeneralHandler, this, _1, HONK));
  highlight_sub_ = nh_.subscribe<std_msgs::Bool>(
      "/decision/highlight", 1,
      boost::bind(&CanParser::GeneralHandler, this, _1, HIGHLIGHT));
  left_signal_light_sub_ = nh_.subscribe<std_msgs::Bool>(
      "/decision/left_signal_light", 1,
      boost::bind(&CanParser::GeneralHandler, this, _1, LEFT_SIGNAL_LIGHT));
  right_signal_light_sub_ = nh_.subscribe<std_msgs::Bool>(
      "/decision/right_signal_light", 1,
      boost::bind(&CanParser::GeneralHandler, this, _1, RIGHT_SIGNAL_LIGHT));
  brake_light_sub_ = nh_.subscribe<std_msgs::Bool>(
      "/decision/brake_light", 1,
      boost::bind(&CanParser::GeneralHandler, this, _1, BRAKE_LIGHT));
  controller_sub_ = nh_.subscribe<std_msgs::Bool>(
      "/decision/controller", 1,
      boost::bind(&CanParser::GeneralHandler, this, _1, CONTROLLER));
  cmd_vel_sub_ =
      nh_.subscribe("/decision/cmd_vel", 1, &CanParser::CmdVelHandler, this);
  brake_sub_ =
      nh_.subscribe("/decision/brake", 1, &CanParser::BrakeHandler, this);
  battery_pub_ = nh_.advertise<std_msgs::Float32>("/battery", 1);
  error_state_pub_ = nh_.advertise<std_msgs::UInt8>("/error_state", 1);
  angle_pub_ =
      nh_.advertise<geometry_msgs::Vector3Stamped>("/steering_wheel_angle", 1);
  sas_pub_ = nh_.advertise<std_msgs::Float32>("/steering_wheel_angle_sas", 1);
  swa_steer_pub_ =
      nh_.advertise<std_msgs::Float64MultiArray>("/steer_feedback_swa", 1);
  sonar_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/pub_float", 1);
  nh_.param<int>("car_max_motor_speed", car_max_motor_speed_, 5500);
  nh_.param<int>("car_max_motor_speed_reverse", car_max_motor_speed_reverse_,
                 5500);

  io_frame_.id = io_id_;
  io_frame_.header.frame_id = "io_control";
  for (auto& i : io_frame_.data) {
    i = 0;
  }
  ComputeCrcCheck();

  speed_frame_.id = speed_id_;
  speed_frame_.header.frame_id = "speed_control";
  speed_frame_.data = {0x38, 0x00, 0x00, 0x01, 0x01, 0x84, 0x03, 0x00};

  // io_frame init end

  // max_speed_reciprocal_ =
  //     1.0 / (static_cast<double>(car_max_motor_speed_) / 3000.0 * 4.75308642)
  //     * 100.0;
  // max_speed_reciprocal_reverse_ =
  //     -1.0 /
  //     (static_cast<double>(car_max_motor_speed_reverse_) / 3000.0 *
  //      4.75308642) *
  //     100.0;
  max_speed_reciprocal_ = 3000.0 / 19.7165065 * 3.6;           // 547 ;
  max_speed_reciprocal_reverse_ = -3000.0 / 19.7165065 * 3.6;  // 4.75308642 ;

  {
    std::unique_lock<std::mutex> lock(can_buffer_mutex_);
    can_to_send_buffer_.clear();
  }

  heartbeat_timer_ = nh_.createTimer(ros::Duration(heartbeat_cycle_),
                                     &CanParser::HeartBeatCb, this);
  receive_timer_ = nh_.createTimer(ros::Duration(trans_receive_cycle_),
                                   &CanParser::ReceiveCb, this);
  transmit_timer_ = nh_.createTimer(ros::Duration(trans_receive_cycle_),
                                    &CanParser::TransmitCb, this);

  can_bridge_ptr_ =
      std::shared_ptr<cantools::CanBridge>(new cantools::CanBridge());
  can_bridge_ptr_->CanOpen();
  // ROS_WARN_STREAM("ptr_crash");
  encoder_calculator_ptr_ = shared_ptr<golfcar::EncoderVelocityToOdom>(
      new golfcar::EncoderVelocityToOdom(nh_));
  // encoder_calculator_.OnInit(nh_);
  // ROS_WARN_STREAM("HERE");
  initialize_finished_ = true;
}

void CanParser::GeneralHandler(std_msgs::BoolConstPtr msg,
                               const IoType& io_type) {
  static int left_count = 0;
  static int right_count = 0;
  if (true == msg->data) {
    if (io_type != LEFT_SIGNAL_LIGHT && io_type != RIGHT_SIGNAL_LIGHT) {
      SetBit(io_type);
    }
    if (io_type == LEFT_SIGNAL_LIGHT) {
      if (++left_count == 1) {
        SetBit(REAR_LEFT_SIGNAL_LIGHT);
        SetBit(io_type);
      } else if (left_count == 4) {
        ResetBit(REAR_LEFT_SIGNAL_LIGHT);
        ResetBit(io_type);
      } else if (left_count == 5) {
        left_count = 0;
      }
    }
    if (io_type == RIGHT_SIGNAL_LIGHT) {
      if (++right_count == 1) {
        SetBit(io_type);
        SetBit(REAR_RIGHT_SIGNAL_LIGHT);
      } else if (right_count == 4) {
        ResetBit(io_type);
        ResetBit(REAR_RIGHT_SIGNAL_LIGHT);
      } else if (right_count == 5) {
        right_count = 0;
      }
    }
    if (io_type == HIGHLIGHT) {
      SetBit(WIDTH_LIGHT);
    }
  } else if (false == msg->data) {
    if (io_type != LEFT_SIGNAL_LIGHT && io_type != RIGHT_SIGNAL_LIGHT) {
      ResetBit(io_type);
    }
    if (io_type == RIGHT_SIGNAL_LIGHT) {
      right_count = 0;
      ResetBit(REAR_RIGHT_SIGNAL_LIGHT);
      ResetBit(io_type);
    }
    if (io_type == LEFT_SIGNAL_LIGHT) {
      left_count = 0;
      ResetBit(REAR_LEFT_SIGNAL_LIGHT);
      ResetBit(io_type);
    }
    if (io_type == HIGHLIGHT) {
      ResetBit(WIDTH_LIGHT);
    }
  }
}

void CanParser::SetBit(const IoType& io_type) {
  // ROS_INFO_STREAM(io_names_[io_type] << " ON");
  io_frame_.data[io_byte_dot_bit_[io_type][0]] |=
      (1 << io_byte_dot_bit_[io_type][1]);
  ComputeCrcCheck();
  //		v{
  //			std::unique_lock<std::mutex> lock(can_buffer_mutex_);
  //			can_to_send_buffer_.push_back(io_frame_);
  //		}
}

void CanParser::ResetBit(const IoType& io_type) {
  // ROS_INFO_STREAM(io_names_[io_type] << "OFF");
  io_frame_.data[io_byte_dot_bit_[io_type][0]] &=
      ~(1 << io_byte_dot_bit_[io_type][1]);
  ComputeCrcCheck();
  //		{
  //			std::unique_lock<std::mutex> lock(can_buffer_mutex_);
  //			can_to_send_buffer_.push_back(io_frame_);
  //		}
}

void CanParser::CmdVelHandler(geometry_msgs::TwistConstPtr msg) {
  static float linear_x, angular_z;
  if (!brake_state_) {
    linear_x = msg->linear.x;
    angular_z = msg->angular.z;

    if (linear_x >= 0) {
      if ((linear_x - last_linear_x_ > 0) && acc_flag_ != 1) {
        acc_flag_ = 1;
        ResetBit(BRAKE_LIGHT);
      } else if ((linear_x - last_linear_x_ < 0) && acc_flag_ != -1) {
        acc_flag_ = -1;
        SetBit(BRAKE_LIGHT);
      } else if ((linear_x - last_linear_x_ == 0) && linear_x == 0 &&
                 acc_flag_ != -1) {
        acc_flag_ = -1;
        SetBit(BRAKE_LIGHT);
      } else if ((linear_x - last_linear_x_ == 0) && linear_x > 0 &&
                 acc_flag_ != 1) {
        acc_flag_ = 1;
        ResetBit(BRAKE_LIGHT);
      }
    } else {
      if (-1 != acc_flag_) {
        SetBit(BRAKE_LIGHT);
      }
    }

    last_linear_x_ = linear_x;

    static double min_tolerance = 1.0 / max_speed_reciprocal_;
    static double min_walk = 0.12;
    // ROS_INFO_STREAM("min_tole:" << min_tolerance);

    if (linear_x > min_tolerance) {
      linear_x = (linear_x > min_walk) ? linear_x : min_walk;

      static uint16_t real_time_rpm;
      real_time_rpm = static_cast<uint16_t>(
          std::min((linear_x * max_speed_reciprocal_), 5000.0));

      static uint16_t real_time_steer;
      real_time_steer = static_cast<uint16_t>(
          std::min(-angular_z / 41.0625 * 900 + 900, 1800.0));

      speed_frame_.data[0] = 0x15;  // byte0: 0001 0110 = 0x16 forward
      speed_frame_.data[1] = static_cast<uint8_t>(real_time_rpm & 0xFF);
      speed_frame_.data[2] = static_cast<uint8_t>(real_time_rpm >> 8);
      speed_frame_.data[3] = 0x01;
      speed_frame_.data[4] = 0x01;
      speed_frame_.data[5] = static_cast<uint8_t>(real_time_steer & 0xFF);
      speed_frame_.data[6] = static_cast<uint8_t>(real_time_steer >> 8);
      speed_frame_.data[7] = 0x00;
      ROS_INFO_STREAM(real_time_rpm);
    } else if (fabs(linear_x) <= min_tolerance) {
      static uint16_t real_time_steer;
      real_time_steer = static_cast<uint16_t>(
          std::min(-angular_z / 41.0625 * 900 + 900, 1800.0));

      speed_frame_.data[0] = 0x38;  // byte0: 0011 1000 = 0x38
      speed_frame_.data[1] = 0x00;
      speed_frame_.data[2] = 0x00;
      speed_frame_.data[3] = 0x01;
      speed_frame_.data[4] = 0x01;
      speed_frame_.data[5] = static_cast<uint8_t>(real_time_steer & 0xFF);
      speed_frame_.data[6] = static_cast<uint8_t>(real_time_steer >> 8);
      speed_frame_.data[7] = 0;
    } else {
      linear_x = (linear_x < -min_walk) ? linear_x : (-min_walk);

      static uint16_t real_time_rpm;
      real_time_rpm = static_cast<uint16_t>(
          std::min((linear_x * max_speed_reciprocal_reverse_), 5000.0));

      static uint16_t real_time_steer;
      real_time_steer = static_cast<uint16_t>(
          std::min(-angular_z / 41.0625 * 900 + 900, 1800.0));

      speed_frame_.data[0] = 0x16;  // byte0: 0001 0101 = 0x15 backward
      speed_frame_.data[1] = static_cast<uint8_t>(real_time_rpm & 0xFF);
      speed_frame_.data[2] = static_cast<uint8_t>(real_time_rpm >> 8);
      speed_frame_.data[3] = 0x01;
      speed_frame_.data[4] = 0x01;
      speed_frame_.data[5] = static_cast<uint8_t>(real_time_steer & 0xFF);
      speed_frame_.data[6] = static_cast<uint8_t>(real_time_steer >> 8);
      speed_frame_.data[7] = 0;
    }

    {
      std::unique_lock<std::mutex> lock(can_buffer_mutex_);
      can_to_send_buffer_.push_back(speed_frame_);
    }
  } else {
    SetForceBrake();
  }
}

void CanParser::BrakeHandler(std_msgs::BoolConstPtr msg) {
  brake_state_ = msg->data;
  if (brake_state_) {
    SetForceBrake();
  }
}

void CanParser::SetForceBrake() {
  ROS_WARN_STREAM("Force brake on");
  speed_frame_.data[0] = 1 << 5;
  speed_frame_.data[1] = 0;
  {
    std::unique_lock<std::mutex> lock(can_buffer_mutex_);
    can_to_send_buffer_.push_back(speed_frame_);
  }
}

void CanParser::HeartBeatCb(const ros::TimerEvent&) {
  if (!initialize_finished_) {
    return;
  }
  // ROS_WARN_STREAM("Heartbeat test!");
  {
    std::unique_lock<std::mutex> lock(can_buffer_mutex_);
    can_to_send_buffer_.push_back(speed_frame_);
    // Dealing with IO can
    // can_to_send_buffer_.push_back(io_frame_);
  }
}

void CanParser::TransmitCb(const ros::TimerEvent&) {
  if (!initialize_finished_) {
    return;
  }
  if (!can_to_send_buffer_.empty()) {
    std::unique_lock<std::mutex> lock(can_buffer_mutex_);
    // for (auto& i : can_to_send_buffer_)
    // {
    // 	//                if (i.data[7] != 0)
    // 	//                {
    // 	//                    continue;
    // 	//                }
    // 	for (int ii = 0; ii < i.data.size(); ii++)
    // 	{
    // 		std::cout.setf ( std::ios::hex,std::ios::basefield);;
    // 	    std::cout << static_cast<int>(i.data[ii]) << ", ";
    // 	}
    // 	std::cout.unsetf ( std::ios::hex);
    // 	std::cout << std::endl;
    // }
    if (static_cast<int>(can_to_send_buffer_.size()) > 0) {
      can_bridge_ptr_->Write(can_to_send_buffer_);
      can_to_send_buffer_.clear();
    }
  } else {
    // Nothing to transmit
  }
}

void CanParser::TransmitSonar() {
  std_msgs::Float64MultiArray sonar_msg;
  sonar_msg.data[0] = *std::min_element(front_distance_array_.at(0).begin(),
                                        front_distance_array_.at(0).end());
  sonar_msg.data[1] = *std::min_element(front_distance_array_.at(1).begin(),
                                        front_distance_array_.at(1).end());
  sonar_msg.data[2] = *std::min_element(right_distance_array_.at(0).begin(),
                                        right_distance_array_.at(0).end());
  sonar_msg.data[3] = *std::min_element(right_distance_array_.at(1).begin(),
                                        right_distance_array_.at(1).end());
  sonar_msg.data[4] = *std::min_element(right_distance_array_.at(2).begin(),
                                        right_distance_array_.at(2).end());
  sonar_msg.data[5] = *std::min_element(right_distance_array_.at(3).begin(),
                                        right_distance_array_.at(3).end());
  sonar_msg.data[6] = *std::min_element(rear_distance_array_.at(0).begin(),
                                        rear_distance_array_.at(0).end());
  sonar_msg.data[7] = *std::min_element(rear_distance_array_.at(1).begin(),
                                        rear_distance_array_.at(1).end());
  sonar_msg.data[8] = *std::min_element(left_distance_array_.at(0).begin(),
                                        left_distance_array_.at(0).end());
  sonar_msg.data[9] = *std::min_element(left_distance_array_.at(1).begin(),
                                        left_distance_array_.at(1).end());
  sonar_msg.data[10] = *std::min_element(left_distance_array_.at(2).begin(),
                                         left_distance_array_.at(2).end());
  sonar_msg.data[11] = *std::min_element(left_distance_array_.at(3).begin(),
                                         left_distance_array_.at(3).end());
  sonar_pub_.publish(sonar_msg);
}

void CanParser::ReceiveCb(const ros::TimerEvent&) {
  if (!initialize_finished_) {
    return;
  }
  if (can_bridge_ptr_->Read(to_read_buffer_) > 0) {
    for (auto& it : to_read_buffer_) {
      // ROS_INFO_STREAM("receive data from id: " << it.id);
      if (it.id == 0x18F01D48) {
        static double feedback_angle;
        static std_msgs::Float64MultiArray steer_feedback_msgs;
        static uint16_t feedback_angle_16;

        feedback_angle_16 = it.data[0] | (it.data[1] << 8);
        if (feedback_angle_16 <= 32767) {
          feedback_angle = static_cast<double>(0.1 * feedback_angle_16);
        } else {
          feedback_angle =
              static_cast<double>(0.1 * (feedback_angle_16 - 65535));
        }

        static uint8_t feedback_angle_rate_8;
        feedback_angle_rate_8 = it.data[2];
        static double feedback_angle_rate =
            static_cast<double>(feedback_angle_rate_8);
        static double rotation_feedback_scale_ = 30.0 / 516.164384 ;
        feedback_angle = -(feedback_angle + 216.0) * rotation_feedback_scale_;

        steer_feedback_msgs.data.clear();
        steer_feedback_msgs.data.emplace_back(feedback_angle - 4.2);
        steer_feedback_msgs.data.emplace_back(feedback_angle_rate);
        swa_steer_pub_.publish(steer_feedback_msgs);
      }

      if (it.id == 0x2F2) {
        // Battery publish
        static int battery_cnt = 98;
        if (battery_cnt++ >= 50) {
          static std_msgs::Float32 battery_msg;
          battery_msg.data = static_cast<double>(it.data[6]) / 255.0 * 100.0;
          battery_pub_.publish(battery_msg);
          battery_cnt = 0;
        }
        // Error state publish
        static std_msgs::UInt8 error_state_msg;
        // error_state_msg.data = it.data[6];
        error_state_msg.data = 0x00;                // set arbitrarily
        error_state_pub_.publish(error_state_msg);  // no error msg now

        static geometry_msgs::Vector3Stamped msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "steering_wheel";
        msg.vector.z = it.data[4] | (it.data[5] << 8);
        angle_pub_.publish(msg);
      }

      if (0x180 == it.id) {
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

      if (it.id == 0x306) {
        // ROS_WARN_STREAM("crash");
        // encoder_calculator_.ProcessRawData(it);
        encoder_calculator_ptr_->ProcessRawData(it);
      }

      // id: 0x301 front ->0x303(now)
      // if (0x303 == it.id) {
      //  for (int i = 0; i < front_sonar_num_; ++i) {
      //    front_distance_array_.at(i).push_back(
      //        static_cast<double>(it.data[i + 1]) * 2);
      //  }
      //  if (it.data[0] == 0x80) {
      //    front_distance_array_.at(0).pop_back();
      //    front_distance_array_.at(0).push_back(-1);
      //    ROS_WARN_STREAM("front 1 data not received");
      //  } else if (it.data[0] == 0x82) {
      //    front_distance_array_.at(1).pop_back();
      //    front_distance_array_.at(1).push_back(-1);
      //    ROS_WARN_STREAM("front 2 data not received");
      //  }
      //  // } else if (it.data[0] == 0x84) {
      //  //   front_distance_array_.at(2).pop_back();
      //  //   front_distance_array_.at(2).push_back(-1);
      //  //   ROS_WARN_STREAM("front 3 data not received");
      //  // } else if (it.data[0] == 0x88) {
      //  //   front_distance_array_.at(3).pop_back();
      //  //   front_distance_array_.at(3).push_back(-1);
      //  //   ROS_WARN_STREAM("front 4 data not received");
      //  } else if (it.data[0] != 0x40) {
      //    ROS_WARN_STREAM("front Unkonwn error: " << it.data[0]);
      //  }
      //  if (front_distance_array_.at(0).size() >= sonar_cache_length_) {
      //    for (int i = 0; i < front_sonar_num_; ++i) {
      //      front_distance_array_.at(i).pop_front();
      //    }
      //  }

      //// id: 0x302 rear ->0x304(now)
      // if (0x304 == it.id) {
      //  for (int i = 0; i < rear_sonar_num_; ++i) {
      //    rear_distance_array_.at(i).push_back(
      //        static_cast<double>(it.data[i + 1]) * 2);
      //  }
      //  if (it.data[0] == 0x80) {
      //    rear_distance_array_.at(0).pop_back();
      //    rear_distance_array_.at(0).push_back(-1);
      //    ROS_WARN_STREAM("rear 1 data not received");
      //  } else if (it.data[0] == 0x82) {
      //    rear_distance_array_.at(1).pop_back();
      //    rear_distance_array_.at(1).push_back(-1);
      //    ROS_WARN_STREAM("rear 2 data not received");
      //  }
      //  // } else if (it.data[0] == 0x84) {
      //  //   rear_distance_array_.at(2).pop_back();
      //  //   rear_distance_array_.at(2).push_back(-1);
      //  //   ROS_WARN_STREAM("rear 3 data not received");
      //  // } else if (it.data[0] == 0x88) {
      //  //   rear_distance_array_.at(3).pop_back();
      //  //   rear_distance_array_.at(3).push_back(-1);
      //  //   ROS_WARN_STREAM("rear 4 data not received");
      //  // } else if (it.data[0] != 0x40) {
      //  //   ROS_WARN_STREAM("rear Unkonwn error: " << it.data[0]);
      //  // }
      //  if (rear_distance_array_.at(0).size() >= sonar_cache_length_) {
      //    for (int i = 0; i < rear_sonar_num_; ++i) {
      //      rear_distance_array_.at(i).pop_front();
      //    }
      //  }
      //}

      //// id: 0x303 right ->0x301(now)
      // if (0x301 == it.id) {
      //  for (int i = 0; i < right_sonar_num_; ++i) {
      //    right_distance_array_.at(i).push_back(
      //        static_cast<double>(it.data[i + 1]) * 2);
      //  }
      //  if (it.data[0] == 0x80) {
      //    right_distance_array_.at(0).pop_back();
      //    right_distance_array_.at(0).push_back(-1);
      //    ROS_WARN_STREAM("right 1 data not received");
      //  } else if (it.data[0] == 0x82) {
      //    right_distance_array_.at(1).pop_back();
      //    right_distance_array_.at(1).push_back(-1);
      //    ROS_WARN_STREAM("right 2 data not received");
      //  } else if (it.data[0] == 0x84) {
      //    right_distance_array_.at(2).pop_back();
      //    right_distance_array_.at(2).push_back(-1);
      //    ROS_WARN_STREAM("right 3 data not received");
      //  } else if (it.data[0] == 0x88) {
      //    right_distance_array_.at(3).pop_back();
      //    right_distance_array_.at(3).push_back(-1);
      //    ROS_WARN_STREAM("right 4 data not received");
      //  } else if (it.data[0] != 0x40) {
      //    ROS_WARN_STREAM("right Unkonwn error: " << it.data[0]);
      //  }
      //  if (right_distance_array_.at(0).size() >= sonar_cache_length_) {
      //    for (int i = 0; i < right_sonar_num_; ++i) {
      //      right_distance_array_.at(i).pop_front();
      //    }
      //  }
      //}

      //// id: 0x304 left ->0x302(now)
      // if (0x302 == it.id) {
      //  for (int i = 0; i < left_sonar_num_; ++i) {
      //    left_distance_array_.at(i).push_back(
      //        static_cast<double>(it.data[i + 1]) * 2);
      //  }
      //  if (it.data[0] == 0x80) {
      //    left_distance_array_.at(0).pop_back();
      //    left_distance_array_.at(0).push_back(-1);
      //    ROS_WARN_STREAM("left 1 data not received");
      //  } else if (it.data[0] == 0x82) {
      //    left_distance_array_.at(1).pop_back();
      //    left_distance_array_.at(1).push_back(-1);
      //    ROS_WARN_STREAM("left 2 data not received");
      //  } else if (it.data[0] == 0x84) {
      //    left_distance_array_.at(2).pop_back();
      //    left_distance_array_.at(2).push_back(-1);
      //    ROS_WARN_STREAM("left 3 data not received");
      //  } else if (it.data[0] == 0x88) {
      //    left_distance_array_.at(3).pop_back();
      //    left_distance_array_.at(3).push_back(-1);
      //    ROS_WARN_STREAM("left 4 data not received");
      //  } else if (it.data[0] != 0x40) {
      //    ROS_WARN_STREAM("left Unkonwn error: " << it.data[0]);
      //  }
      //  if (left_distance_array_.at(0).size() >= sonar_cache_length_) {
      //    for (int i = 0; i < left_sonar_num_; ++i) {
      //      left_distance_array_.at(i).pop_front();
      //    }
      //  }

      //  TransmitSonar();
      //}
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

void CanParser::ComputeCrcCheck() {
  uint16_t check_reg = 0xFFFF;
  uint16_t current_value;

  for (int i = 0; i < 6; i++) {
    current_value = io_frame_.data[i] << 8;

    for (int j = 0; j < 8; j++) {
      if ((short)(check_reg ^ current_value) < 0) {
        check_reg = (check_reg << 1) ^ 0x8005;
      } else {
        check_reg <<= 1;
      }
      current_value <<= 1;
    }
  }

  io_frame_.data[6] = check_reg & 0xFF;
  io_frame_.data[7] = (check_reg >> 8) & 0xFF;
}
}  // namespace golfmiddleware
