/**
******************************************************************************
* Copyright (C) 2018 Joey.Liu <lty2226262@gmail.com>
* Distributed under terms of the MIT license.
******************************************************************************
*/

//#define DEBUG
//#define DEBUG_BRIDGE

#include "can_bridge.h"
#include <ros/ros.h>

using boost::format;
using std::mutex;
using std::string;
using std::to_string;
using std::vector;

unsigned int cantools::CanBridge::device_index_ = 0;
unsigned int cantools::CanBridge::can_index_ = 0;
// unsigned int cantools::CanBridge::light_index_ = 1;
int cantools::CanBridge::run_flag_ = 0;
vector<can_msgs::Frame> cantools::CanBridge::can_buf_ =
    vector<can_msgs::Frame>();
can_msgs::Frame cantools::CanBridge::can_tmp_ = can_msgs::Frame();
mutex cantools::CanBridge::can_mutex_sync_;

namespace cantools {
CanBridge::CanBridge() {}
CanBridge::~CanBridge() { CanClose(); }

int CanBridge::Read(vector<can_msgs::Frame>& frames) {
  frames.clear();
  {
    unique_lock<mutex> lock(can_mutex_sync_);
    for (const auto& it : can_buf_) {
      frames.push_back(it);
    }
    can_buf_.clear();
  }
  return frames.size();
}

void* CanBridge::ReceiveFunc(void* param) {
  int receive_length;
  VCI_CAN_OBJ receive_buffer[100];

  int* run = static_cast<int*>(param);
  int index = ((*run) >> 4);
  while ((*run) & 0x0f) {
    if ((receive_length = VCI_Receive(VCI_USBCAN2, device_index_, index,
                                      receive_buffer, 100, 100)) > 0) {
      ros::Time tmp_t = ros::Time::now();
      for (int i = 0; i < receive_length; i++) {
        // std::cout.setf ( std::ios::hex,std::ios::basefield);;
        // std::cout << "receive id:" << receive_buffer[i].ID << std::endl;
        // std::cout.unsetf ( std::ios::hex);
        // if (receive_buffer[i].ID != 0x2F2 && receive_buffer[i].ID !=
        // 0x18F01D48)
        // {
        // 	continue;
        // }
        can_tmp_.header.stamp = tmp_t;
        can_tmp_.header.frame_id = "/can";
        can_tmp_.id = receive_buffer[i].ID;
        can_tmp_.is_extended = receive_buffer[i].ExternFlag;
        can_tmp_.is_rtr = receive_buffer[i].RemoteFlag;
        for (int j = 0; j < receive_buffer[i].DataLen; j++) {
          can_tmp_.data[j] = receive_buffer[i].Data[j];
        }
        {
          unique_lock<mutex> lock(can_mutex_sync_);
          can_buf_.push_back(can_tmp_);
        }
      }
    }
  }
  pthread_exit(0);
}

void CanBridge::CanOpen() {
  if (run_flag_ != 0) {
    ROS_ERROR_STREAM(">> can tool is open");
    return;
  }

  ROS_ERROR_STREAM("can tools self test");

  if (VCI_OpenDevice(VCI_USBCAN2, device_index_, 0) == 1) {
    ROS_ERROR_STREAM(">> open device success");
  } else {
    ROS_ERROR_STREAM(">> open device fail");
    exit(1);
  }

  if (VCI_ReadBoardInfo(VCI_USBCAN2, device_index_, &board_info_) == 1) {
    ROS_INFO_STREAM(">> Get VCI_ReadBoardInfo success!");

    std::string output_string("");
    for (int i = 0; i < 20; i++) {
      output_string += board_info_.str_Serial_Num[i];
    }
    ROS_INFO_STREAM(">> Serial_Num: " << output_string);
    output_string.clear();

    for (int i = 0; i < 10; i++) {
      output_string += board_info_.str_hw_Type[i];
    }
    ROS_INFO_STREAM(">> hw_Type:" << output_string);
  } else {
    ROS_ERROR_STREAM(">> Get VCI_ReadBoardInfo error!");
    exit(1);
  }

  VCI_INIT_CONFIG config;
  config.AccCode = 0;
  config.AccMask = 0xffffffff;
  config.Filter = 1;
  config.Mode = 0;

  /*500 Kbps  0x00  0x1C*/
  config.Timing0 = 0x01;
  config.Timing1 = 0x1C;

  if (VCI_InitCAN(VCI_USBCAN2, device_index_, can_index_, &config) != 1) {
    ROS_ERROR_STREAM("init CAN 0 error");
    VCI_CloseDevice(VCI_USBCAN2, device_index_);
  }

  if (VCI_StartCAN(VCI_USBCAN2, device_index_, can_index_) != 1) {
    ROS_ERROR_STREAM("Start CAN 0 error");
    VCI_CloseDevice(VCI_USBCAN2, device_index_);
  }

  // open CAN2 for loop test
  if (VCI_InitCAN(VCI_USBCAN2, device_index_, 1, &config) != 1) {
    ROS_ERROR_STREAM("init CAN 1 error");
    VCI_CloseDevice(VCI_USBCAN2, device_index_);
  }

  if (VCI_StartCAN(VCI_USBCAN2, device_index_, 1) != 1) {
    ROS_ERROR_STREAM("Start CAN 1 error");
    VCI_CloseDevice(VCI_USBCAN2, device_index_);
  }

  run_flag_ = 1;

  int ret;
  ret = pthread_create(&thread_0_, NULL, CanBridge::FuncHelper, this);
  ROS_ERROR_STREAM("thread return value: " << ret << std::endl);
}

void CanBridge::CanClose() {
  run_flag_ = 0;
  pthread_join(thread_0_, NULL);
  VCI_CloseDevice(VCI_USBCAN2, device_index_);
}

bool CanBridge::Write(const vector<can_msgs::Frame>& msgs) {
  // ROS_INFO_STREAM("msgs length: " << msgs.size());
  for (const auto& each : msgs) {
    if (each.data.size() > 8) {
      ROS_ERROR_STREAM("The size of can message must less than 8");
      return false;
    }
    VCI_CAN_OBJ to_send[1];
    to_send[0].ID = each.id;
    to_send[0].SendType = 1;
    to_send[0].RemoteFlag = each.is_rtr;
    to_send[0].ExternFlag = each.is_extended;
    to_send[0].DataLen = 8;

    unsigned int index = 0;
    for (const auto& it : each.data) {
	    to_send[0].Data[index++] = it;
    }
    if (VCI_Transmit(VCI_USBCAN2, device_index_, can_index_, to_send, 1) > 0) {
	    ROS_INFO_STREAM("Send suc: " << to_send[0].ID);
	    for (int i = 0; i < 3; ++i){	
		    ROS_INFO_STREAM("Data of current frame, index: " << i <<", data: " << static_cast<double>(to_send[0].Data[i]));
	    }

    } else {
	    ROS_ERROR_STREAM("Transmit error!");
    }
  }
  return true;
}

}  // namespace cantools
