/**
******************************************************************************
* Copyright (C) 2018 Joey.Liu <lty2226262@gmail.com>
* Distributed under terms of the MIT license.
******************************************************************************
*/

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "ramlab_msgs.h"

#ifndef RAMLAB_RELEASE
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#endif

namespace golfmiddleware{
    class CmdVelMux: public nodelet::Nodelet
    {
    public:
        CmdVelMux();

    private:
        virtual void onInit();
        ros::NodeHandle nh_;
        ros::Subscriber manual_cmd_vel_sub_, auto_cmd_vel_sub_;
        void ManualCmdVelHandler(geometry_msgs::TwistConstPtr twist_const_ptr);
        void AutoCmdVelHandler(geometry_msgs::TwistConstPtr twist_const_ptr);
        ros::Publisher decision_cmd_vel_pub_;
        geometry_msgs::Twist decision_cmd_vel_msg_;

        typedef enum {
            IDLE = 0,
            MANUAL,
            AUTO
        } ControlState;

        ControlState current_state_;
        ros::Subscriber auto_start_sub_, manual_enable_sub_;
        void AutoStartHandler(std_msgs::Bool auto_start_msg);
        void ManualEnableHandler(std_msgs::Bool manual_msg);
        void TurnToIdle();

        double linear_limit_ = 0.0, angular_limit_ = 0.0;
        void NormalizeAndPub();
    };
}

PLUGINLIB_EXPORT_CLASS(golfmiddleware::CmdVelMux, nodelet::Nodelet);

namespace golfmiddleware{
    CmdVelMux::CmdVelMux() {}

    void CmdVelMux::onInit()
    {
        nh_ = getPrivateNodeHandle();

        nh_.param<double>("linear_limit", linear_limit_, 2.0);
        nh_.param<double>("angular_limit", angular_limit_, 45.0);

        manual_cmd_vel_sub_ = nh_.subscribe("/joy/cmd_vel", 1, &CmdVelMux::ManualCmdVelHandler, this);
        auto_cmd_vel_sub_ = nh_.subscribe("/auto/cmd_vel", 1, &CmdVelMux::AutoCmdVelHandler, this);

        auto_start_sub_ = nh_.subscribe("/joy_parser/auto_start", 1, &CmdVelMux::AutoStartHandler, this);
        manual_enable_sub_ = nh_.subscribe("/joy_parser/manual_enable", 1, &CmdVelMux::ManualEnableHandler, this);

        decision_cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/decision/cmd_vel",3);

        decision_cmd_vel_msg_.angular.z = 0;
        TurnToIdle();
    }

    void CmdVelMux::ManualCmdVelHandler(geometry_msgs::TwistConstPtr twist_const_ptr)
    {
        if (MANUAL == current_state_)
        {
            decision_cmd_vel_msg_.linear.x = twist_const_ptr->linear.x;
            decision_cmd_vel_msg_.angular.z = twist_const_ptr->angular.z;
            NormalizeAndPub();
        } else
        {
            ROS_WARN_STREAM("Get joy command at IDLE/AUTO state");
        }
    }

    void CmdVelMux::AutoCmdVelHandler(geometry_msgs::TwistConstPtr twist_const_ptr)
    {
        if (AUTO == current_state_)
        {
            decision_cmd_vel_msg_.linear.x = twist_const_ptr->linear.x;
            decision_cmd_vel_msg_.angular.z = twist_const_ptr->angular.z;
            NormalizeAndPub();
        } else {
            // get auto commands at idle/manual state
        }
    }

    void CmdVelMux::AutoStartHandler(std_msgs::Bool auto_start_msg)
    {
        if (true == auto_start_msg.data)
        {
            if (IDLE == current_state_)
            {
                current_state_ = AUTO;
            }
            else
            {
                //Already auto or change to manual immediately
            }
        }
        else
        {
            //false
        }
    }

    void CmdVelMux::ManualEnableHandler(std_msgs::Bool manual_msg)
    {
        if (true == manual_msg.data)
        {
            current_state_ = MANUAL;
        }
        else
        {
            TurnToIdle();
        }
    }

    void CmdVelMux::TurnToIdle()
    {
        current_state_ = IDLE;
        decision_cmd_vel_msg_.linear.x = 0.0;
        NormalizeAndPub();
    }

    void CmdVelMux::NormalizeAndPub()
    {
        if (decision_cmd_vel_msg_.linear.x > linear_limit_)
        {
            decision_cmd_vel_msg_.linear.x = linear_limit_;
        }
        else if (decision_cmd_vel_msg_.linear.x < -linear_limit_)
        {
            decision_cmd_vel_msg_.linear.x = -linear_limit_;
        }

        if (decision_cmd_vel_msg_.angular.z > angular_limit_)
        {
            decision_cmd_vel_msg_.angular.z = angular_limit_;
        }
        else if (decision_cmd_vel_msg_.angular.z < -angular_limit_)
        {
            decision_cmd_vel_msg_.angular.z = -angular_limit_;
        }

        decision_cmd_vel_pub_.publish(decision_cmd_vel_msg_);
    }
}
