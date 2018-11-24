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
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>
#endif

namespace golfmiddleware{
    class JoyParser: public nodelet::Nodelet
    {
    public:
        JoyParser();

    private:
        virtual void onInit();
        void JoystickHandler(sensor_msgs::JoyConstPtr joy_const_ptr);
        ros::Subscriber joy_sub_;
        ros::NodeHandle nh_;

        ros::Publisher honk_pub_, brake_pub_, cmd_vel_pub_, manual_enable_pub_;
        ros::Publisher clean_costmap_pub_;
        std_msgs::Bool brake_state_msg_, manual_enable_msg_, honk_state_msg_;
        std_msgs::Bool clean_costmap_msg_;
        geometry_msgs::Twist cmd_vel_msg_;

        ros::Publisher auto_start_pub_;
        std_msgs::Bool auto_start_msg_;

        void CmdVelClean();
        void CmdVelUpdateAndPub(sensor_msgs::JoyConstPtr joy_const_ptr);

        double linear_scale_ = 0.5, angular_scale_ = 45.0;

    };
}

PLUGINLIB_EXPORT_CLASS(golfmiddleware::JoyParser, nodelet::Nodelet);

namespace golfmiddleware{
    JoyParser::JoyParser() {}

    void JoyParser::onInit()
    {
        nh_ = getPrivateNodeHandle();
        joy_sub_ = nh_.subscribe("joy", 1, &JoyParser::JoystickHandler, this);

        brake_pub_ = nh_.advertise<std_msgs::Bool>("brake",1);
        honk_pub_ = nh_.advertise<std_msgs::Bool>("honk",1);
        manual_enable_pub_ = nh_.advertise<std_msgs::Bool>("manual_enable",1);
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel",1);
        auto_start_pub_ = nh_.advertise<std_msgs::Bool>("auto_start",1);
        clean_costmap_pub_ = nh_.advertise<std_msgs::Bool>("clean_costmap",1);


        nh_.param<double>("linear_scale", linear_scale_, 0.5);
        nh_.param<double>("angular_scale", angular_scale_, 45);

        brake_state_msg_.data = false;
        honk_state_msg_.data = false;
        manual_enable_msg_.data = false;
        auto_start_msg_.data = false;
        clean_costmap_msg_.data = false;
        CmdVelClean();

        manual_enable_pub_.publish(manual_enable_msg_);
        brake_pub_.publish(brake_state_msg_);
        honk_pub_.publish(honk_state_msg_);
        clean_costmap_pub_.publish(clean_costmap_msg_);
        auto_start_pub_.publish(auto_start_msg_);
        cmd_vel_pub_.publish(cmd_vel_msg_);
    }

    void JoyParser::JoystickHandler(sensor_msgs::JoyConstPtr joy_const_ptr)
    {
        // This is for braking
        if (1 == joy_const_ptr->buttons[2])
        {
            if (false == brake_state_msg_.data) {
                // state change: from false -> true
                brake_state_msg_.data = true;
                brake_pub_.publish(brake_state_msg_);
                CmdVelClean();
            } else {
                // no change: from true -> true
            }
        } else if (0 == joy_const_ptr->buttons[2])
        {
            if (true == brake_state_msg_.data){
                //state change: from true -> false
                brake_state_msg_.data = false;
                brake_pub_.publish(brake_state_msg_);
            } else {
                // no change: from false -> false
            }
        }

        // This is for honking
        if (1 == joy_const_ptr->buttons[4])
        {
            if (false == honk_state_msg_.data) {
                // state change: from false -> true
                honk_state_msg_.data = true;
                honk_pub_.publish(honk_state_msg_);
            } else {
                // no change: from true -> true
            }
        }
        else if (0 == joy_const_ptr->buttons[4])
        {
            if (true == honk_state_msg_.data){
                //state change: from true -> false
                honk_state_msg_.data = false;
                honk_pub_.publish(honk_state_msg_);
            } else {
                // no change: from false -> false
            }
        }

        // This is for cmd_vel commands
        if (1 == joy_const_ptr->buttons[7])
        {
            if (false == manual_enable_msg_.data){
                // state change: from false -> true
                manual_enable_msg_.data = true;
                manual_enable_pub_.publish(manual_enable_msg_);
            } else {
                // no change: from true -> true;
            }
            // Update and publish cmd_vel if enable is true
            if (false == brake_state_msg_.data){
                CmdVelUpdateAndPub(joy_const_ptr);
            }
        }
        else if (0 == joy_const_ptr->buttons[7])
        {
            if (true == manual_enable_msg_.data)
            {
                // state change: from true-> false
                manual_enable_msg_.data = false;
                manual_enable_pub_.publish(manual_enable_msg_);
                CmdVelClean();
            } else
            {
                // no change: from false -> false
            }
        }

        //This is for auto_mode start
        if (1 == joy_const_ptr->buttons[8] && 1 == joy_const_ptr->buttons[9])
        {
            if (false == auto_start_msg_.data)
            {
                //state change: from false -> true
                auto_start_msg_.data = true;
                auto_start_pub_.publish(auto_start_msg_);
            } else
            {
                // no change: from true -> true
            }
        }
        else
        {
            if (true == auto_start_msg_.data)
            {
                //state change: from true -> false
                auto_start_msg_.data = false;
                auto_start_pub_.publish(auto_start_msg_);
            } else
            {
                // no change: from false -> false
            }
        }

        //This is for clean_costmap
        if (1 == joy_const_ptr->buttons[5])
        {
            if (false == clean_costmap_msg_.data)
            {
                //state change: from false -> true
                clean_costmap_msg_.data = true;
                clean_costmap_pub_.publish(clean_costmap_msg_);
            } else
            {
                // no change: from true -> true
            }
        }
        else if (0 == joy_const_ptr->buttons[5])
        {
            if (true == clean_costmap_msg_.data)
            {
                //state change: from true -> false
                clean_costmap_msg_.data = false;
                clean_costmap_pub_.publish(clean_costmap_msg_);
            } else
            {
                // no change: from false -> false
            }
        }
    }

    void JoyParser::CmdVelClean()
    {
        cmd_vel_msg_.linear.x = 0;
        cmd_vel_pub_.publish(cmd_vel_msg_);
    }

    void JoyParser::CmdVelUpdateAndPub(sensor_msgs::JoyConstPtr joy_const_ptr)
    {
        cmd_vel_msg_.linear.x = linear_scale_ * joy_const_ptr->axes[1];
        if (fabs(cmd_vel_msg_.linear.x) < 0.05)
        {
            cmd_vel_msg_.linear.x = 0.0;
        }
        cmd_vel_msg_.angular.z = angular_scale_ * joy_const_ptr->axes[2];
        cmd_vel_pub_.publish(cmd_vel_msg_);
    }

}
