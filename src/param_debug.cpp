#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

double expect_angle;
double real_angle;
double expect_v;
double real_v;

void SteerFeedBack(std_msgs::Float64MultiArray msgs) {
    real_angle = msgs.data[0];
}

void CmdVel(geometry_msgs::Twist msgs) {
    expect_v = msgs.linear.x;
    expect_angle = msgs.angular.z;
}

void Odom(nav_msgs::Odometry msgs) {
    real_v = msgs.twist.twist.linear.x;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "debug");
    ros::NodeHandle n;
    ros::Subscriber cmd_vel_sub, steer_sub, odom_sub;
    ros::Publisher debug_pub;
    ros::Rate rate(20);
    cmd_vel_sub = n.subscribe("/decision/cmd_vel",1 ,CmdVel);
    steer_sub = n.subscribe("/steer_feedback_swa",1 , SteerFeedBack);
    odom_sub = n.subscribe("/golfcar/odom",1, Odom);
    debug_pub = n.advertise<geometry_msgs::Twist>("debug_param", 1);

    geometry_msgs::Twist debug_msg;
    while(ros::ok()) {
        debug_msg.linear.x = expect_v;
        debug_msg.linear.y = real_v;
        debug_msg.angular.x = expect_angle;
        debug_msg.angular.y = real_angle;
        debug_pub.publish(debug_msg);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}

