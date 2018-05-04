//
// Created by marc on 04.05.18.
//

#ifndef PROJECT_ACKERMANN_MOTION_MODEL_HPP
#define PROJECT_ACKERMANN_MOTION_MODEL_HPP

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <tuw_nav_msgs/JointsIWS.h>
#include <boost/thread.hpp>

struct Motion_Delta {
    double deltaX;
    double deltaY;
    double deltaTheta;
};


boost::mutex IWS_message_lock;

std::string child_frame = "base_link";
std::string iws_channel = "iws_channel";

void IWS_Callback(const tuw_nav_msgs::JointsIWS::ConstPtr& cmd_msg);

Motion_Delta CalculateMotionDelta(tuw_nav_msgs::JointsIWS actionInputs);

tuw_nav_msgs::JointsIWS current_iws;


#endif //PROJECT_ACKERMANN_MOTION_MODEL_HPP
