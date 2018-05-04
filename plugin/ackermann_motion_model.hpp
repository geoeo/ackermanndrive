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

struct MotionDelta {
    double deltaX = 0.0;
    double deltaY = 0.0;
    double deltaTheta = 0.0;
};

struct Pose {
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;

    void ApplyMotion(MotionDelta delta, double dt){

        x = x + (delta.deltaX*cos(theta) - delta.deltaY*sin(theta))*dt;
        y = y + (delta.deltaX*sin(theta) + delta.deltaY*cos(theta))*dt;
        theta += delta.deltaTheta*dt;

    }
};





void IWS_Callback(const tuw_nav_msgs::JointsIWS::ConstPtr& cmd_msg);

MotionDelta CalculateAckermannMotionDelta(tuw_nav_msgs::JointsIWS actionInputs);

Pose ApplyMotionToPose(Pose p, MotionDelta delta);


double wheel_base = 0.26; //TODO: Get this from config


#endif //PROJECT_ACKERMANN_MOTION_MODEL_HPP
