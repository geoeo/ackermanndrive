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

    // Pose Update p. 395
    void ApplyMotion(MotionDelta delta, double dt){

        x += (delta.deltaX*cos(theta) - delta.deltaY*sin(theta))*dt;
        y += (delta.deltaX*sin(theta) + delta.deltaY*cos(theta))*dt;
        theta += delta.deltaTheta*dt;

    }
};





void IWS_Callback(const tuw_nav_msgs::JointsIWS::ConstPtr& cmd_msg);

MotionDelta CalculateAckermannMotionDelta(tuw_nav_msgs::JointsIWS actionInputs);


double wheel_base = 0.26; //TODO: Get this from config
double steering_velocity = 25.0;
double max_steering_omega = 25.0;
bool ableToCalculateDeltaTime = false;
double deltaTime = 0.0;
double gazebo_update_rate = 33.0;


#endif //PROJECT_ACKERMANN_MOTION_MODEL_HPP
