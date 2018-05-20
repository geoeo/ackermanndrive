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

    void ApplyMotion_2(MotionDelta delta, double dt){

        x += delta.deltaX*dt;
        y += delta.deltaY*dt;
        theta += delta.deltaTheta*dt;

    }
};





void IWS_Callback(const tuw_nav_msgs::JointsIWS::ConstPtr& cmd_msg);

MotionDelta CalculateAckermannMotionDelta(tuw_nav_msgs::JointsIWS actionInputs);
MotionDelta CalculateAckermannMotionDelta_2(tuw_nav_msgs::JointsIWS actionInputs);
MotionDelta CalculateAckermannMotionDelta_3(tuw_nav_msgs::JointsIWS actionInputs,Pose& pose);

Pose robotPose;
Pose robotPose_2;
Pose robotPose_3;

ros::Time current_time, last_time;

//TODO: Get this from config
double wheel_base = 0.26;
double steering_velocity = 25.0;
double max_steering_omega = 25.0;
bool ableToCalculateDeltaTime = false;
double deltaTime = 0.0;
double gazebo_update_rate = 30.0; // this has to by snyced with gazebo frame rate for optimal results
double gazebo_noise_factor_linear_velocity = 1.0;


#endif //PROJECT_ACKERMANN_MOTION_MODEL_HPP
