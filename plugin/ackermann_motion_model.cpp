//
// Created by marc on 04.05.18.
//

#include "ackermann_motion_model.hpp"

boost::mutex IWS_message_lock;
tuw_nav_msgs::JointsIWS current_iws;


double RoundTo(double value, int decimal_places) {

    double factor = pow(10.0,(double)decimal_places);
    return ceil(value*factor)/factor;
}

// http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom

int main(int argc, char** argv){
    ros::init(argc, argv, "odometry_publisher");

    ros::NodeHandle n;
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    tf::TransformBroadcaster odom_broadcaster;
    ros::Subscriber iws_sub = n.subscribe("iws_channel",1,IWS_Callback);

    current_iws.revolute.resize (2);
    current_iws.steering.resize (2);
    current_iws.steering[0] = 0.0;
    current_iws.revolute[0] = std::nan("1");
    current_iws.steering[1] = std::nan("1");
    current_iws.revolute[1] = 0.0;
    current_iws.header.stamp = ros::Time(0);


    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    ros::Rate r(gazebo_update_rate);
    deltaTime = 0.0;

    // strange startup values
    ros::Duration(2).sleep();
    while(n.ok()){

        ros::spinOnce();              // check for incoming messages
        //current_time = ros::Time::now();
        current_time = current_iws.header.stamp;
        //deltaTime = (current_time - last_time).toSec();
        if(!ableToCalculateDeltaTime)
            continue;

        //TODO use time from header

        MotionDelta motionDelta = CalculateAckermannMotionDelta_2(current_iws);

        //ROS_INFO("dt: %f",deltaTime);
        //ROS_INFO("dw: %f", motionDelta.deltaTheta);

        robotPose.ApplyMotion_2(motionDelta,deltaTime);
        //robotPose.ApplyMotion(motionDelta,1.0);

        //ROS_INFO("new x: %f",robotPose.x);
        //ROS_INFO("new y: %f",robotPose.y);


        //since all odometry is 6DOF we'll need a quaternion created from yaw
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(robotPose.theta);
        double w = odom_quat.w;
        //odom_quat.w = odom_quat.z;
        //odom_quat.z = w;

        //first, we'll publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "map";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = robotPose.x;
        odom_trans.transform.translation.y = robotPose.y;
        //odom_trans.transform.translation.x = 0.0;
        //odom_trans.transform.translation.y = 0.0;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        //send the transform

        odom_broadcaster.sendTransform(odom_trans);

        //next, we'll publish the odometry message over ROS
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "map";

        //set the position
        odom.pose.pose.position.x = robotPose.x;
        odom.pose.pose.position.y = robotPose.y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        //set the velocity
        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = motionDelta.deltaX;
        odom.twist.twist.linear.y = motionDelta.deltaY;
        odom.twist.twist.angular.z = motionDelta.deltaTheta;

        //publish the message
        odom_pub.publish(odom);

        last_time = current_time;
        r.sleep();
    }
}


void IWS_Callback(const tuw_nav_msgs::JointsIWS::ConstPtr& cmd_msg){

    current_iws.revolute[1] = RoundTo(cmd_msg->revolute[1],1);
    current_iws.steering[0] = RoundTo(cmd_msg->steering[0],2);
    // skip first message
    if(!current_iws.header.stamp.isZero()){
        deltaTime = (cmd_msg->header.stamp-current_iws.header.stamp).toSec()/gazebo_update_rate;
        ableToCalculateDeltaTime = true;
    }
    current_iws.header.stamp = cmd_msg->header.stamp;


    //ROS_INFO("RECEIVED: %f",  current_iws.revolute[1]);
    //ROS_INFO("Steering: %f",  current_iws.steering[0]);
}

// Madrigal, USe with velocity update #1
MotionDelta CalculateAckermannMotionDelta(tuw_nav_msgs::JointsIWS actionInputs){
    boost::mutex::scoped_lock scoped_lock ( IWS_message_lock );


    double linear_velocity = actionInputs.revolute[1];
    double steering_angle = actionInputs.steering[0];

    MotionDelta motionDelta;

    double sin_steering = RoundTo(sin(steering_angle),1);

    // Intro. SLAM by Juan-Antonio Fernandez p. 164
    // Should be same error thresh. as in iws plugin
    if(fabs(steering_angle) < 0.1){
        motionDelta.deltaX = linear_velocity;
        motionDelta.deltaY = 0.0;
        motionDelta.deltaTheta = 0.0;
    }

    else if( fabs(linear_velocity) > gazebo_noise_factor_linear_velocity){

        motionDelta.deltaTheta = steering_velocity *sin_steering/wheel_base;

        //
        if(motionDelta.deltaTheta > max_steering_omega) motionDelta.deltaTheta = max_steering_omega;
        else if (motionDelta.deltaTheta < -max_steering_omega) motionDelta.deltaTheta = -max_steering_omega;

        //TODO: investigate these scaling parameters
        motionDelta.deltaTheta *= 1.5; // increases curve traversal speed

        motionDelta.deltaX = -9.0*linear_velocity*wheel_base*sin(motionDelta.deltaTheta)/tan(steering_angle);
        motionDelta.deltaY = -9.0*linear_velocity*wheel_base*(1.0-cos(motionDelta.deltaTheta))/tan(steering_angle);

    }

    else {
        motionDelta.deltaTheta = 0.0;
        motionDelta.deltaX = 0.0;
        motionDelta.deltaY = 0.0;
    }

    //ROS_INFO("dx: %f", motionDelta.deltaX);
    //ROS_INFO("dy: %f", motionDelta.deltaY);
    //ROS_INFO("dtheta: %f", motionDelta.deltaTheta);
    //ROS_INFO("dt: %f", deltaTime);

    return motionDelta;



}

// https://pdfs.semanticscholar.org/5849/770f946e7880000056b5a378d2b7ac89124d.pdf , use with velocity update #2
MotionDelta CalculateAckermannMotionDelta_2(tuw_nav_msgs::JointsIWS actionInputs){
    boost::mutex::scoped_lock scoped_lock ( IWS_message_lock );


    double linear_velocity = actionInputs.revolute[1];
    double steering_angle = actionInputs.steering[0];

    MotionDelta motionDelta;


    // Intro. SLAM by Juan-Antonio Fernandez p. 164
    // Should be same error thresh. as in iws plugin
    if(fabs(steering_angle) < 0.1){
        motionDelta.deltaX = linear_velocity;
        motionDelta.deltaY = 0.0;
        motionDelta.deltaTheta = 0.0;
    }

    else if( fabs(linear_velocity) > gazebo_noise_factor_linear_velocity){

        motionDelta.deltaTheta = steering_velocity *tan(steering_angle)/wheel_base;

        motionDelta.deltaX = linear_velocity*sin(M_PI/2.0 - robotPose.theta);
        motionDelta.deltaY = linear_velocity*cos(M_PI/2.0 - robotPose.theta);

    }

    else {
        motionDelta.deltaTheta = 0.0;
        motionDelta.deltaX = 0.0;
        motionDelta.deltaY = 0.0;
    }

    //ROS_INFO("dx: %f", motionDelta.deltaX);
    //ROS_INFO("dy: %f", motionDelta.deltaY);
    //ROS_INFO("dtheta: %f", motionDelta.deltaTheta);
    //ROS_INFO("dt: %f", deltaTime);

    return motionDelta;



}

// https://robotics.stackexchange.com/questions/4486/ackerman-steering-model , use with vel update #2
MotionDelta CalculateAckermannMotionDelta_3(tuw_nav_msgs::JointsIWS actionInputs){
    boost::mutex::scoped_lock scoped_lock ( IWS_message_lock );


    double linear_velocity = actionInputs.revolute[1];
    double steering_angle = actionInputs.steering[0];

    MotionDelta motionDelta;

    // Should be same error thresh. as in iws plugin
    if(fabs(steering_angle) < 0.1){
        motionDelta.deltaX = linear_velocity;
        motionDelta.deltaY = 0.0;
        motionDelta.deltaTheta = 0.0;
    }

    else if( fabs(linear_velocity) > gazebo_noise_factor_linear_velocity){

        motionDelta.deltaTheta = steering_velocity *tan(steering_angle)/wheel_base;

        motionDelta.deltaX = linear_velocity*cos(robotPose.theta + (motionDelta.deltaTheta*deltaTime)/2.0);
        motionDelta.deltaY = linear_velocity*sin(robotPose.theta + (motionDelta.deltaTheta*deltaTime)/2.0);

    }

    else {
        motionDelta.deltaTheta = 0.0;
        motionDelta.deltaX = 0.0;
        motionDelta.deltaY = 0.0;
    }

    //ROS_INFO("dx: %f", motionDelta.deltaX);
    //ROS_INFO("dy: %f", motionDelta.deltaY);
    //ROS_INFO("dtheta: %f", motionDelta.deltaTheta);
    //ROS_INFO("dt: %f", deltaTime);

    return motionDelta;



}

MotionDelta CalculateHolonomicMotionDelta(tuw_nav_msgs::JointsIWS actionInputs){
    boost::mutex::scoped_lock scoped_lock ( IWS_message_lock );


    double linear_velocity = actionInputs.revolute[1];
    double steering_angle = actionInputs.steering[0];

    MotionDelta motionDelta;

    double sin_steering = RoundTo(sin(steering_angle),1);

    // Intro. SLAM by Juan-Antonio Fernandez p. 164
    // Should be same error thresh. as in iws plugin
    if(fabs(steering_angle) < 0.1){
        motionDelta.deltaX = linear_velocity;
        motionDelta.deltaY = 0.0;
        motionDelta.deltaTheta = 0.0;
    }

    else if( fabs(linear_velocity) > gazebo_noise_factor_linear_velocity){

        motionDelta.deltaTheta = steering_velocity*sin(steering_angle)/wheel_base;

        if(motionDelta.deltaTheta > max_steering_omega) motionDelta.deltaTheta = max_steering_omega;
        else if (motionDelta.deltaTheta < -max_steering_omega) motionDelta.deltaTheta = -max_steering_omega;

        motionDelta.deltaX = linear_velocity*wheel_base*cos(motionDelta.deltaTheta)/tan(steering_angle);
        motionDelta.deltaY = linear_velocity*wheel_base*sin(motionDelta.deltaTheta)/tan(steering_angle);

    }

    else {
        motionDelta.deltaTheta = 0.0;
        motionDelta.deltaX = 0.0;
        motionDelta.deltaY = 0.0;
    }

    //ROS_INFO("dx: %f", motionDelta.deltaX);
    //ROS_INFO("dy: %f", motionDelta.deltaY);
    //ROS_INFO("dtheta: %f", motionDelta.deltaTheta);
    //ROS_INFO("dt: %f", deltaTime);

    return motionDelta;



}


