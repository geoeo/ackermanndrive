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


    Pose robotPose;

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    ros::Rate r(1.0);
    while(n.ok()){

        ros::spinOnce();               // check for incoming messages
        current_time = ros::Time::now();

        //compute odometry in a typical way given the velocities of the robot
        double dt = (current_time - last_time).toSec();
        //double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
        //double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
        //double delta_th = vth * dt;

        //x += delta_x;
        //y += delta_y;
        //th += delta_th;

        MotionDelta motionDelta = CalculateAckermannMotionDelta(current_iws);

        robotPose.ApplyMotion(motionDelta,dt);
        //robotPose.ApplyMotion(motionDelta,1.0);


        //since all odometry is 6DOF we'll need a quaternion created from yaw
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(robotPose.theta);

        //first, we'll publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = robotPose.x;
        odom_trans.transform.translation.y = robotPose.y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        //send the transform
        odom_broadcaster.sendTransform(odom_trans);

        //next, we'll publish the odometry message over ROS
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";

        //set the position
        odom.pose.pose.position.x = robotPose.x;
        odom.pose.pose.position.y = robotPose.y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        //set the velocity
        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = motionDelta.deltaX;
        odom.twist.twist.linear.y = motionDelta.deltaY;
        odom.twist.twist.angular.z = motionDelta.deltaY;

        //publish the message
        odom_pub.publish(odom);

        last_time = current_time;
        r.sleep();
    }
}


void IWS_Callback(const tuw_nav_msgs::JointsIWS::ConstPtr& cmd_msg){
    boost::mutex::scoped_lock scoped_lock ( IWS_message_lock );

    current_iws.revolute[1] = RoundTo(cmd_msg->revolute[1],1);
    current_iws.steering[0] = RoundTo(cmd_msg->steering[0],2);

    ROS_INFO("RECEIVED: %f",  current_iws.revolute[1]);
    ROS_INFO("Steering: %f",  current_iws.steering[0]);
}

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

    else {
        motionDelta.deltaTheta = linear_velocity*sin_steering/wheel_base;
        motionDelta.deltaX = wheel_base*sin(motionDelta.deltaTheta)/tan(steering_angle);
        motionDelta.deltaY = wheel_base*(1.0-cos(motionDelta.deltaTheta)/tan(steering_angle));
    }

    return motionDelta;



}


