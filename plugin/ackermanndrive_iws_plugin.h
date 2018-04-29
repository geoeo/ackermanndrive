//
// Created by marc on 24.04.18.
//

#ifndef PROJECT_ACKERMANNDRIVE_IWS_PLUGIN_HPP
#define PROJECT_ACKERMANNDRIVE_IWS_PLUGIN_HPP

#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include <tuw_geometry/tuw_geometry.h>

// Gazebo
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/math/gzmath.hh>
#include <gazebo_plugins/gazebo_ros_utils.h>

// ROS
//#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <tuw_nav_msgs/JointsIWS.h>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>


namespace gazebo {
    class Ackermannplugin_IWS : public ModelPlugin {
    public:
        Ackermannplugin_IWS();		//Constructor
        ~Ackermannplugin_IWS();		//Destructor
        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
    private:
        physics::ModelPtr parent;
        GazeboRosPtr gazebo_ros_;
        event::ConnectionPtr update_connection_;


        boost::mutex lock;
        boost::mutex IWS_message_lock;
        bool gazebo_debug;

        //Robot Parameters
        double wheelbase;
        double track;
        double steeringwidth;
        double wheeldiameter;
        double steeringVelocity;
        double streeringtorque;
        double wheeltorque;
        double steeringangle;

        //Joints
        std::vector<physics::JointPtr> steerings_;
        std::vector<physics::JointPtr> wheels_;

        //Rostopic
        std::string command_topic_;
        std::string odometry_topic_encoder_;
        std::string odometry_topic_godview_;
        ros::Subscriber command_subscriber_;
        ros::Subscriber command_subscriber_iws_;

        // Custom Callback Queue
        ros::CallbackQueue queue_;
        boost::thread callback_queue_thread_;
        void QueueThread();

        void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg);
        void cmdIWSCallback(const tuw_nav_msgs::JointsIWS::ConstPtr& cmd_msg);

        void UpdateChild();
        double roundTo(double value, int decimal_places);

        //Odometry
        double v;
        double w;


        bool reset;


        std::string odometry_frame_;
        std::string robot_base_frame_;

        //Update
        double update_rate_;
        double update_period_;
        common::Time last_update_time_;

        double wheel_velocity;
        double steering_omega;

        double target_velocity;
        double angle_center;


        double actual_angle_[2];

    };
}

#endif //PROJECT_ACKERMANNDRIVE_IWS_PLUGIN_HPP
