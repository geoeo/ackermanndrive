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
        boost::shared_ptr<tf::TransformBroadcaster> transform_broadcaster_;


        boost::mutex lock;
        boost::mutex IWS_message_lock;
        bool gazebo_debug;

        //Robot Parameters
        double wheelbase;
        double track;
        double steeringwidth;
        double wheel_diameter;
        double steering_velocity;
        double streeringtorque;
        double wheeltorque;
        double max_steering_angle;
        double max_revolute_velocity;
        double max_steering_omega;
        double steering_acceleration;
        double linear_acceleration;

        double wheel_radius_;

        //Joints
        std::vector<physics::JointPtr> steerings_;
        std::vector<physics::JointPtr> wheels_;
        std::vector<physics::JointPtr> front_wheels_;
        std::vector<double> front_wheels_previous_rotation_;
        std::vector<double> front_wheels_rotation_delta_;

        //Rostopic
        std::string command_topic_;
        std::string joint_iws_topic;
        std::string odometry_topic_encoder_;
        std::string odometry_topic_godview_;

        // ROS Pub/Subscribe
        ros::Subscriber command_subscriber_;
        ros::Subscriber command_subscriber_iws_;

        ros::Publisher joint_iws_publisher_;

        tuw_nav_msgs::JointsIWS cmd_iws_publish_;

        // Custom Callback Queue
        ros::CallbackQueue queue_;
        boost::thread callback_queue_thread_;
        void QueueThread();

        void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg);
        void cmdIWSCallback(const tuw_nav_msgs::JointsIWS::ConstPtr& cmd_msg);
        void PublishJointIWS();
        void PublishDeadReckoningMotionModel();
        void PublishVelocityMotionModel();

        void UpdateChild();

        double OmegaFromTricicleModel(double steering_angle);
        double CalculateVelocityWithAcceleration(double current_velocity, double target_velocity, double acceleration,double limit);

        double RoundTo(double value, int decimal_places);
        bool IsBetween(double value, double min, double max);

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
        ros::Time vel_last_update_time_;

        double wheel_velocity_;
        double steering_omega_;
        double steering_angle_;

        double target_velocity;
        double angle_center;

        double actual_angle_[2];
        double actual_velocity[2];


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


      Pose pose;

    };
}

#endif //PROJECT_ACKERMANNDRIVE_IWS_PLUGIN_HPP
