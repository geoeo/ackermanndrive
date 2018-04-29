//
// Created by marc on 24.04.18.
//

#include "ackermanndrive_iws_plugin.h"

namespace gazebo {

    enum {
        RIGHT,
        LEFT,
    };

    // Needed for GZ_REGISTER_MODEL_PLUGIN macro
    Ackermannplugin_IWS::Ackermannplugin_IWS() {}
    Ackermannplugin_IWS::~Ackermannplugin_IWS() {}

    void Ackermannplugin_IWS::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
        parent = _parent;
        ROS_INFO("Ackermann IWS Drive Plugin loading...");
        gazebo_ros_ = GazeboRosPtr ( new GazeboRos ( _parent, _sdf, "Ackeramnndrive_IWS" ) );
        gazebo_ros_ -> isInitialized();

        // getting parameters from plugin.xacro
        gazebo_ros_ -> getParameter<std::string> ( command_topic_, "CommandTopic", "command" );
        gazebo_ros_ -> getParameter<std::string> ( odometry_topic_encoder_, "OdometryTopicEncoder", "odometry_encoder" );
        gazebo_ros_ -> getParameter<std::string> ( odometry_topic_godview_, "OdometryTopicGodview", "odometry_godview" );
        gazebo_ros_ -> getParameter<std::string> ( odometry_frame_, 	"odometryFrame", "odom" );
        gazebo_ros_ -> getParameter<std::string> ( robot_base_frame_, "robotBaseFrame", "base_link" );
        gazebo_ros_ -> getParameter<double> ( wheelbase, 		"Wheelbase",		0.0 );
        gazebo_ros_ -> getParameter<double> ( track, 			"Track",		0.0 );
        gazebo_ros_ -> getParameter<double> ( steeringwidth, 		"SteeringWidth",	0.0 );
        gazebo_ros_ -> getParameter<double> ( wheeldiameter, 		"Wheeldiameter",	0.0 );
        gazebo_ros_ -> getParameter<double> ( steeringVelocity, 	"SteeringVelocity",	 10.0 );
        gazebo_ros_ -> getParameter<double> ( streeringtorque, 		"Streeringtorque",	10.0 );
        gazebo_ros_ -> getParameter<double> ( steeringangle, 		"Steeringangle",	0.0  );
        gazebo_ros_ -> getParameter<double> ( wheeltorque,			"Wheeltorque",	10.0 );
        gazebo_ros_ -> getParameter<bool> ( gazebo_debug,			"GazeboDebug",	false );

        target_velocity = 0.0;

        // steering
        steerings_.resize ( 2 );
        steerings_[LEFT ] = gazebo_ros_ -> getJoint ( parent, "LeftSteering",  "leftsteering"  );
        steerings_[RIGHT] = gazebo_ros_ -> getJoint ( parent, "RightSteering", "rightsteering" );
        steerings_[LEFT ] -> SetParam( "fmax", 0, streeringtorque );
        steerings_[RIGHT] -> SetParam( "fmax", 0, streeringtorque );

        // wheels
        wheels_.resize ( 2 );
        wheels_[LEFT ] = gazebo_ros_ -> getJoint ( parent, "LeftRearWheel",  "leftrearwheel"  );
        wheels_[RIGHT] = gazebo_ros_ -> getJoint ( parent, "RightRearWheel", "rightrearwheel" );
        wheels_[LEFT ] -> SetParam( "fmax", 0, wheeltorque );
        wheels_[RIGHT] -> SetParam( "fmax", 0, wheeltorque );

        // initialize update rate stuff
        if ( this->update_rate_ > 0.0 ) this->update_period_ = 1.0 / this->update_rate_;
        else this->update_period_ = 0.0;
        last_update_time_ = parent->GetWorld()->GetSimTime();

        // rostopic subscriber
        ros::SubscribeOptions so_twist = ros::SubscribeOptions::create<geometry_msgs::Twist>(command_topic_, 1, boost::bind(&Ackermannplugin_IWS::cmdVelCallback, this, _1), ros::VoidPtr(), &queue_);
        ros::SubscribeOptions so_iws = ros::SubscribeOptions::create<tuw_nav_msgs::JointsIWS>(command_topic_, 1, boost::bind(&Ackermannplugin_IWS::cmdIWSCallback, this, _1), ros::VoidPtr(), &queue_); //TODO:Queue
        //command_subscriber_ = gazebo_ros_->node()->subscribe(so_twist);
        command_subscriber_iws_ = gazebo_ros_->node()->subscribe(so_iws);

        // start custom queue
        this->callback_queue_thread_ = boost::thread ( boost::bind ( &Ackermannplugin_IWS::QueueThread, this ) );

        // listen to the update event (broadcast every simulation iteration)
        this->update_connection_  = event::Events::ConnectWorldUpdateBegin ( boost::bind ( &Ackermannplugin_IWS::UpdateChild, this ) );
        ROS_INFO("Ackermann Drive IWS Plugin loaded!");
    }

    void Ackermannplugin_IWS::cmdVelCallback ( const geometry_msgs::Twist::ConstPtr& cmd_msg ) {
        boost::mutex::scoped_lock scoped_lock ( lock );
        target_velocity = cmd_msg->linear.x;
        angle_center = cmd_msg->angular.z;

        // anglar.x is abused to get cheap communication with the controller
        reset = false;
        if (cmd_msg->angular.x != 0) {
            reset = true;
            ROS_INFO("A Botton pressed");
        }

    }

    void Ackermannplugin_IWS::cmdIWSCallback ( const tuw_nav_msgs::JointsIWS::ConstPtr& cmd_msg ) {
        boost::mutex::scoped_lock scoped_lock ( IWS_message_lock );
        ROS_DEBUG("IWS Received");
        wheel_velocity = cmd_msg->revolute[0];
        steering_omega = cmd_msg->steering[0];

    }

    void Ackermannplugin_IWS::UpdateChild() {

        // wheelcontroll
        //double velocity = target_velocity / ( wheeldiameter / 2.0 );
        wheels_[LEFT ] -> SetParam( "vel", 0, wheel_velocity );
        wheels_[RIGHT] -> SetParam( "vel", 0, wheel_velocity );


        // steering control
        actual_angle_[2];
        actual_angle_[LEFT ] = steerings_[LEFT ] -> GetAngle(0).Radian();
        actual_angle_[RIGHT] = steerings_[RIGHT] -> GetAngle(0).Radian();

        //actual_angle_[LEFT ] = steerings_[LEFT ] -> GetVelocity(0);
        //actual_angle_[RIGHT] = steerings_[RIGHT] -> GetVelocity(0);

        // round to 1 d.p to avoid numerical errors
        actual_angle_[LEFT ] = roundTo(actual_angle_[LEFT ],1);
        actual_angle_[RIGHT ] = roundTo(actual_angle_[RIGHT ],1);

        if(gazebo_debug){
            ROS_INFO("left angle: %f",actual_angle_[LEFT ]);
            ROS_INFO("right angle: %f",actual_angle_[RIGHT ]);
            ROS_INFO("steering omega: %f",steering_omega);
        }

        double delta_omega_[2];

        // Corrects back to 0 angle if steering = 0
        // seems that left wheel has initial angle of 0.1
        delta_omega_[LEFT ] = steering_omega != 0.0 ? steering_omega : actual_angle_[LEFT ] <= -0.1 || actual_angle_[LEFT ] >= 0.1 ? -steeringVelocity* sin(actual_angle_[LEFT ])/wheelbase : 0.0;
        delta_omega_[RIGHT] =  steering_omega != 0.0  ? steering_omega : actual_angle_[RIGHT ] <= -0.1 || actual_angle_[RIGHT ] >= 0.1 ?  -steeringVelocity*sin(actual_angle_[RIGHT ])/wheelbase : 0.0;


        steerings_[LEFT ]->SetParam("vel", 0, delta_omega_[LEFT ]);
        steerings_[RIGHT]->SetParam("vel", 0, delta_omega_[RIGHT]);


        last_update_time_ = last_update_time_ + common::Time ( update_period_ );
    }

    void Ackermannplugin_IWS::QueueThread () {
        static const double timeout = 0.01;
        while ( gazebo_ros_->node()->ok() ) {
            queue_.callAvailable ( ros::WallDuration ( timeout ) );
        }
    }

    double Ackermannplugin_IWS::roundTo(double value, int decimal_places) {

        double factor = pow(10.0,(double)decimal_places);

        return ceil(value*factor)/factor;
    }




    GZ_REGISTER_MODEL_PLUGIN(Ackermannplugin_IWS)
}
