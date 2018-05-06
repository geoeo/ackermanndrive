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
        gazebo_ros_ -> getParameter<std::string> ( joint_iws_topic, "IWSTopic", "joint_iws_pub" );
        gazebo_ros_ -> getParameter<std::string> ( odometry_topic_encoder_, "OdometryTopicEncoder", "odometry_encoder" );
        gazebo_ros_ -> getParameter<std::string> ( odometry_topic_godview_, "OdometryTopicGodview", "odometry_godview" );
        gazebo_ros_ -> getParameter<std::string> ( odometry_frame_, 	"odometryFrame", "odom" );
        gazebo_ros_ -> getParameter<std::string> ( robot_base_frame_, "robotBaseFrame", "base_link" );
        gazebo_ros_ -> getParameter<double> ( wheelbase, 		"Wheelbase",		0.0 );
        gazebo_ros_ -> getParameter<double> ( track, 			"Track",		0.0 );
        gazebo_ros_ -> getParameter<double> ( steeringwidth, 		"SteeringWidth",	0.0 );
        gazebo_ros_ -> getParameter<double> ( wheel_diameter, 		"Wheeldiameter",	0.0 );
        gazebo_ros_ -> getParameter<double> ( steering_velocity, 	"SteeringVelocity",	 10.0 );
        gazebo_ros_ -> getParameter<double> ( streeringtorque, 		"Streeringtorque",	10.0 );
        gazebo_ros_ -> getParameter<double> ( max_steering_angle, 		"MaxSteeringAngle",	0.0  );
        gazebo_ros_ -> getParameter<double> ( max_revolute_velocity, 		"MaxVelocityRevolute",	1.0  );
        gazebo_ros_ -> getParameter<double> ( max_steering_omega, 		"MaxSteeringOmega",	10.0  );
        gazebo_ros_ -> getParameter<double> ( steering_acceleration, 		"SteeringAcceleration",	1.0  );
        gazebo_ros_ -> getParameter<double> ( linear_acceleration, 		"LinearAcceleration",	1.0  );
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
        ros::SubscribeOptions so_iws = ros::SubscribeOptions::create<tuw_nav_msgs::JointsIWS>(command_topic_, 1, boost::bind(&Ackermannplugin_IWS::cmdIWSCallback, this, _1), ros::VoidPtr(), &queue_);
        //command_subscriber_ = gazebo_ros_->node()->subscribe(so_twist);
        command_subscriber_iws_ = gazebo_ros_->node()->subscribe(so_iws);

        // rostopic publisher
        joint_iws_publisher_ = gazebo_ros_->node()->advertise<tuw_nav_msgs::JointsIWS>(joint_iws_topic,1);

        // start custom queue
        this->callback_queue_thread_ = boost::thread ( boost::bind ( &Ackermannplugin_IWS::QueueThread, this ) );

        // listen to the update event (broadcast every simulation iteration)
        this->update_connection_  = event::Events::ConnectWorldUpdateBegin ( boost::bind ( &Ackermannplugin_IWS::UpdateChild, this ) );


        steering_angle_ = 0.0;
        steering_omega_ = 0.0;
        wheel_velocity_ = 0.0;

        cmd_iws_publish_.header.seq = 0;
        cmd_iws_publish_.header.stamp = ros::Time::now();
        cmd_iws_publish_.type_steering = "measures_steering";
        cmd_iws_publish_.type_revolute = "measures_velocity";
        cmd_iws_publish_.revolute.resize (2);
        cmd_iws_publish_.steering.resize (2);
        cmd_iws_publish_.steering[0] = steering_angle_;
        cmd_iws_publish_.revolute[0] = std::nan("1");
        cmd_iws_publish_.steering[1] = std::nan("1");
        cmd_iws_publish_.revolute[1] = wheel_velocity_;
        joint_iws_publisher_.publish (cmd_iws_publish_);

        ROS_INFO("Ackermann Drive IWS Plugin loaded!");
        ROS_INFO("Wheel base: %f", wheelbase);
        ROS_INFO("Steering Velocity %f", steering_velocity);
    }

    void Ackermannplugin_IWS::QueueThread () {
        static const double timeout = 0.01;
        while ( gazebo_ros_->node()->ok() ) {
            queue_.callAvailable ( ros::WallDuration ( timeout ) );
        }
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

        wheel_velocity_ = cmd_msg->revolute[1];
        steering_angle_ = cmd_msg->steering[0];
        //ROS_INFO("IWS RECEIVED");

    }

    void Ackermannplugin_IWS::UpdateChild() {
        //TODO Investigate Locking due to potential Callback Race condtions

        wheel_velocity_ /= wheel_diameter / 2.0;

        double actual_wheel_velocity = RoundTo(wheels_[LEFT]->GetVelocity(0),2);
        wheel_velocity_ = RoundTo(wheel_velocity_,2);

        if(gazebo_debug){
            ROS_INFO("target wheel velocity: %f",wheel_velocity_);
            ROS_INFO("actual wheel velocity: %f",actual_wheel_velocity);
        }

        // strange initialization bug, where velocity is not 0
        if(fabs(wheel_velocity_) > 0.1)
            wheel_velocity_ = CalculateVelocityWithAcceleration(actual_wheel_velocity,wheel_velocity_,linear_acceleration,max_revolute_velocity);

        wheels_[LEFT ] -> SetParam( "vel", 0, wheel_velocity_ );
        wheels_[RIGHT] -> SetParam( "vel", 0, wheel_velocity_ );

        // steering control
        actual_angle_[LEFT ] = steerings_[LEFT ] -> GetAngle(0).Radian();
        actual_angle_[RIGHT] = steerings_[RIGHT] -> GetAngle(0).Radian();

        actual_velocity[LEFT ] = steerings_[LEFT ] -> GetVelocity(0);
        actual_velocity[RIGHT] = steerings_[RIGHT] -> GetVelocity(0);

        // round to 1 d.p to avoid numerical errors
        actual_angle_[LEFT ] = RoundTo(actual_angle_[LEFT ],2);
        actual_angle_[RIGHT ] = RoundTo(actual_angle_[RIGHT ],2);

        actual_velocity[LEFT ] = RoundTo(actual_velocity[LEFT ],1);
        actual_velocity[RIGHT ] = RoundTo(actual_velocity[RIGHT ],1);

        // limit angle based on config settings
        if(steering_angle_ > max_steering_angle) steering_angle_ = max_steering_angle;
        else if(steering_angle_ < -max_steering_angle) steering_angle_ = -max_steering_angle;

        steering_omega_ = OmegaFromTricicleModel(steering_angle_);

        if(steering_omega_ > max_steering_omega) steering_omega_ = max_steering_omega;
        else if (steering_omega_ < -max_steering_omega) steering_omega_ = -max_steering_omega;

        steering_omega_ = RoundTo(steering_omega_,1);

        double steering_back_left = steering_velocity*sin(actual_angle_[LEFT ]);
        double steering_back_right = steering_velocity*sin(actual_angle_[RIGHT ]);

        if(gazebo_debug){
            ROS_INFO("left angle: %f",actual_angle_[LEFT ]);
            ROS_INFO("right angle: %f",actual_angle_[RIGHT ]);
            ROS_INFO("steering omega: %f",steering_omega_);
            ROS_INFO("steering angle: %f",steering_angle_);
        }

        double omega[2];

        // Corrects back to 0 angle if steering = 0

        omega[LEFT] = steering_angle_ != 0.0 ? steering_omega_ : !IsBetween(actual_angle_[LEFT ],-0.01,0.01) ? -steering_back_left : 0.0;
        omega[RIGHT] = steering_angle_ != 0.0 ? steering_omega_ :  !IsBetween(actual_angle_[RIGHT ],-0.01,0.01) ? -steering_back_right : 0.0;

        // seems to be more numerically instable for small angles
        //omega[LEFT] = steering_angle_ != 0.0 ? CalculateVelocityWithAcceleration(actual_velocity[LEFT],steering_omega_,steering_acceleration,steering_velocity) : !IsBetween(actual_angle_[LEFT ],-0.1,0.01) ? CalculateVelocityWithAcceleration(actual_velocity[LEFT],-steering_back_left,steering_acceleration,steering_velocity) : 0.0;
        //omega[RIGHT] = steering_angle_ != 0.0 ? CalculateVelocityWithAcceleration(actual_velocity[RIGHT],steering_omega_,steering_acceleration,steering_velocity) :  !IsBetween(actual_angle_[RIGHT ],-0.01,0.01) ?  CalculateVelocityWithAcceleration(actual_velocity[RIGHT],-steering_back_right,steering_acceleration,steering_velocity) : 0.0;


        if(gazebo_debug){
            ROS_INFO("delta omega left: %f" ,omega[LEFT ]);
            ROS_INFO("delta omega right: %f", omega[RIGHT ]);
        }


        steerings_[LEFT ]->SetParam("vel", 0, omega[LEFT ]);
        steerings_[RIGHT]->SetParam("vel", 0, omega[RIGHT]);

        last_update_time_ = last_update_time_ + common::Time ( update_period_ );

        PublishJointIWS();
    }

    double Ackermannplugin_IWS::OmegaFromTricicleModel(double steering_angle) {
        return steering_velocity*sin(steering_angle)/wheelbase;
    }

    double Ackermannplugin_IWS::CalculateVelocityWithAcceleration(double current_velocity, double target_velocity, double acceleration, double limit) {
        double eta = 0.1;
        double diff = current_velocity - target_velocity;
        double computed_velocity = current_velocity;

        if(fabs(diff) > eta){
            if(diff > 0.0)
                computed_velocity -= acceleration;
            else if (diff < 0.0)
                computed_velocity += acceleration;
        }

        computed_velocity = RoundTo(computed_velocity,2);

        if(computed_velocity > limit) computed_velocity = limit;
        else if(computed_velocity < -limit) computed_velocity = -limit;

        return computed_velocity;
    }

    double Ackermannplugin_IWS::RoundTo(double value, int decimal_places) {

        double factor = pow(10.0,(double)decimal_places);
        return ceil(value*factor)/factor;
    }

    bool Ackermannplugin_IWS::IsBetween(double value, double min, double max){
        return value < max && value > min;
    }

    void Ackermannplugin_IWS::PublishJointIWS() {
        cmd_iws_publish_.header.seq++;
        cmd_iws_publish_.header.stamp = ros::Time::now();
        //cmd_iws_publish_.header.stamp = ros::Time(parent->GetWorld()->GetSimTime().Double());

        //cmd_iws_publish_.steering[0] = steering_angle_;
        //cmd_iws_publish_.revolute[1] = wheel_velocity_;

        // TODO: estimate based on turning directions as inner wheel changes
        double steering_inner =  RoundTo(steerings_[LEFT ]->GetAngle(0).Radian(),2);
        double cot_steering_inner = cos(steering_inner)/sin(steering_inner);
        double arc_cot_inner = (steeringwidth / (2.0*wheelbase)) + cot_steering_inner;
        double steering_tricicle = atan(1.0/arc_cot_inner);

        //cmd_iws_publish_.steering[0] = RoundTo(steerings_[LEFT ]->GetAngle(0).Radian(),2);
        cmd_iws_publish_.steering[0] = RoundTo(steering_inner,2);
        //cmd_iws_publish_.steering[0] = RoundTo(steering_angle_,2);
        cmd_iws_publish_.revolute[1] = RoundTo(wheels_[LEFT ]->GetVelocity(0),1);

        //ROS_INFO("Publishing steering: %f", cmd_iws_publish_.steering[0]);
        //ROS_INFO("Publishing rev: %f", cmd_iws_publish_.revolute[1]);

        joint_iws_publisher_.publish (cmd_iws_publish_);
    }




    GZ_REGISTER_MODEL_PLUGIN(Ackermannplugin_IWS)
}
