#include "ackermanndrive_plugin.h"

namespace gazebo {
	
	enum {
		RIGHT,
		LEFT,
	};
	
	Ackermannplugin::Ackermannplugin() {}
	Ackermannplugin::~Ackermannplugin() {}
	
	void Ackermannplugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
		parent = _parent;
		ROS_INFO("Ackermann Drive Plugin loading...");
		gazebo_ros_ = GazeboRosPtr ( new GazeboRos ( _parent, _sdf, "Ackeramnndrive" ) );
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
		gazebo_ros_ -> getParameter<double> ( steeringvelocity, 	"Steeringfactor",	10.0 );
		gazebo_ros_ -> getParameter<double> ( streeringtorque, 		"Streeringtorque",	10.0 );
		gazebo_ros_ -> getParameter<double> ( steeringangle, 		"Steeringangle",	0.0  );
		gazebo_ros_ -> getParameter<double> ( wheeltorque,			"Wheeltorque",	10.0 );
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
		transform_broadcaster_ = boost::shared_ptr<tf::TransformBroadcaster>(new tf::TransformBroadcaster());
		
		// rostopic publisher
		odometry_publisher_godview_ = gazebo_ros_->node()->advertise<nav_msgs::Odometry>(odometry_topic_godview_, 1);
		odometry_publisher_encoder_ = gazebo_ros_->node()->advertise<nav_msgs::Odometry>(odometry_topic_encoder_, 1);
				
		// rostopic subscriber
		ros::SubscribeOptions so = ros::SubscribeOptions::create<geometry_msgs::Twist>(command_topic_, 1, boost::bind(&Ackermannplugin::cmdVelCallback, this, _1), ros::VoidPtr(), &queue_);
		command_subscriber_ = gazebo_ros_->node()->subscribe(so);
		
		// start custom queue
		this->callback_queue_thread_ = boost::thread ( boost::bind ( &Ackermannplugin::QueueThread, this ) );
		
		// listen to the update event (broadcast every simulation iteration)
		this->update_connection_  = event::Events::ConnectWorldUpdateBegin ( boost::bind ( &Ackermannplugin::UpdateChild, this ) );
		ROS_INFO("Ackermann Drive Plugin loaded!");
	}
	
	void Ackermannplugin::cmdVelCallback ( const geometry_msgs::Twist::ConstPtr& cmd_msg ) {
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
	
	void Ackermannplugin::UpdateChild() {
		common::Time current_time = parent->GetWorld()->GetSimTime();
		double seconds_since_last_update = ( current_time - last_update_time_ ).Double();
		PublishOdometryGodview ( seconds_since_last_update );
		PublishOdometryEncoder ( seconds_since_last_update );
		
		// wheelcontroll
		velocity = target_velocity / ( wheeldiameter / 2.0 );
		wheels_[LEFT ] -> SetParam( "vel", 0, velocity );
		wheels_[RIGHT] -> SetParam( "vel", 0, velocity );
		
		// steeringcontoll
		actual_angle_[2];
		actual_angle_[LEFT ] = steerings_[LEFT ] -> GetAngle(0).Radian();
		actual_angle_[RIGHT] = steerings_[RIGHT] -> GetAngle(0).Radian();
		
		double small_curve_radius = ( wheelbase / tan( steeringangle ) ) + steeringwidth / 2;
		double alpha_center_MAX =  atan(wheelbase / small_curve_radius);
		double alpha_center_MIN = -atan(wheelbase / small_curve_radius);
		//small_curve_radius is the radius (of the center wheel) of the samllest curve that can be driven with the range set by the steering
		
		if (angle_center > alpha_center_MAX ) angle_center = alpha_center_MAX;
		if (angle_center < alpha_center_MIN ) angle_center = alpha_center_MIN;
		
		target_angle_[2];
		target_angle_[LEFT ] = 0;
		target_angle_[RIGHT] = 0;
		
		if (  angle_center != 0 ) {
			curve_radius = wheelbase / tan ( angle_center ); //curve radius for the imaginary centerd wheel
			target_angle_[LEFT ] = atan( wheelbase / ( curve_radius - steeringwidth / 2 ) );
			target_angle_[RIGHT] = atan( wheelbase / ( curve_radius + steeringwidth / 2 ) );
		}
		
		direction_[2]; 	//sets the directen of the steering movement
		direction_[LEFT ] =  0;
		direction_[RIGHT] =  0;
		if ( actual_angle_[LEFT ] <  target_angle_[LEFT ] ) direction_[LEFT ] =  1;
		if ( actual_angle_[LEFT ] >  target_angle_[LEFT ] ) direction_[LEFT ] = -1;
		if ( actual_angle_[RIGHT] <  target_angle_[RIGHT] ) direction_[RIGHT] =  1;
		if ( actual_angle_[RIGHT] >  target_angle_[RIGHT] ) direction_[RIGHT] = -1;
		
		factor_[2]; 	//sets the speed of the steering movement
		factor_[LEFT ] = sqrt( ( target_angle_[LEFT ] - actual_angle_[LEFT ] ) * ( target_angle_[LEFT ] - actual_angle_[LEFT ] ) );
		factor_[RIGHT] = sqrt( ( target_angle_[RIGHT] - actual_angle_[RIGHT] ) * ( target_angle_[RIGHT] - actual_angle_[RIGHT] ) );
		if (factor_[LEFT ] > 1) factor_[LEFT ] = 1;
		if (factor_[RIGHT] > 1) factor_[RIGHT] = 1;

		steerings_[LEFT ]->SetParam("vel", 0, direction_[LEFT ]*factor_[LEFT ]*steeringvelocity);
		steerings_[RIGHT]->SetParam("vel", 0, direction_[RIGHT]*factor_[RIGHT]*steeringvelocity);
		
		last_update_time_ = last_update_time_ + common::Time ( update_period_ );
	}
	
	void Ackermannplugin::QueueThread () {
			static const double timeout = 0.01;
			while ( gazebo_ros_->node()->ok() ) {
			queue_.callAvailable ( ros::WallDuration ( timeout ) );
		}
	}
	
	void Ackermannplugin::PublishOdometryGodview ( double step_time ) {
		ros::Time current_time = ros::Time::now();
		std::string odom_frame = gazebo_ros_->resolveTF ( odometry_frame_ );
		std::string base_footprint_frame = gazebo_ros_->resolveTF ( robot_base_frame_ );

		tf::Quaternion qt;
		tf::Vector3 vt;

		// getting data form gazebo world
		math::Pose pose = parent->GetWorldPose();
		
		vt = tf::Vector3 ( pose.pos.x, pose.pos.y, pose.pos.z );
		odometry_godview_.pose.pose.position.x = vt.x();
		odometry_godview_.pose.pose.position.y = vt.y();
		odometry_godview_.pose.pose.position.z = vt.z();

		qt = tf::Quaternion ( pose.rot.x, pose.rot.y, pose.rot.z, pose.rot.w );
		odometry_godview_.pose.pose.orientation.x = qt.x();
		odometry_godview_.pose.pose.orientation.y = qt.y();
		odometry_godview_.pose.pose.orientation.z = qt.z();
		odometry_godview_.pose.pose.orientation.w = qt.w();

		// get velocity in /odom frame
		math::Vector3 linear;
		linear = parent->GetWorldLinearVel();
		odometry_godview_.twist.twist.angular.z = parent->GetWorldAngularVel().z;

		// convert velocity to child_frame_id (aka base_footprint)
		float yaw = pose.rot.GetYaw();
		odometry_godview_.twist.twist.linear.x = cosf ( yaw ) * linear.x + sinf ( yaw ) * linear.y;
		odometry_godview_.twist.twist.linear.y = cosf ( yaw ) * linear.y - sinf ( yaw ) * linear.x;

		tf::Transform base_footprint_to_odom ( qt, vt );
		transform_broadcaster_->sendTransform (	tf::StampedTransform ( base_footprint_to_odom , current_time , odom_frame, base_footprint_frame ) );

		// covariance
		odometry_godview_.pose.covariance[ 0+0] = 0.0;
		odometry_godview_.pose.covariance[ 6+1] = 0.0;
		odometry_godview_.pose.covariance[12+2] = 0.0;
		odometry_godview_.pose.covariance[18+3] = 0.0;
		odometry_godview_.pose.covariance[24+4] = 0.0;
		odometry_godview_.pose.covariance[30+5] = 0.0;

		// set header
		odometry_godview_.header.stamp = current_time;
		odometry_godview_.header.frame_id = odom_frame;
		odometry_godview_.child_frame_id = base_footprint_frame;
		
		// publish godview odometry
		odometry_publisher_godview_.publish ( odometry_godview_ );
	}
	
	void Ackermannplugin::PublishOdometryEncoder ( double step_time ) {
		ros::Time current_time = ros::Time::now();
		std::string odom_frame = gazebo_ros_->resolveTF ( odometry_frame_ );
		std::string base_footprint_frame = gazebo_ros_->resolveTF ( robot_base_frame_ );
		
		common::Time actual_time = parent->GetWorld()->GetSimTime();
		double seconds_since_last_update = ( actual_time - last_odom_update_ ).Double();
		last_odom_update_ = actual_time;
		
		v = target_velocity;
		if ( angle_center == 0 ) w = 0;
		if ( angle_center != 0 ) w = v * tan(angle_center) / wheelbase ;
		
		double dt = seconds_since_last_update;
		double dx = v * cos(pose_encoder_.theta) * seconds_since_last_update;
		double dy = v * sin(pose_encoder_.theta) * seconds_since_last_update;
		double dtheta = w * seconds_since_last_update;
		
		double alpha_1 = 1.0;
		double alpha_2 = 1.0;
		double alpha_3 = 1.0;
		double alpha_4 = 1.0;
		
		// calculating the covariance matrix
		//G = cv::Matx<double, 3, 3> (1.0, 0, -abs(v)*dt*sin(pose_encoder_.theta), 0, 1.0,  abs(v)*dt*cos(pose_encoder_.theta), 0, 0,  1.0);
		if ( v >= 0 ) {
			G = cv::Matx<double, 3, 3> (1.0, 0, -(v*dt*sin(pose_encoder_.theta)), 0, 1.0,  (v*dt*cos(pose_encoder_.theta)), 0, 0,  1.0);
			V = cv::Matx<double, 3, 2> (dt*cos(pose_encoder_.theta), 0, dt*sin(pose_encoder_.theta), 0, dt*tan(angle_center)/wheelbase, (v)*dt / (wheelbase *cos(angle_center)*cos(angle_center)));
		} else {
			G = cv::Matx<double, 3, 3> (1.0, 0, -(v*dt*sin(pose_encoder_.theta+M_PI)), 0, 1.0,  (v*dt*cos(pose_encoder_.theta+M_PI)), 0, 0,  1.0);
			V = cv::Matx<double, 3, 2> (dt*cos(pose_encoder_.theta+M_PI), 0, dt*sin(pose_encoder_.theta+M_PI), 0, dt*tan(angle_center)/wheelbase, (v)*dt / (wheelbase *cos(angle_center)*cos(angle_center))); 
		}
		M = cv::Matx<double, 2, 2> (alpha_1*v*v + alpha_2*angle_center*angle_center, 0, 0, alpha_3*v*v + alpha_4*angle_center*angle_center);
		
		// inital matrix needs to be set
		if ( P(0,0)==0 || P(1,1)==0 || P(2,2)==0 ) ResetMatrix();
		// anglar.x is abused to get cheap communication with the controller
		if ( reset == true ) ResetMatrix();
		
		P = (G*P*G.t()+V*M*V.t());
		
		pose_encoder_.x = pose_encoder_.x + dx;
		pose_encoder_.y = pose_encoder_.y + dy;
		pose_encoder_.theta = pose_encoder_.theta + dtheta;
		
		tf::Quaternion qt;
		tf::Vector3 vt;

		qt.setRPY( 0,0,pose_encoder_.theta);
		vt = tf::Vector3 ( pose_encoder_.x, pose_encoder_.y, 0 );
		
		if (reset == false) {
			odometry_encoder_.pose.pose.position.x = vt.x();
			odometry_encoder_.pose.pose.position.y = vt.y();
			odometry_encoder_.pose.pose.position.z = vt.z();

			odometry_encoder_.pose.pose.orientation.x = qt.x();
			odometry_encoder_.pose.pose.orientation.y = qt.y();
			odometry_encoder_.pose.pose.orientation.z = qt.z();
			odometry_encoder_.pose.pose.orientation.w = qt.w();

			odometry_encoder_.twist.twist.angular.z = w;
			odometry_encoder_.twist.twist.linear.x = dx/seconds_since_last_update;
			odometry_encoder_.twist.twist.linear.y = dy/seconds_since_last_update;
		}
		// getting data form encoder integration
		qt = tf::Quaternion (	odometry_encoder_.pose.pose.orientation.x,
								odometry_encoder_.pose.pose.orientation.y,
								odometry_encoder_.pose.pose.orientation.z,
								odometry_encoder_.pose.pose.orientation.w );
		vt = tf::Vector3 (	odometry_encoder_.pose.pose.position.x,
							odometry_encoder_.pose.pose.position.y,
							odometry_encoder_.pose.pose.position.z );

		tf::Transform base_footprint_to_odom ( qt, vt );
		transform_broadcaster_->sendTransform (	tf::StampedTransform ( base_footprint_to_odom , current_time , odom_frame, base_footprint_frame ) );

		// covariance
		odometry_encoder_.pose.covariance[ 0+0] = P(0,0);
		odometry_encoder_.pose.covariance[ 0+1] = P(0,1);
		odometry_encoder_.pose.covariance[ 6+0] = P(1,0);
		odometry_encoder_.pose.covariance[ 6+1] = P(1,1);
		odometry_encoder_.pose.covariance[12+2] = 0.1;
		odometry_encoder_.pose.covariance[18+3] = 0.01;
		odometry_encoder_.pose.covariance[24+4] = 0.01;
		odometry_encoder_.pose.covariance[30+5] = 0.01;

		// set header	
		odometry_encoder_.header.stamp = current_time;
		odometry_encoder_.header.frame_id = odom_frame;
		odometry_encoder_.child_frame_id = base_footprint_frame;
		
		// publish encoder odometry
		odometry_publisher_encoder_.publish ( odometry_encoder_ );
	}
	void Ackermannplugin::ResetMatrix () {
		P(0,0) = 0.1;
		P(1,1) = 0.1;
		P(2,2) = 0.1;
	}
	GZ_REGISTER_MODEL_PLUGIN(Ackermannplugin)
}
