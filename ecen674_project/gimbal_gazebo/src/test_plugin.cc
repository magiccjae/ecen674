#ifndef _VELODYNE_PLUGIN_HH_
#define _VELODYNE_PLUGIN_HH_

#include <boost/bind.hpp>
#include <iostream>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh> 		// added
#include <gazebo/common/common.hh>
#include <gazebo/math/gzmath.hh>
#include <ctime>
#include <stdio.h>
#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Twist.h"

#define PI 3.141592


using namespace std;
namespace gazebo
{


  /// \brief A plugin to control a Velodyne sensor.
	class VelodynePlugin : public ModelPlugin
	{
    /// \brief Constructor
		public: VelodynePlugin() {}
            // void commandCallback(const geometry_msgs::Vector3::ConstPtr& msg){

            //   cout << "desired azimuth: " << msg->x << endl;
            //   cout << "desired elevation: " << msg->y << endl;
            //   angle_desired = msg->x;
            //   ball_angle_desired = msg->y;

            // }

		void commandCallback(const geometry_msgs::Twist::ConstPtr& msg){

              // cout << "desired azimuth: " << msg->linear.x << endl;
              // cout << "desired elevation: " << msg->angular.z << endl;
			if(msg->linear.x==0.5){
				direction = 1;
			}
			else if(msg->linear.x==-0.5){
				direction = 2;
			}

			if(msg->angular.z==0.5){
				direction = 3;
			}
			else if(msg->angular.z==-0.5){
				direction = 4;
			}

			if(msg->linear.x==0 && msg->angular.z==0){
				direction = 0;
			}

		}

            /// \brief The load function is called by Gazebo when the plugin is
            /// inserted into simulation
            /// \param[in] _model A pointer to the model that this plugin is
            /// attached to.
            /// \param[in] _sdf A pointer to the plugin's SDF element.
		virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
		{
              // Make sure the ROS node for Gazebo has already been initalized
			if (!ros::isInitialized())
			{
				ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
					<< "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
				return;
			}

			ROS_INFO("ROS connected!");
			nh = ros::NodeHandle();
			command_sub = nh.subscribe("cmd_vel", 1, &VelodynePlugin::commandCallback, this);
              // Just output a message for now
			std::cerr << "\nThe plugin is attach to model[" <<
			_model->GetName() << "]\n";
              // Listen to the update event. This event is broadcast every
              // simulation iteration.
			this->updateConnection = event::Events::ConnectWorldUpdateBegin(
				boost::bind(&VelodynePlugin::OnUpdate, this, _1));

          	  // Store the model pointer for convenience.
			this->model = _model;

          	  // Get the first joint. We are making an assumption about the model
          	  // having one joint that is the rotational joint.
			this->joint = _model->GetJoints()[0];
			this->ball_joint = _model->GetJoints()[1];
          		// this->ball_joint->SetForce(0,0.1);
          	  // cout <<	"Initial angle: " << this->joint->GetAngle(0).Degree() << endl;
			math::Angle angle(0);
			math::Angle ball_angle(0);
			this->joint->SetAngle(0,angle);
			this->ball_joint->SetAngle(0,ball_angle);
        	    // cout <<	this->joint->GetAngle(0).Degree() << endl;
              // Azimuth Angle
			angle_desired = 0;
			kp = 0.001;
			kd = 0.0001;
              // Elevation Angle
			ball_angle_desired = 90;
			ball_kp = 0.001;
			ball_kd = 0.0001;
              // Initialize Error
			previous_time = clock()/(double) CLOCKS_PER_SEC;
			previous_error = 0;

			direction = -1;
			update_rate = 0.5;
		}
            void OnUpdate(const common::UpdateInfo & /*_info*/)
		{
              // Update time
			current_time = clock()/(double) CLOCKS_PER_SEC;
			float time_step = current_time - previous_time;

              // Azimuth Controller
			float angle_current = this->joint->GetAngle(0).Degree();
			if(direction == 3){
				angle_desired = angle_current + update_rate;
				if(angle_desired > 360){
					angle_desired = 360;
				}
			}
			else if(direction == 4){
				angle_desired = angle_current - update_rate;
				if(angle_desired < -360){
					angle_desired = -360;
				}
			}
			float error = angle_desired - angle_current;
			float error_derivative = (error-previous_error)/time_step;
			float force_desired = kp*error + kd*error_derivative;
			joint->SetForce(0,force_desired);      
              // cout << "Azimuth angle: " << angle_current << endl;

              // Elevation Controller
			float ball_angle_current = this->ball_joint->GetAngle(0).Degree();
			if(direction == 1){
				ball_angle_desired = ball_angle_current + update_rate;
				if(ball_angle_desired > 180){
					ball_angle_desired = 180;
				}
			}
			else if(direction == 2){
				ball_angle_desired = ball_angle_current - update_rate;
				if(ball_angle_desired < 0){
					ball_angle_desired = 0;
				}
			}
			float ball_error = ball_angle_desired - ball_angle_current;
			float ball_error_derivative = (ball_error-ball_previous_error)/time_step;
			float ball_force_desired = ball_kp*ball_error + ball_kd*ball_error_derivative;
			ball_joint->SetForce(0,ball_force_desired);      
              // cout << "Elevation angle: " <<  ball_angle_current << endl;

              // Update error and time
			previous_error = error;
			ball_previous_error = ball_error;
			previous_time = current_time;

        //	cout << "I am updating" << endl;
        //	joint->SetVelocity(0,1000);
        //	joint->SetForce(0,0.0001);
        //	cout << joint->GetVelocity(0) << endl;
		}
  // Pointer to the model
	private:
		physics::ModelPtr model;
            // brief Pointer to the joint.
		physics::JointPtr joint;
		physics::JointPtr ball_joint;
            // Pointer to the update event connection
		event::ConnectionPtr updateConnection;
            // Azimuth Angle
		float kp;
		float kd;
		float previous_error;
            // Elevation Angle
		float ball_kp;
		float ball_kd;
		float ball_previous_error;
            // Time
		float previous_time;
		float current_time;

            // ROS variables
		ros::NodeHandle nh;
		ros::Subscriber command_sub;

		float angle_desired;
		float ball_angle_desired;
		int direction; 		// 0: no direction, 1: up, 2: down, 3: CW, 4: CCW
		float update_rate;
	};

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
	GZ_REGISTER_MODEL_PLUGIN(VelodynePlugin)
}
#endif
