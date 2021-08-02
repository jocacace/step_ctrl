/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#include <ros/ros.h>
#include <sdf/sdf.hh>
#include <boost/bind.hpp>
#include "gazebo/gazebo.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"
#include "boost/thread.hpp"
#include <shared_control_msgs/GetTorques.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <shared_control_msgs/requestCalc.h>

using namespace std;

/// \example examples/plugins/world_edit.cc
/// This example creates a WorldPlugin, initializes the Transport system by
/// creating a new Node, and publishes messages to alter gravity.
namespace gazebo
{
  class WorldEdit : public WorldPlugin
  {
    public: void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
    {

		// Create a new transport node
		this->node.reset(new transport::Node());
		_node_handle = new ros::NodeHandle();
		// Initialize the node with the world name
		this->node->Init(_parent->Name());

		std::cout << _parent->Name() << std::endl;
		this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&WorldEdit::OnUpdate, this));
		// Create a publisher
		this->pub = this->node->Advertise<msgs::WorldControl>("~/world_control");
		_node_handle = new ros::NodeHandle();
		_ready = false;


		joint_tor_sub = _node_handle->subscribe("/torque_joints", 1, &WorldEdit::torque_cb, this);

  		//_client = _node_handle->serviceClient<shared_control_msgs::GetTorques>("get_torques");	
  		_client = _node_handle->serviceClient<shared_control_msgs::requestCalc>("start_calc");	
		boost::thread mainLoop_t( &WorldEdit::mainLoop, this);
		
    }

    public: void torque_cb( std_msgs::Float64MultiArray data ) {
		
		tau1_msg.data = data.data[0];
		tau2_msg.data = data.data[1];
		tau3_msg.data = data.data[2];
		tau4_msg.data = data.data[3];
		tau5_msg.data = data.data[4];
		tau6_msg.data = data.data[5];
		tau7_msg.data = data.data[6];
		_torque_ready = true;

    }

	public: void mainLoop() {

		msgs::WorldControl msg;

		ros::Rate r(1);
		ros::Publisher joint1_effort_pub = _node_handle->advertise<std_msgs::Float64>("/lbr_iiwa/lbr_iiwa_joint_1_effort_controller/command", 1);
		ros::Publisher joint2_effort_pub = _node_handle->advertise<std_msgs::Float64>("/lbr_iiwa/lbr_iiwa_joint_2_effort_controller/command", 1);
		ros::Publisher joint3_effort_pub = _node_handle->advertise<std_msgs::Float64>("/lbr_iiwa/lbr_iiwa_joint_3_effort_controller/command", 1);
		ros::Publisher joint4_effort_pub = _node_handle->advertise<std_msgs::Float64>("/lbr_iiwa/lbr_iiwa_joint_4_effort_controller/command", 1);
		ros::Publisher joint5_effort_pub = _node_handle->advertise<std_msgs::Float64>("/lbr_iiwa/lbr_iiwa_joint_5_effort_controller/command", 1);
		ros::Publisher joint6_effort_pub = _node_handle->advertise<std_msgs::Float64>("/lbr_iiwa/lbr_iiwa_joint_6_effort_controller/command", 1);
		ros::Publisher joint7_effort_pub = _node_handle->advertise<std_msgs::Float64>("/lbr_iiwa/lbr_iiwa_joint_7_effort_controller/command", 1);

		shared_control_msgs::GetTorques gtorque;

		cout << "Waiting the system starts!" << endl;
		while( !_ready ) {
			usleep(0.05*1e6);
		} 
		cout << "Ready!" << endl;
	
		//ros::Publisher synch = _node_handle->advertise<std_msgs::Bool>("/mpc/sync", 10);
		std_msgs::Bool data;
		data.data = true;



		shared_control_msgs::requestCalc rc;
		while( !_client.call(rc) ) {
			sleep(1);
		}

		cout << "Starting!" << endl;

		while ( ros::ok() ) {
			
			_client.call(rc);
       	
			while (!_torque_ready ) {
				ros::spinOnce();
				usleep ( 0.1*1e6);
				cout << "_torque_ready: " << _torque_ready << endl;
			}
			_torque_ready = false;
			
			joint1_effort_pub.publish(tau1_msg);
			joint2_effort_pub.publish(tau2_msg);
			joint3_effort_pub.publish(tau3_msg);
			joint4_effort_pub.publish(tau4_msg);
			joint5_effort_pub.publish(tau5_msg);
			joint6_effort_pub.publish(tau6_msg);
			joint7_effort_pub.publish(tau7_msg);
			
			//apply torque
			worldControlMsg.set_pause(0);
			this->pub->Publish(worldControlMsg);
			//sleep(1);
	
			/*
			// Throttle Publication
			//worldControlMsg.set_pause(1);
			//this->pub->Publish(worldControlMsg);
		    worldControlMsg.set_pause(0);
			this->pub->Publish(worldControlMsg);
        	
			gazebo::common::Time::MSleep(1000);
			worldControlMsg.set_pause(1);
			this->pub->Publish(worldControlMsg);
        	
			std::cout << "Publishing OnUpdate." << std::endl;
			*/



		}
	}

    // Called by the world update start event.
    public: void OnUpdate() 
    {
		_ready = true;

		worldControlMsg.set_pause(1); //PAUSE
		this->pub->Publish(worldControlMsg);
        //gazebo::common::Time::MSleep(1000);

        /*msgs::WorldControl msg;
        msg.set_step(1);
        this->pub->Publish(msg);*/
    }

    // Pointer to the world_controller
    private: transport::NodePtr node;
    private: transport::PublisherPtr pub;
	private: ros::NodeHandle* _node_handle;
    private: bool _ready;
	// Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
	msgs::WorldControl worldControlMsg;
	ros::ServiceClient _client;
	private: ros::Subscriber joint_tor_sub;
	private: bool _torque_ready;
	std_msgs::Float64 tau1_msg, tau2_msg, tau3_msg, tau4_msg, tau5_msg, tau6_msg, tau7_msg;

  };

  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(WorldEdit)
}
