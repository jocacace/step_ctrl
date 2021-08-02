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

  		_client = _node_handle->serviceClient<shared_control_msgs::GetTorques>("get_torques");	
		boost::thread mainLoop_t( &WorldEdit::mainLoop, this);
		
    }

    public: void torque_cb( std_msgs::Float64MultiArray data ) {
		
		_torque_ready = true;
		cout << " torque ready " << endl;
    }

	public: void mainLoop() {

		msgs::WorldControl msg;

		ros::Rate r(1);
		shared_control_msgs::GetTorques gtorque;

		cout << "Waiting the system starts!" << endl;
		while( !_ready ) {
			usleep(0.05*1e6);
		} 
		cout << "Ready!" << endl;

		ros::Publisher synch = _node_handle->advertise<std_msgs::Bool>("/mpc/sync", 10);
		std_msgs::Bool data;
		data.data = true;

		while ( ros::ok() ) {
			
			cout << "before publishing" << endl;
			for(int i=0; i<100; i++ ) synch.publish( data );
			ros::spinOnce();

			worldControlMsg.set_pause(0); //PAUSE
			this->pub->Publish(worldControlMsg);
        	
			while (!_torque_ready ) usleep ( 0.1*1e6);
			_torque_ready = false;

			//apply torque
			
			worldControlMsg.set_pause(1);
			this->pub->Publish(worldControlMsg);
			sleep(1);
	
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

  };

  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(WorldEdit)
}
