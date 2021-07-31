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

  		_client = _node_handle->serviceClient<shared_control_msgs::GetTorques>("get_torques");	
		boost::thread mainLoop_t( &WorldEdit::mainLoop, this);
		
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

		while ( ros::ok() ) {


			if( _client.call( gtorque ) ) {
				cout << "gtorque: " << gtorque.response.tj0 << " "  << gtorque.response.tj1 << " "  << gtorque.response.tj2 << " "  << gtorque.response.tj3 << " " 
			 	<< gtorque.response.tj4 << " "   << gtorque.response.tj5 << endl;  
			}
			else {
				cout << "failed to call the service" << endl;
			}

			worldControlMsg.set_pause(0);
			this->pub->Publish(worldControlMsg);
        	
			sleep(1);
			
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
  };

  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(WorldEdit)
}
