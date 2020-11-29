/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

/// \brief This file contains a gazebo plugin for the ContainPlugin tutorial.
/// Doxygen comments and PIMPL are omitted to reduce the amount of text.

#include <ignition/msgs.hh>
#include <ignition/transport/Node.hh>

#include "gazebo/common/Plugin.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/transport/Node.hh"
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
namespace gazebo
{
    class TurnOnLightPlugin : public WorldPlugin
    {
    private:
        std::unique_ptr<ros::NodeHandle> rosNode;

        /// \brief A ROS subscriber
    private:
        ros::Subscriber rosSub;

        /// \brief A ROS callbackqueue that helps process messages
    private:
        ros::CallbackQueue rosQueue;

        /// \brief A thread the keeps running the rosQueue
    private:
        std::thread rosQueueThread;

    public:
        TurnOnLightPlugin() {}

    public:
        void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) override
        {
            gzmsg << "Loading Example plugin\n";
            // Transport initialization
            this->gzNode = transport::NodePtr(new transport::Node());
            this->gzNode->Init();


            // Make a publisher for the light topic
            this->lightPub = this->gzNode->Advertise<msgs::Light>("~/light/modify");
            // Initialize ros, if it has not already bee initialized.
            if (!ros::isInitialized())
            {
                int argc = 0;
                char **argv = NULL;
                ros::init(argc, argv, "gazebo_client",
                          ros::init_options::NoSigintHandler);
            }

            // Create our ROS node. This acts in a similar manner to
            // the Gazebo node
            this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

            // Create a named topic, and subscribe to it.
            ros::SubscribeOptions so =
                ros::SubscribeOptions::create<std_msgs::Float32>(
                    "/boxer_light_toggle",
                    1,
                    boost::bind(&TurnOnLightPlugin::OnRosMsg, this, _1),
                    ros::VoidPtr(), &this->rosQueue);
            this->rosSub = this->rosNode->subscribe(so);

            // Spin up the queue helper thread.
            this->rosQueueThread =
                std::thread(std::bind(&TurnOnLightPlugin::QueueThread, this));
        }
    public:
        void OnRosMsg(const std_msgs::Float32ConstPtr &_msg)
        {
            msgs::Light lightMsg;
            lightMsg.set_name("/::base_link::light_source");
            lightMsg.set_range(_msg->data);

            this->lightPub->Publish(lightMsg);
        }

        /// \brief ROS helper function that processes messages
    private:
        void QueueThread()
        {
            static const double timeout = 0.01;
            while (this->rosNode->ok())
            {
                this->rosQueue.callAvailable(ros::WallDuration(timeout));
            }
        }

    private:
        ignition::transport::Node node;

    private:
        transport::NodePtr gzNode;

    private:
        transport::PublisherPtr lightPub;
    };
    GZ_REGISTER_WORLD_PLUGIN(TurnOnLightPlugin);
} // namespace gazebo
