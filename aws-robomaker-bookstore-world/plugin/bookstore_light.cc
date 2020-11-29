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
#include "aws_robomaker_bookstore_world/bookstore_light_msgs.h"

namespace gazebo
{
    class Bookstore_LightPlugin : public WorldPlugin
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
        Bookstore_LightPlugin() {}

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
                ros::SubscribeOptions::create<aws_robomaker_bookstore_world::bookstore_light_msgs>(
                    "/bookstore_lights",
                    1,
                    boost::bind(&Bookstore_LightPlugin::OnRosMsg, this, _1),
                    ros::VoidPtr(), &this->rosQueue);
            this->rosSub = this->rosNode->subscribe(so);

            // Spin up the queue helper thread.
            this->rosQueueThread =
                std::thread(std::bind(&Bookstore_LightPlugin::QueueThread, this));
        }

    public:
        void OnRosMsg(const aws_robomaker_bookstore_world::bookstore_light_msgsConstPtr &_msg)
        {
            msgs::Light lightMsg;
            std::vector<int> light_num = _msg->light_number;
            for (int i = 0; i < light_num.size(); i++)
            {

                std::string light_name = "RetailShop_CeilingLight_00" + std::to_string(light_num[i]);
                lightMsg.set_name(light_name);
                lightMsg.set_range(_msg->range);
                this->lightPub->Publish(lightMsg);
            }
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
    GZ_REGISTER_WORLD_PLUGIN(Bookstore_LightPlugin);
} // namespace gazebo
