/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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
/*
 * Author: Jennifer Buehler
 * Date: December 2016
 */
#ifndef COLLISION_BENCHMARK_GAZEBOTOPICFORWARDER_H
#define COLLISION_BENCHMARK_GAZEBOTOPICFORWARDER_H

#include <collision_benchmark/MirrorWorld.hh>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/transport/Publisher.hh>
#include <gazebo/transport/Subscriber.hh>

#include <mutex>
#include <condition_variable>

#include <string>
#include <iostream>

namespace collision_benchmark
{

/**
 * \brief Forwards messages of a Gazebo world to a separate topic.
 *
 * Only supports worlds which use a Gazebo world.
 *
 * \author Jennifer Buehler
 * \date February 2016
 */
class GazeboTopicForwarder:
  public MirrorWorld//<gazebo::physics::WorldState>
{
    public: typedef std::shared_ptr<GazeboTopicForwarder> Ptr;
    public: typedef std::shared_ptr<const GazeboTopicForwarder> ConstPtr;
    public: typedef MirrorWorld Parent;
    //public: typedef MirrorWorld<gazebo::physics::WorldState> Parent;

    // PhysicsEngineWorldInterface instantiated with Gazebo types.
    // Required to use GetWorld().
    private: typedef PhysicsEngineWorldInterface<
                std::string,
                gazebo::physics::Model,
                gazebo::physics::Contact,
                gazebo::physics::PhysicsEngine,
                gazebo::physics::World> GazeboPhysicsEngineWorld;

    /// Constructor.
    public:  GazeboTopicForwarder(const std::string& worldname="default");
    // prohibit copy constructor
    private: GazeboTopicForwarder(const GazeboTopicForwarder& o){}
    public:  ~GazeboTopicForwarder();

    /// Documentation inherited
    public:  virtual void Sync();
    protected: virtual void NotifyOriginalWorldChanged();

    private: void Init();

    // connect the subscribers to the world name of this topic
    private: void ConnectOriginalWorld(const std::string origWorldName);


    /// \brief Called when a request message is received.
    private: void OnRequest(ConstRequestPtr &_msg);

    /// \brief Called when a response to the original world comes back
    private: void OnOrigResponse(ConstResponsePtr &_msg);

    /// \brief Called when a world stat message is received.
    private: void OnWorldStatMsg(ConstWorldStatisticsPtr &_msg);

    /// \brief Called when a model message is received.
    private: void OnModelMsg(ConstModelPtr &_msg);

    /// \brief Called when a light message is received.
    private: void OnLightMsg(ConstLightPtr &_msg);

    /// \brief Transportation node.
    private: gazebo::transport::NodePtr node;

    /// \brief Publisher for world statistics messages.
    private: gazebo::transport::PublisherPtr statPub;
    private: gazebo::transport::SubscriberPtr statSub;

    /// \brief Publisher for model messages.
    private: gazebo::transport::PublisherPtr modelPub;
    private: gazebo::transport::SubscriberPtr modelSub;

    /// \brief Publisher for gui messages.
    private: gazebo::transport::PublisherPtr guiPub;
    private: gazebo::transport::SubscriberPtr guiSub;

    /// \brief Publisher for light messages.
    private: gazebo::transport::PublisherPtr lightPub;
    private: gazebo::transport::SubscriberPtr lightSub;

    /// \brief Publisher for pose messages.
    private: gazebo::transport::PublisherPtr posePub;



    /// \brief Publisher for request response messages.
    private: gazebo::transport::PublisherPtr responsePub;

    /// \brief Subscriber to request messages.
    private: gazebo::transport::SubscriberPtr requestSub;

    /// \brief Publisher to send request messages to original world.
    private: gazebo::transport::PublisherPtr requestOrigPub;

    /// \brief Subscriber to receive responses from original world.
    private: gazebo::transport::SubscriberPtr responseOrigSub;

    /// \brief Outgoing world statistics message.
/*    private: gazebo::msgs::WorldStatistics worldStatsMsg;

    /// \brief Outgoing scene message.
    private: gazebo::msgs::Scene sceneMsg;*/

    private: std::string worldName;
};

}  // namespace collision_benchmark
#endif
