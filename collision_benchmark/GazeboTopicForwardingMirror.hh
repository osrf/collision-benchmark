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
#ifndef COLLISION_BENCHMARK_GAZEBOTOPICFORWARDINGMIRROR_H
#define COLLISION_BENCHMARK_GAZEBOTOPICFORWARDINGMIRROR_H

#include <collision_benchmark/MirrorWorld.hh>
#include <collision_benchmark/GazeboTopicForwarder.hh>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/transport/Publisher.hh>
#include <gazebo/transport/Subscriber.hh>

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
class GazeboTopicForwardingMirror:
  public MirrorWorld//<gazebo::physics::WorldState>
{
    public: typedef std::shared_ptr<GazeboTopicForwardingMirror> Ptr;
    public: typedef std::shared_ptr<const GazeboTopicForwardingMirror> ConstPtr;
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
    public:  GazeboTopicForwardingMirror(const std::string& worldname="default");
    // prohibit copy constructor
    private: GazeboTopicForwardingMirror(const GazeboTopicForwardingMirror& o){}
    public:  ~GazeboTopicForwardingMirror();

    /// Documentation inherited
    public:  virtual void Sync();
    protected: virtual void NotifyOriginalWorldChange
                  (const OriginalWorldPtr &_newWorld);

    private: void Init();

    // connect the subscribers to the world name of this topic
    private: void ConnectOriginalWorld(const std::string origWorldName);

    private: void DisconnectFromOriginal();

    /// \brief Called when a request message is received.
    private: void OnRequest(ConstRequestPtr &_msg);

    /// \brief Called when a response to the original world comes back
    private: void OnOrigResponse(ConstResponsePtr &_msg);

    /// \brief Transportation node.
    private: gazebo::transport::NodePtr node;

    /// \brief topic forwarder for WorldStatistics
    private: GazeboTopicForwarder<gazebo::msgs::WorldStatistics>::Ptr statFwd;

    /// \brief topic forwarder for model messages.
    private: GazeboTopicForwarder<gazebo::msgs::Model>::Ptr modelFwd;

    /// \brief topic forwarder for gui messages.
    private: GazeboTopicForwarder<gazebo::msgs::GUI>::Ptr guiFwd;

    /// \brief topic forwarder for light messages.
    private: GazeboTopicForwarder<gazebo::msgs::Light>::Ptr lightFwd;

    /// \brief topic forwarder for pose messages.
    private: GazeboTopicForwarder<gazebo::msgs::PosesStamped>::Ptr poseFwd;

    /// \brief Publisher for request response messages.
    private: gazebo::transport::PublisherPtr responsePub;

    /// \brief Publisher for request messages to use for clients subscribed
    /// the the mirror world (e.g. visualization/gzclient).
    private: gazebo::transport::PublisherPtr requestPub;

    /// \brief Subscriber to request messages.
    private: gazebo::transport::SubscriberPtr requestSub;

    /// \brief Publisher to send request messages to original world.
    private: gazebo::transport::PublisherPtr requestOrigPub;

    /// \brief Subscriber to receive responses from original world.
    private: gazebo::transport::SubscriberPtr responseOrigSub;

    private: std::string worldName;
};

}  // namespace collision_benchmark
#endif
