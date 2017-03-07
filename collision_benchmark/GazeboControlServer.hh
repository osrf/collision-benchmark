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
 * Date: February 2017
 */
#ifndef COLLISION_BENCHMARK_GAZEBOCONTROLSERVER_H
#define COLLISION_BENCHMARK_GAZEBOCONTROLSERVER_H

#include <collision_benchmark/ControlServer.hh>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/WorldState.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/transport/Publisher.hh>
#include <gazebo/transport/Subscriber.hh>

#include <string>
#include <iostream>

namespace collision_benchmark
{

/**
 * \author Jennifer Buehler
 * \date February 2017
 */
class GazeboControlServer:
  public ControlServer<std::string>
{
  private: typedef ControlServer<std::string> Super;
  public: typedef std::shared_ptr<GazeboControlServer> Ptr;
  public: typedef std::shared_ptr<const GazeboControlServer> ConstPtr;

  /// Constructor.
  public:  GazeboControlServer(const std::string& _worldName="default");

  // prohibit copy constructor
  private: GazeboControlServer(const GazeboControlServer& o){}
  public:  ~GazeboControlServer();

  private: void Init(const std::string &_worldName);

  private: void OnWorldControl
           (const boost::shared_ptr<gazebo::msgs::WorldControl const> &_msg);
  private: void OnModelModify
           (const boost::shared_ptr<gazebo::msgs::Model const> &_msg);
  private: void OnPoseModify
           (const boost::shared_ptr<gazebo::msgs::Pose const> &_msg);
  private: void OnUserCmd
           (const boost::shared_ptr<gazebo::msgs::UserCmd const> &_msg);
  private: void OnFactory
           (const boost::shared_ptr<gazebo::msgs::Factory const> &_msg);
  private: void OnPhysics
           (const boost::shared_ptr<gazebo::msgs::Physics const> &_msg);

  private: void HandleWorldControl (const gazebo::msgs::WorldControl &_msg);

  /// Callback for control client subscriber \e wldIdxCtrlSubscriber.
  /// Will send back the current world name with \e wldIdxCtrlPublisher.
  private:  void WorldSelectClientCallback(ConstAnyPtr &_msg);
  /// sends the name of a word via \e wldIdxCtrlPublisher
  private: void SendWorldName(const std::string& name);

  /// \brief Transportation node, initialized with world name
  private: gazebo::transport::NodePtr node;

  /// \brief Subscriber to world control messages.
  private: gazebo::transport::SubscriberPtr controlSub;

  /// \brief Subscriber to model messages.
  private: gazebo::transport::SubscriberPtr modelModSub;

  /// \brief Subscriber to pose modify messages.
  private: gazebo::transport::SubscriberPtr poseModSub;

  /// \brief Subscriber to UserCmd messages.
  private: gazebo::transport::SubscriberPtr userCmdSub;

  /// \brief Subscriber to Factory messages.
  private: gazebo::transport::SubscriberPtr factorySub;

  /// \brief Subscriber to Physics messages.
  private: gazebo::transport::SubscriberPtr physicsSub;

  // subscriber for change of world index
  private: gazebo::transport::SubscriberPtr wldIdxCtrlSubscriber;
  // publisher for change of world name
  private: gazebo::transport::PublisherPtr wldIdxCtrlPublisher;

  // General node, initialized without namespace
  private: gazebo::transport::NodePtr generalCtrlNode;
};

}  // namespace collision_benchmark
#endif
