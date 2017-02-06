/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
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
#include <collision_benchmark/GazeboControlServer.hh>
#include <collision_benchmark/Exception.hh>

#include <gazebo/transport/TransportIface.hh>

using collision_benchmark::GazeboControlServer;

GazeboControlServer::GazeboControlServer(const std::string &_worldName)
{
  Init(_worldName);
}

GazeboControlServer::~GazeboControlServer()
{
}

void GazeboControlServer::Init(const std::string &_worldName)
{
  this->node = gazebo::transport::NodePtr(new gazebo::transport::Node());
  this->node->Init(_worldName);

  this->controlSub = this->node->Subscribe("~/world_control",
       &GazeboControlServer::OnWorldControl, this);

  this->modelModSub = this->node->Subscribe<gazebo::msgs::Model>(
      "~/model/modify", &GazeboControlServer::OnModelModify, this);

  this->poseModSub = this->node->Subscribe<gazebo::msgs::Pose>(
      "~/pose/modify", &GazeboControlServer::OnPoseModify, this);

  if (!this->generalCtrlNode)
  {
    this->generalCtrlNode.reset(new gazebo::transport::Node());
    this->generalCtrlNode->Init();
  }
  if (!this->wldIdxCtrlSubscriber)
    this->wldIdxCtrlSubscriber =
      generalCtrlNode->Subscribe("mirror_world/set_world",
                      &GazeboControlServer::WorldSelectClientCallback, this);
  if (!this->wldIdxCtrlPublisher)
    this->wldIdxCtrlPublisher  =
      generalCtrlNode->Advertise<gazebo::msgs::Any>("mirror_world/get_world");

}

void GazeboControlServer::OnWorldControl
      (const boost::shared_ptr<gazebo::msgs::WorldControl const> &_msg)
{
  std::cout<<"Received world control"<<std::endl;
  if (_msg->has_pause())
    this->NotifyPause(_msg->pause());
  if (_msg->has_step())
    this->NotifyUpdate(1);
  if (_msg->has_multi_step())
    this->NotifyUpdate(_msg->multi_step());
}

void GazeboControlServer::OnModelModify
      (const boost::shared_ptr<gazebo::msgs::Model const> &_msg)
{
  std::cout<<"Received model modify"<<std::endl;
}

void GazeboControlServer::OnPoseModify
      (const boost::shared_ptr<gazebo::msgs::Pose const> &_msg)
{
  std::cout<<"Received pose modify"<<std::endl;
}

void GazeboControlServer::SendWorldName(const std::string& name)
{
  gazebo::msgs::Any m;
  m.set_type(gazebo::msgs::Any::STRING);
  m.set_string_value(name);
  this->wldIdxCtrlPublisher->Publish(m);
}


void GazeboControlServer::WorldSelectClientCallback(ConstAnyPtr &_msg)
{
  // std::cout << "Received: "<<_msg->DebugString();
  if (_msg->type() != gazebo::msgs::Any::INT32)
  {
    gzerr<<"Received Control message of invalid type, expecting INT32.\n"<<_msg->DebugString();
    return;
  }
  int ctrl=_msg->int_value();
  std::string worldName = this->CallSelectWorld(ctrl);
  if (!worldName.empty())
    this->SendWorldName(worldName);
}

