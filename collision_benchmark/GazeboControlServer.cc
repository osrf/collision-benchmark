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
#include <collision_benchmark/BasicTypes.hh>

#include <gazebo/transport/TransportIface.hh>

using collision_benchmark::GazeboControlServer;
using collision_benchmark::Vector3;
using collision_benchmark::Quaternion;
using collision_benchmark::BasicState;

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
  std::cout<<"GazeboControlServer: Received world control"<<std::endl;
  if (_msg->has_pause())
    this->NotifyPause(_msg->pause());
  if (_msg->has_step())
    this->NotifyUpdate(1);
  if (_msg->has_multi_step())
    this->NotifyUpdate(_msg->multi_step());
}

BasicState GetModelChangeState(const gazebo::msgs::Model &_msg)
{
  BasicState state;

  if (_msg.has_pose())
  {
    ignition::math::Pose3d p = ConvertIgn(_msg.pose());
    state.SetPosition(p.Pos().X(), p.Pos().Y(), p.Pos().Z());
    state.SetRotation(p.Rot().X(), p.Rot().Y(), p.Rot().Z(), p.Rot().W());
  }

  if (_msg.has_scale())
  {
    ignition::math::Vector3d s = ConvertIgn(_msg.scale());
    state.SetScale(s.X(), s.Y(), s.Z());
  }

  // cannot support links at this point, only changes for
  // model itself
  if (_msg.link_size() > 0)
  {
    std::cout<<"WARNING: GazeboControlServer is ignoring model \
      message link field. Not supported."<<std::endl;
  }

  // cannot support nested models at this point, only changes for
  // model itself
  if (_msg.model_size() > 0)
  {
    std::cout<<"WARNING: GazeboControlServer is ignoring model \
      message model field. Not supported."<<std::endl;
  }

  // cannot support is_static as it's not part of ModelState
  if (_msg.has_is_static())
  {
    std::cout<<"WARNING: GazeboControlServer is ignoring model \
      message is_static field. Not supported."<<std::endl;
  }

  // cannot support enable_wind as it's not part of ModelState
  if (_msg.has_enable_wind())
  {
    std::cout<<"WARNING: GazeboControlServer is ignoring model \
      message enable_wind field. Not supported."<<std::endl;
  }

  // cannot support joint as it's not part of ModelState
  if (_msg.joint_size() > 0)
  {
    std::cout<<"WARNING: GazeboControlServer is ignoring model \
      message joint field. Not supported."<<std::endl;
  }
  // cannot support deleted as it's not part of ModelState
  if (_msg.has_deleted())
  {
    std::cout<<"WARNING: GazeboControlServer is ignoring model \
      message deleted field. Not supported."<<std::endl;
  }
  // cannot support visual as it's not part of ModelState
  if (_msg.visual_size() > 0)
  {
    std::cout<<"WARNING: GazeboControlServer is ignoring model \
      message visual field. Not supported."<<std::endl;
  }
  // cannot support self_collide as it's not part of ModelState
  if (_msg.has_self_collide())
  {
    std::cout<<"WARNING: GazeboControlServer is ignoring model \
      message self_collide field. Not supported."<<std::endl;
  }
  // cannot support plugin as it's not part of ModelState
  if (_msg.plugin_size() > 0)
  {
    std::cout<<"WARNING: GazeboControlServer is ignoring model \
      message plugin field. Not supported."<<std::endl;
  }
  return state;
}

void GazeboControlServer::OnModelModify
      (const boost::shared_ptr<gazebo::msgs::Model const> &_msg)
{
  std::cout<<"GazeboControlServer: Received model modify"<<std::endl;
  BasicState mState = GetModelChangeState(*_msg);
  // std::cout<<"Constructed model state: "<<mState<<std::endl;
  this->NotifySetModelState(_msg->name(), mState);
}

void GazeboControlServer::OnPoseModify
      (const boost::shared_ptr<gazebo::msgs::Pose const> &_msg)
{
  std::cout<<"GazeboControlServer: Received pose modify"<<std::endl;
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

