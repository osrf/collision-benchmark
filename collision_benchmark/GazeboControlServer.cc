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
  std::cout<<"GazeboControlServer: Received world control"<<std::endl;
  if (_msg->has_pause())
    this->NotifyPause(_msg->pause());
  if (_msg->has_step())
    this->NotifyUpdate(1);
  if (_msg->has_multi_step())
    this->NotifyUpdate(_msg->multi_step());
}

/*sdf::ElementPtr GetCollisionSDF(const gazebo::msgs::Collision &_msg)
{
  if (!_msg.has_pose())
  {
    std::cout<<"WARNING: GazeboControlServer expects collision message \
      to have at least pose specified. Skipping message."<<std::endl;
    return nullptr;
  }

  // need to go via an SDF to load the state because CollisionState does not
  // provide options to set individual fields (there are only getters)
  sdf::ElementPtr sdf(new sdf::Element());
  sdf->SetName("collision");
  sdf->AddValue("string", _msg.name(), true, "Collision name");

  sdf::ElementPtr poseSDF(new sdf::Element());
  poseSDF->SetName("pose");
  poseSDF->Set(gazebo::msgs::ConvertIgn(_msg.pose()));
  sdf->InsertElement(poseSDF);

  // CollisionState does not have a field for laser_retro
  if (_msg.has_laser_retro())
  {
    std::cout<<"WARNING: GazeboControlServer is ignoring link \
      message laser_retro field. Not supported."<<std::endl;
  }

  // CollisionState does not have a field for max_contacts
  if (_msg.has_max_contacts())
  {
    std::cout<<"WARNING: GazeboControlServer is ignoring link \
      message max_contacts field. Not supported."<<std::endl;
  }

  // CollisionState does not have a field for geometry
  if (_msg.has_geometry())
  {
    std::cout<<"WARNING: GazeboControlServer is ignoring link \
      message geometry field. Not supported."<<std::endl;
  }

  // CollisionState does not have a field for surface
  if (_msg.has_surface())
  {
    std::cout<<"WARNING: GazeboControlServer is ignoring link \
      message surface field. Not supported."<<std::endl;
  }


  // CollisionState does not have any visuals
  if (_msg.visual_size() > 0)
  {
    std::cout<<"WARNING: GazeboControlServer is ignoring link \
      message visual field. Not supported."<<std::endl;
  }


  return sdf;
}

bool GetCollisionState(const gazebo::msgs::Collision &_msg, gazebo::physics::CollisionState& cState)
{
  sdf::ElementPtr sdf = GetCollisionSDF(_msg);
  if (!sdf) return false;
  cState.Load(sdf);
  std::cout<<"DEBUG "<<__FILE__<<": Loaded collision state: "<<cState<<std::endl;
  return true;
}*/

sdf::ElementPtr GetLinkSDF(const gazebo::msgs::Link &_msg)
{
  // message must have at least a pose in order to have
  // any effect
  if (!_msg.has_pose())
  {
    std::cout<<"WARNING: GazeboControlServer expects link message \
      to have at least pose specified. Skipping message."<<std::endl;
    return nullptr;
  }

  // need to go via an SDF to load the state because CollisionState does not
  // provide options to set individual fields (there are only getters)
  sdf::ElementPtr sdf(new sdf::Element());
  sdf->SetName("link");
  sdf->AddValue("string", _msg.name(), true, "Name");

  sdf::ElementPtr poseSDF(new sdf::Element());
  poseSDF->SetName("pose");
  poseSDF->Set(gazebo::msgs::ConvertIgn(_msg.pose()));
  sdf->InsertElement(poseSDF);

  // LinkState does not have a field for self collision.
  if (_msg.has_self_collide())
  {
    std::cout<<"WARNING: GazeboControlServer is ignoring link \
      message self_collide field. Not supported."<<std::endl;
  }

  // LinkState has no field to change gravity.
  if (_msg.has_gravity())
  {
    std::cout<<"WARNING: GazeboControlServer is ignoring link \
      message gravity field. Not supported."<<std::endl;
  }

  // LinkState has no field to enable wind.
  if (_msg.has_enable_wind())
  {
    std::cout<<"WARNING: GazeboControlServer is ignoring link \
      message enable_wind field. Not supported."<<std::endl;
  }

  if (_msg.has_kinematic())
  {
    std::cout<<"WARNING: GazeboControlServer is ignoring link \
      message kinematic field. Not supported."<<std::endl;
  }

  if (_msg.has_inertial())
  {
    std::cout<<"WARNING: GazeboControlServer is ignoring link \
      message inertial field. Not supported."<<std::endl;
  }

  if (_msg.visual_size()>0)
  {
    std::cout<<"WARNING: GazeboControlServer is ignoring link \
      message inertial field. Not supported."<<std::endl;
  }

  // the SDF does not support adding of collisions, but it looks
  // anyway as it's not used at the moment, as in LinkState all
  // functionality is commented out.
/*  for (int i = 0; i < _msg.collision_size(); i++)
  {
    sdf::ElementPtr collSDF = GetCollisionSDF(_msg.collision(i));
    if (collSDF) sdf->InsertElement(collSDF);
  }*/

  return sdf;
}

bool GetLinkState(const gazebo::msgs::Link &_msg, gazebo::physics::LinkState& lState)
{
  sdf::ElementPtr sdf = GetLinkSDF(_msg);
  if (!sdf) return false;
  lState.Load(sdf);
  std::cout<<"DEBUG "<<__FILE__<<": Loaded link state: "<<lState<<std::endl;
  return true;
}


sdf::ElementPtr GetModelSDF(const gazebo::msgs::Model &_msg)
{
  if (!_msg.has_pose() ||
      !_msg.has_scale())
  {
    std::cout<<"WARNING: GazeboControlServer expects model message \
      to have at least pose and scale specified. Skipping message."<<std::endl;
    std::cout<<_msg.DebugString()<<std::endl;
    return nullptr;
  }

  // need to go via an SDF to load the state because CollisionState does not
  // provide options to set individual fields (there are only getters)
  sdf::ElementPtr sdf(new sdf::Element());
  sdf->SetName("model");
  sdf->AddValue("string", _msg.name(), true, "Name");

  sdf::ElementPtr poseSDF(new sdf::Element());
  poseSDF->SetName("pose");
  poseSDF->Set(gazebo::msgs::ConvertIgn(_msg.pose()));
  sdf->InsertElement(poseSDF);

  sdf::ElementPtr scaleSDF(new sdf::Element());
  scaleSDF->SetName("scale");
  scaleSDF->Set(gazebo::msgs::ConvertIgn(_msg.scale()));
  sdf->InsertElement(scaleSDF);

  for (int i = 0; i < _msg.link_size(); i++)
  {
    sdf::ElementPtr linkSDF = GetLinkSDF(_msg.link(i));
    if (linkSDF)
      sdf->InsertElement(linkSDF);
  }

  // at this point, physics::Model ignores nested models in the
  // state message, but copy it anyway.
  for (int i = 0; i < _msg.model_size(); i++)
  {
    sdf::ElementPtr nModelSDF = GetModelSDF(_msg.model(i));
    if (nModelSDF)
    {
      sdf->InsertElement(nModelSDF);
    }
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
  return sdf;
}

bool GetModelState(const gazebo::msgs::Model &_msg, gazebo::physics::ModelState& mState)
{
  sdf::ElementPtr sdf = GetModelSDF(_msg);
  if (!sdf) return false;
  mState.Load(sdf);
  std::cout<<"DEBUG "<<__FILE__<<": Loaded model state: "<<mState<<std::endl;
  return true;
}

void GazeboControlServer::OnModelModify
      (const boost::shared_ptr<gazebo::msgs::Model const> &_msg)
{
  std::cout<<"GazeboControlServer: Received model modify"<<std::endl;

  // get the model state
  gazebo::physics::ModelState mState;
  if (GetModelState(*_msg, mState))
  {
    std::cout<<"Constructed model state: "<<mState<<std::endl;
  }
  else
  {
    std::cerr<<"ERROR: Could not use the model state in the\
      message to construct a state: "<<_msg->DebugString()<<std::endl;
    return;
  }

  // construct world state
  gazebo::physics::WorldState worldState;
//  worldState.
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

