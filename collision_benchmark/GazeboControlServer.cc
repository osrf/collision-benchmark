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

BasicState GetBasicState(const gazebo::msgs::Factory &_msg)
{
  BasicState state;
  if (_msg.has_pose())
  {
    ignition::math::Pose3d p = ConvertIgn(_msg.pose());
    state.SetPosition(p.Pos().X(), p.Pos().Y(), p.Pos().Z());
    state.SetRotation(p.Rot().X(), p.Rot().Y(), p.Rot().Z(), p.Rot().W());
  }

  state.SetScale(1,1,1);

  return state;
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
    std::cout<<"WARNING: GazeboControlServer is ignoring model "
      <<"message link field. Not supported."<<std::endl;
  }

  // cannot support nested models at this point, only changes for
  // model itself
  if (_msg.model_size() > 0)
  {
    std::cout<<"WARNING: GazeboControlServer is ignoring model "
      <<"message model field. Not supported."<<std::endl;
  }

  // cannot support is_static as it's not part of ModelState
  if (_msg.has_is_static())
  {
    std::cout<<"WARNING: GazeboControlServer is ignoring model "
      <<"message is_static field. Not supported."<<std::endl;
  }

  // cannot support enable_wind as it's not part of ModelState
  if (_msg.has_enable_wind())
  {
    std::cout<<"WARNING: GazeboControlServer is ignoring model "
      <<"message enable_wind field. Not supported."<<std::endl;
  }

  // cannot support joint as it's not part of ModelState
  if (_msg.joint_size() > 0)
  {
    std::cout<<"WARNING: GazeboControlServer is ignoring model "
      <<"message joint field. Not supported."<<std::endl;
  }
  // cannot support deleted as it's not part of ModelState
  if (_msg.has_deleted())
  {
    std::cout<<"WARNING: GazeboControlServer is ignoring model "
      <<"message deleted field. Not supported."<<std::endl;
  }
  // cannot support visual as it's not part of ModelState
  if (_msg.visual_size() > 0)
  {
    std::cout<<"WARNING: GazeboControlServer is ignoring model "
      <<"message visual field. Not supported."<<std::endl;
  }
  // cannot support self_collide as it's not part of ModelState
  if (_msg.has_self_collide())
  {
    std::cout<<"WARNING: GazeboControlServer is ignoring model "
      <<"message self_collide field. Not supported."<<std::endl;
  }
  // cannot support plugin as it's not part of ModelState
  if (_msg.plugin_size() > 0)
  {
    std::cout<<"WARNING: GazeboControlServer is ignoring model "
      <<"message plugin field. Not supported."<<std::endl;
  }
  return state;
}



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

  this->userCmdSub = this->node->Subscribe<gazebo::msgs::UserCmd>(
      "~/user_cmd", &GazeboControlServer::OnUserCmd, this);

  this->factorySub = this->node->Subscribe<gazebo::msgs::Factory>(
      "~/factory", &GazeboControlServer::OnFactory, this);

  this->physicsSub = this->node->Subscribe<gazebo::msgs::Physics>(
      "~/physics", &GazeboControlServer::OnPhysics, this);

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
  this->HandleWorldControl(*_msg);
}

void GazeboControlServer::HandleWorldControl
  (const gazebo::msgs::WorldControl &_msg)
{
  if (_msg.has_pause())
    this->NotifyPause(_msg.pause());
  if (_msg.has_step())
    this->NotifyUpdate(1);
  if (_msg.has_multi_step())
    this->NotifyUpdate(_msg.multi_step());
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


void GazeboControlServer::OnUserCmd
  (const boost::shared_ptr<gazebo::msgs::UserCmd const> &_msg)
{
  std::cout<<"GazeboControlServer: Received user cmd"<<std::endl;

  // Forward message after we've saved the current state
  switch (_msg->type())
  {
    case gazebo::msgs::UserCmd::MOVING:
    {
      for (int i = 0; i < _msg->model_size(); ++i)
      {
        BasicState mState = GetModelChangeState(_msg->model(i));
        // std::cout<<"Constructed model state: "<<mState<<std::endl;
        this->NotifySetModelState(_msg->model(i).name(), mState);
      }

      if (_msg->light_size() > 0)
      {
        std::cout<<"WARNING: GazeboControlServer is ignoring light "
          <<"modification field in user command. Not supported."<<std::endl;
      }
      break;
    }
    case gazebo::msgs::UserCmd::SCALING:
    {
      for (int i = 0; i < _msg->model_size(); ++i)
      {
        BasicState mState = GetModelChangeState(_msg->model(i));
        // std::cout<<"Constructed model state: "<<mState<<std::endl;
        this->NotifySetModelState(_msg->model(i).name(), mState);
      }
      break;
    }
    case gazebo::msgs::UserCmd::WORLD_CONTROL:
    {
      if (_msg->has_world_control())
      {
        HandleWorldControl(_msg->world_control());
      }
      else
      {
        gzwarn << "World control command [" << _msg->description() <<
            "] without a world control message. Command won't be executed."
            << std::endl;
      }
      break;
    }
    case gazebo::msgs::UserCmd::WRENCH:
    {
      gzwarn << "WARNING: GazeboControlServer is ignoring wrench "
          <<"field in user command. Not supported (yet)."<<std::endl;
      // Set publisher
      /*std::string topicName = "~/";
      topicName += _msg->entity_name() + "/wrench";
      boost::replace_all(topicName, "::", "/");
      auto wrenchPub = this->dataPtr->node->Advertise<msgs::Wrench>(topicName);
      wrenchPub->Publish(_msg->wrench());
      wrenchPub->Fini();*/
      break;
    }
    default:
    {
      gzwarn << "Unsupported command type [" << _msg->type() << "]" <<
          std::endl;
      break;
    }
  }
}

void GazeboControlServer::OnPhysics
  (const boost::shared_ptr<gazebo::msgs::Physics const> &_msg)
{
  GZ_ASSERT(_msg, "Message must not be NULL");
//  std::cout << "GazeboControlServer received physics msg "
//            << _msg->DebugString() << std::endl;

  // all fields will be dropped, except enable_physics
  // and gravity
  if (!_msg->has_enable_physics() &&
      !_msg->has_gravity())
  {
    std::cout << "Only 'enable_physics' and 'gravity' fields "
              << "of msgs::Physics messages can be forwarded, "
              << "but none of those fields are set." <<std::endl;
    return;
  }
  if (_msg->has_enable_physics())
    NotifyDynamicsEnable(_msg->enable_physics());
  if (_msg->has_gravity())
      NotifyGravity(_msg->gravity().x(),
                    _msg->gravity().y(),
                    _msg->gravity().z());
}

void GazeboControlServer::OnFactory
  (const boost::shared_ptr<gazebo::msgs::Factory const> &_msg)
{
  std::string sdf;
  bool isString=true;

  if (_msg->has_sdf() && !_msg->sdf().empty())
  {
    sdf=_msg->sdf();
  }
  else if (_msg->has_sdf_filename() &&
          !_msg->sdf_filename().empty())
  {
    sdf=_msg->sdf();
    isString=false;
  }
  else if (_msg->has_clone_model_name())
  {
    gzwarn << "WARNING: GazeboControlServer is ignoring clone model "
        <<"field in factory message. Not supported."<<std::endl;
    return;
  }
  else
  {
    gzerr << "Unable to load sdf from factory message."
      << "No SDF or SDF filename specified."<<std::endl;
    return;
  }

  if (_msg->has_edit_name())
  {
    gzwarn << "WARNING: GazeboControlServer is ignoring name edit "
        <<"field in factory message. Not supported."<<std::endl;
  }

  BasicState state = GetBasicState(*_msg);
  NotifySdfModelLoad(sdf, isString, state);
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
    gzerr<<"Received Control message of invalid type, expecting INT32: "
      <<_msg->DebugString()<<std::endl;
    return;
  }
  int ctrl=_msg->int_value();
  std::string worldName = this->CallSelectWorld(ctrl);
  if (!worldName.empty())
    this->SendWorldName(worldName);
}

