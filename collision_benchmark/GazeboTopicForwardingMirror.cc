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
#include <collision_benchmark/GazeboTopicForwardingMirror.hh>
#include <collision_benchmark/Exception.hh>

#include <collision_benchmark/GazeboPhysicsWorld.hh>
#include <collision_benchmark/GazeboStateCompare.hh>
#include <collision_benchmark/GazeboWorldState.hh>
#include <collision_benchmark/GazeboHelpers.hh>
#include <gazebo/physics/PhysicsIface.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/ContactManager.hh>
#include <gazebo/physics/Model.hh>


#include <gazebo/transport/TransportIface.hh>

using collision_benchmark::GazeboTopicForwarder;
using collision_benchmark::GazeboTopicForwardingMirror;

GazeboTopicForwardingMirror::GazeboTopicForwardingMirror(const std::string& worldname):
  worldName(worldname)
{
  Init();
}

GazeboTopicForwardingMirror::~GazeboTopicForwardingMirror()
{
}

void GazeboTopicForwardingMirror::ConnectOriginalWorld(const std::string origWorldName)
{
  std::cout<<"Mirror is connecting to world '"<<origWorldName<<"'"<<std::endl;

  this->responseOrigSub = this->node->Subscribe("/gazebo/"+origWorldName+"/response",
                                           &GazeboTopicForwardingMirror::OnOrigResponse, this, true);

  this->requestOrigPub = this->node->Advertise<gazebo::msgs::Request>
                                          ("/gazebo/"+origWorldName+"/request");

  bool latch=false;
  assert(this->statFwd);
  this->statFwd->RedirectFrom("/gazebo/"+origWorldName+"/world_stats", this->node, latch);

  assert(this->modelFwd);
  this->modelFwd->RedirectFrom("/gazebo/"+origWorldName+"/model/info", this->node, latch);

  assert(this->lightFwd);
  this->lightFwd->RedirectFrom("/gazebo/"+origWorldName+"/light/modify", this->node, latch);

  assert(this->poseFwd);
  this->poseFwd->RedirectFrom("/gazebo/"+origWorldName+"/pose/info", this->node, latch);

  assert(this->guiFwd);
  this->guiFwd->RedirectFrom("/gazebo/"+origWorldName+"/gui", this->node, latch);
}

void GazeboTopicForwardingMirror::DisconnectFromOriginal()
{
  assert(this->responseOrigSub);
//  this->responseOrigSub = this->node->Disconnect();

  assert(this->statFwd);
  this->statFwd->DisconnectSubscriber();

  assert(this->modelFwd);
  this->modelFwd->DisconnectSubscriber();

  assert(this->lightFwd);
  this->lightFwd->DisconnectSubscriber();

  assert(this->poseFwd);
  this->poseFwd->DisconnectSubscriber();

  assert(this->guiFwd);
  this->guiFwd->DisconnectSubscriber();
}

/*class ModelsChangeTrigger:
  public TopicTrigger
{
  public: ModelsChangeTrigger(GazeboTopicForwardingMirror * _mirror):
          mirror(_mirror) {}

  public: virtual void Trigger(const std::string& _oldTopic,
                               const std::string& _newTopic,
                               bool _incoming) const = 0;

  public: ModelsChangeTrigger(GazeboTopicForwardingMirror * _mirror):
};*/



void GazeboTopicForwardingMirror::Init()
{
  this->node = gazebo::transport::NodePtr(new gazebo::transport::Node());
  this->node->Init(this->worldName);

  this->statFwd.reset(new GazeboTopicForwarder<gazebo::msgs::WorldStatistics>
                      ("~/world_stats", this->node));

  this->modelFwd.reset(new GazeboTopicForwarder<gazebo::msgs::Model>
                      ("~/model/info", this->node, 1000, 0, nullptr, nullptr, true));

  this->lightFwd.reset(new GazeboTopicForwarder<gazebo::msgs::Light>
                      ("~/light/modify", this->node));

  this->poseFwd.reset(new GazeboTopicForwarder<gazebo::msgs::PosesStamped>
                      ("~/pose/info", this->node));

  this->guiFwd.reset(new GazeboTopicForwarder<gazebo::msgs::GUI>
                      ("~/gui", this->node));

  this->requestSub = this->node->Subscribe("~/request",
                                           &GazeboTopicForwardingMirror::OnRequest, this, true);
  this->responsePub = this->node->Advertise<gazebo::msgs::Response>(
      "~/response");

  this->requestPub = this->node->Advertise<gazebo::msgs::Request>("~/request");
}

void GazeboTopicForwardingMirror::OnRequest(ConstRequestPtr &_msg)
{
  std::cout<<"Got a request: "<<_msg->request()<<std::endl;
  if (_msg->request() == "entity_delete")
  {
    std::cout<<"Entity deletion not forwarded, handled privately!"<<std::endl;
  }
  else
  {
    if (!this->requestOrigPub) THROW_EXCEPTION("Need to have initialized request publisher");
    std::cout<<"Waiting for connection.."<<std::endl;
    this->requestOrigPub->WaitForConnection();
    std::cout<<"Publishing message"<<std::endl;
    this->requestOrigPub->Publish(*_msg);
    // tried to use a condition variable and wait here until
    // the response comes but this blocks the callback loop
    // and the request via requestOrigPub never arrives in gazebo::World...
  }
}

void GazeboTopicForwardingMirror::OnOrigResponse(ConstResponsePtr &_msg)
{
  std::cout<<"Got a response: "<<_msg->response()<<std::endl;
  this->responsePub->Publish(*_msg);
}

void GazeboTopicForwardingMirror::NotifyOriginalWorldChange
          (const OriginalWorldPtr &_newWorld)
{
  OriginalWorldPtr oldWorld = GetOriginalWorld();
  GazeboPhysicsEngineWorld::Ptr gzNewWorld =
    std::dynamic_pointer_cast<GazeboPhysicsEngineWorld>(_newWorld);
  if (!gzNewWorld)
  {
    THROW_EXCEPTION("Only Gazebo original worlds supported");
  }

  if (oldWorld)
  {
    DisconnectFromOriginal();
    GazeboPhysicsEngineWorld::Ptr gzOldWorld =
      std::dynamic_pointer_cast<GazeboPhysicsEngineWorld>(oldWorld);
    if (!gzOldWorld)
    {
      THROW_EXCEPTION("Only Gazebo original worlds supported");
    }
    std::cout<<"Deleting all old world's models"<<std::endl;
    // delete all old models
    gazebo::physics::Model_V oldModels = gzOldWorld->GetWorld()->Models();
    for (gazebo::physics::Model_V::iterator it = oldModels.begin();
         it != oldModels.end(); ++it)
    {
      gazebo::physics::ModelPtr m=*it;
      std::cout<<"Requesting delete of "<<m->GetScopedName()<<std::endl;
      auto msg = gazebo::msgs::CreateRequest("entity_delete", m->GetScopedName());
//      msg->set_parent_name(mirrorWorldName);
      this->requestPub->Publish(*msg, true);
      delete msg;
    }

    // insert all new models
    std::cout<<"Inserting all new world's models"<<std::endl;
    // delete all new models
    gazebo::physics::Model_V newModels = gzNewWorld->GetWorld()->Models();
    for (gazebo::physics::Model_V::iterator it = newModels.begin();
         it != newModels.end(); ++it)
    {
      gazebo::physics::ModelPtr m=*it;
      std::cout<<"Requesting insertion of "<<m->GetScopedName()<<std::endl;
      gazebo::msgs::Model insModelMsg;
      m->FillMsg(insModelMsg);
      gazebo::transport::PublisherPtr pub = this->modelFwd->GetPublisher();
      pub->Publish(insModelMsg);
    }
  }

  std::cout<<"Connecting the new world "<<_newWorld->GetName()<<std::endl;
  ConnectOriginalWorld(_newWorld->GetName());
}

void GazeboTopicForwardingMirror::Sync()
{
}
