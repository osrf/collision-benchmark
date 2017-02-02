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
#include <collision_benchmark/GazeboTopicForwarder.hh>
#include <collision_benchmark/Exception.hh>

#include <collision_benchmark/GazeboPhysicsWorld.hh>
#include <collision_benchmark/GazeboStateCompare.hh>
#include <collision_benchmark/GazeboWorldState.hh>
#include <collision_benchmark/GazeboHelpers.hh>
#include <gazebo/physics/PhysicsIface.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/ContactManager.hh>


#include <gazebo/transport/TransportIface.hh>

using collision_benchmark::GazeboTopicForwarder;

GazeboTopicForwarder::GazeboTopicForwarder(const std::string& worldname):
  worldName(worldname)
{
  Init();
}

GazeboTopicForwarder::~GazeboTopicForwarder()
{
}

void GazeboTopicForwarder::ConnectOriginalWorld(const std::string origWorldName)
{
  std::cout<<"Mirror is connecting to world '"<<origWorldName<<"'"<<std::endl;

  this->responseOrigSub = this->node->Subscribe("/gazebo/"+origWorldName+"/response",
                                           &GazeboTopicForwarder::OnOrigResponse, this, true);

  this->requestOrigPub = this->node->Advertise<gazebo::msgs::Request>
                                          ("/gazebo/"+origWorldName+"/request");

  this->statSub = this->node->Subscribe("/gazebo/"+origWorldName+"/world_stats",
                                         &GazeboTopicForwarder::OnWorldStatMsg,
                                         this, true);

  this->modelSub = this->node->Subscribe("/gazebo/"+origWorldName+"/model/info",
                                         &GazeboTopicForwarder::OnModelMsg,
                                         this, true);

  this->lightSub = this->node->Subscribe("/gazebo/"+origWorldName+"/light",
                                         &GazeboTopicForwarder::OnLightMsg,
                                         this, true);
}

void GazeboTopicForwarder::Init()
{
  this->node = gazebo::transport::NodePtr(new gazebo::transport::Node());
  this->node->Init(this->worldName);

  // pose pub for client with a cap on publishing rate to reduce traffic
  // overhead
  this->posePub = this->node->Advertise<gazebo::msgs::PosesStamped>(
    "~/pose/info", 10, 60);

  this->guiPub = this->node->Advertise<gazebo::msgs::GUI>("~/gui", 5);

  // World statistics
  this->statPub =
    this->node->Advertise<gazebo::msgs::WorldStatistics>(
        "~/world_stats", 100, 5);

  this->modelPub = this->node->Advertise<gazebo::msgs::Model>(
      "~/model/info");

  this->lightPub = this->node->Advertise<gazebo::msgs::Light>(
      "~/light/modify");

  this->requestSub = this->node->Subscribe("~/request",
                                           &GazeboTopicForwarder::OnRequest, this, true);
  this->responsePub = this->node->Advertise<gazebo::msgs::Response>(
      "~/response");
}

void GazeboTopicForwarder::OnRequest(ConstRequestPtr &_msg)
{
  std::cout<<"Got a request: "<<_msg->request()<<std::endl;
  gazebo::msgs::Response response;
  response.set_id(_msg->id());
  response.set_request(_msg->request());
  response.set_response("failure");

  if (_msg->request() == "entity_delete")
  {
    std::cout<<"Entity deletion not supported!"<<std::endl;
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

void GazeboTopicForwarder::OnOrigResponse(ConstResponsePtr &_msg)
{
  std::cout<<"Got a response: "<<_msg->response()<<std::endl;
  this->responsePub->Publish(*_msg);
}


void GazeboTopicForwarder::OnWorldStatMsg(ConstWorldStatisticsPtr &_msg)
{
  assert(_msg);
  // std::cout<<"Received a world stat iter= "<<_msg->iterations()<<std::endl;
  this->statPub->Publish(*_msg);
}

void GazeboTopicForwarder::OnModelMsg(ConstModelPtr &_msg)
{
  assert(_msg);
  // std::cout<<"Received a model msg"<<std::endl;
  this->modelPub->Publish(*_msg);
}

void GazeboTopicForwarder::OnLightMsg(ConstLightPtr &_msg)
{
  assert(_msg);
  std::cout<<"Received a lightmsg"<<std::endl;
  this->lightPub->Publish(*_msg);
}


void GazeboTopicForwarder::NotifyOriginalWorldChanged()
{
  GazeboPhysicsEngineWorld::Ptr gzOrigWorld =
    std::dynamic_pointer_cast<GazeboPhysicsEngineWorld>(GetOriginalWorld());
  if (!gzOrigWorld)
  {
    THROW_EXCEPTION("Only Gazebo original worlds supported");
  }

  ConnectOriginalWorld(GetOriginalWorld()->GetName());
/*
  // unfortunately cannot access world SDF from outside...
  if (gzOrigWorld->GetWorld()->sdf->HasElement("gui"))
  {
    this->guiPub->Publish(
    gazebo::msgs::GUIFromSDF(this->sdf->GetElement("gui")));
  }*/
}

void GazeboTopicForwarder::Sync()
{
}
