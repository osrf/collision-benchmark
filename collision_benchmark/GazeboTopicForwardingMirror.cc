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

/**
 * \brief Strategy pattern to filter request messages
 * \author Jennifer Buehler
 * \date February 2017
 */
class RequestMessageFilter:
  public collision_benchmark::MessageFilter<gazebo::msgs::Request>
{
  private: typedef  RequestMessageFilter Self;
  public: typedef std::shared_ptr<Self> Ptr;
  public: typedef std::shared_ptr<const Self> ConstPtr;

  // Determines whether a message is filtered out
  // \return true if the filter applies to the message
  public: virtual bool Filter(const boost::shared_ptr<gazebo::msgs::Request const>
                              &_msg) const
          {
              if (_msg->request() == "entity_delete")
              {
                // std::cout<<"Entity deletion not forwarded, handled privately!"<<std::endl;
                return true;
              }
              else
              {
                std::cout<<"Got a request: "<<_msg->request()<<std::endl;
              }
              return false;
          }
};

void GazeboTopicForwardingMirror::ConnectOriginalWorld(const std::string origWorldName)
{
  std::cout<<"Mirror is connecting to world '"<<origWorldName<<"'"<<std::endl;

  // connect services
  assert(this->origServiceFwd);
  this->origServiceFwd->Forward("/gazebo/"+origWorldName+"/request",
                                "/gazebo/"+origWorldName+"/response", this->node);

  // connect topic forwarders
  bool latch=false;
  assert(this->statFwd);
  this->statFwd->ForwardFrom("/gazebo/"+origWorldName+"/world_stats", this->node, latch);

  assert(this->modelFwd);
  this->modelFwd->ForwardFrom("/gazebo/"+origWorldName+"/model/info", this->node, latch);

  assert(this->poseFwd);
  this->poseFwd->ForwardFrom("/gazebo/"+origWorldName+"/pose/info", this->node, latch);

  assert(this->guiFwd);
  this->guiFwd->ForwardFrom("/gazebo/"+origWorldName+"/gui", this->node, latch);

  assert(this->jointFwd);
  this->jointFwd->ForwardFrom("/gazebo/"+origWorldName+"/joint", this->node, latch);

  assert(this->contactFwd);
  this->contactFwd->ForwardFrom("/gazebo/"+origWorldName+"/physics/contacts", this->node, latch);

  assert(this->visualFwd);
  this->visualFwd->ForwardFrom("/gazebo/"+origWorldName+"/visual", this->node, latch);

  assert(this->roadFwd);
  this->roadFwd->ForwardFrom("/gazebo/"+origWorldName+"/roads", this->node, latch);

  assert(this->poseAnimFwd);
  this->poseAnimFwd->ForwardFrom("/gazebo/"+origWorldName+"/skeleton_pose/info", this->node, latch);
}

void GazeboTopicForwardingMirror::DisconnectFromOriginal()
{
  assert(this->origServiceFwd);
  this->origServiceFwd->Disconnect();

  assert(this->statFwd);
  this->statFwd->DisconnectSubscriber();

  assert(this->modelFwd);
  this->modelFwd->DisconnectSubscriber();

  assert(this->poseFwd);
  this->poseFwd->DisconnectSubscriber();

  assert(this->guiFwd);
  this->guiFwd->DisconnectSubscriber();

  assert(this->jointFwd);
  this->jointFwd->DisconnectSubscriber();

  assert(this->contactFwd);
  this->contactFwd->DisconnectSubscriber();

  assert(this->visualFwd);
  this->visualFwd->DisconnectSubscriber();

  assert(this->roadFwd);
  this->roadFwd->DisconnectSubscriber();

  assert(this->poseAnimFwd);
  this->poseAnimFwd->DisconnectSubscriber();
}


void GazeboTopicForwardingMirror::Init()
{
  // initialize node
  this->node = gazebo::transport::NodePtr(new gazebo::transport::Node());
  this->node->Init(this->worldName);

  // initialize topic block printers (for user information printing)
  ////////////////////////////////////////////////

  GazeboTopicBlockPrinterInterface::Ptr physicsBlock
    (new GazeboTopicBlockPrinter<gazebo::msgs::Physics>("~/physics",
                                                        this->node));
  this->blockPrinters.push_back(physicsBlock);

  GazeboTopicBlockPrinterInterface::Ptr lightModBlock
    (new GazeboTopicBlockPrinter<gazebo::msgs::Light>("~/light/modify",
                                                        this->node));
  this->blockPrinters.push_back(lightModBlock);

  GazeboTopicBlockPrinterInterface::Ptr atmosphereBlock
    (new GazeboTopicBlockPrinter<gazebo::msgs::Atmosphere>("~/atmosphere",
                                                        this->node));
  this->blockPrinters.push_back(atmosphereBlock);

  GazeboTopicBlockPrinterInterface::Ptr factoryBlock
    (new GazeboTopicBlockPrinter<gazebo::msgs::Factory>("~/factory",
                                                        this->node));
  this->blockPrinters.push_back(factoryBlock);

  GazeboTopicBlockPrinterInterface::Ptr factoryLightBlock
    (new GazeboTopicBlockPrinter<gazebo::msgs::Light>("~/factory/light",
                                                        this->node));
  this->blockPrinters.push_back(factoryLightBlock);

  GazeboTopicBlockPrinterInterface::Ptr logControlBlock
    (new GazeboTopicBlockPrinter<gazebo::msgs::LogControl>("~/log/control",
                                                        this->node));
  this->blockPrinters.push_back(logControlBlock);

  GazeboTopicBlockPrinterInterface::Ptr logStatusBlock
    (new GazeboTopicBlockPrinter<gazebo::msgs::LogStatus>("~/log/status",
                                                        this->node));
  this->blockPrinters.push_back(logStatusBlock);

  GazeboTopicBlockPrinterInterface::Ptr logPlaybackCtrlBlock
    (new GazeboTopicBlockPrinter<gazebo::msgs::LogPlaybackControl>("~/playback_control",
                                                        this->node));
  this->blockPrinters.push_back(logPlaybackCtrlBlock);


  GazeboTopicBlockPrinterInterface::Ptr modelModifyBlock
    (new GazeboTopicBlockPrinter<gazebo::msgs::Model>("~/model/modify",
                                                        this->node));
  this->blockPrinters.push_back(modelModifyBlock);

  GazeboTopicBlockPrinterInterface::Ptr poseModifyBlock
    (new GazeboTopicBlockPrinter<gazebo::msgs::Pose>("~/pose/modify",
                                                        this->node));
  this->blockPrinters.push_back(poseModifyBlock);

  GazeboTopicBlockPrinterInterface::Ptr undoRedoBlock
    (new GazeboTopicBlockPrinter<gazebo::msgs::UndoRedo>("~/undo_redo",
                                                        this->node));
  this->blockPrinters.push_back(undoRedoBlock);

  GazeboTopicBlockPrinterInterface::Ptr userCmdBlock
    (new GazeboTopicBlockPrinter<gazebo::msgs::UserCmd>("~/user_cmd",
                                                        this->node));
  this->blockPrinters.push_back(userCmdBlock);

  GazeboTopicBlockPrinterInterface::Ptr windBlock
    (new GazeboTopicBlockPrinter<gazebo::msgs::Wind>("~/wind",
                                                        this->node));
  this->blockPrinters.push_back(windBlock);

  GazeboTopicBlockPrinterInterface::Ptr worldControlBlock
    (new GazeboTopicBlockPrinter<gazebo::msgs::Wind>("~/world_control",
                                                        this->node));
  this->blockPrinters.push_back(worldControlBlock);


  // initialize services
  ////////////////////////////////////////////////
  this->origServiceFwd.reset(new GazeboServiceForwarder
                             ("~/request", "~/response",
                              RequestMessageFilter::Ptr(new RequestMessageFilter()),
                              1000, 0));

  // initialize topic forwarders
  ////////////////////////////////////////////////

  this->statFwd.reset(new GazeboTopicForwarder<gazebo::msgs::WorldStatistics>
                      ("~/world_stats", this->node));

  this->modelFwd.reset(new GazeboTopicForwarder<gazebo::msgs::Model>
                      ("~/model/info", this->node, 1000, 0, nullptr, true));

  this->poseFwd.reset(new GazeboTopicForwarder<gazebo::msgs::PosesStamped>
                      ("~/pose/info", this->node));

  this->guiFwd.reset(new GazeboTopicForwarder<gazebo::msgs::GUI>
                      ("~/gui", this->node));

  this->jointFwd.reset(new GazeboTopicForwarder<gazebo::msgs::Joint>
                      ("~/joint", this->node));

  this->contactFwd.reset(new GazeboTopicForwarder<gazebo::msgs::Contacts>
                      ("~/physics/contacts", this->node));

  this->visualFwd.reset(new GazeboTopicForwarder<gazebo::msgs::Visual>
                      ("~/visual", this->node));

  this->roadFwd.reset(new GazeboTopicForwarder<gazebo::msgs::Road>
                      ("~/roads", this->node));

  this->poseAnimFwd.reset(new GazeboTopicForwarder<gazebo::msgs::PoseAnimation>
                      ("~/skeleton_pose/info", this->node));

  // create the helper publishers
  ////////////////////////////////////////////////
  this->requestPub = this->node->Advertise<gazebo::msgs::Request>("~/request");
  this->modelPub = this->node->Advertise<gazebo::msgs::Model>("~/model/info");
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
    // if we were already connected to a world, we need to delete all
    // the old world's models and insert all the new world's models.
    // This can be optimized by deleting only the models/lights which only exist
    // in the old world, and update only the pose of the models/lights which
    // are in both worlds. This optimization may be added at a later point.
    DisconnectFromOriginal();
    GazeboPhysicsEngineWorld::Ptr gzOldWorld =
      std::dynamic_pointer_cast<GazeboPhysicsEngineWorld>(oldWorld);
    if (!gzOldWorld)
    {
      THROW_EXCEPTION("Only Gazebo original worlds supported");
    }

    // delete all old models
    // std::cout<<"Deleting all old world's models"<<std::endl;
    gazebo::physics::Model_V oldModels = gzOldWorld->GetWorld()->Models();
    for (gazebo::physics::Model_V::iterator it = oldModels.begin();
         it != oldModels.end(); ++it)
    {
      gazebo::physics::ModelPtr m=*it;
      // std::cout<<"Requesting delete of "<<m->GetScopedName()<<std::endl;
      auto msg = gazebo::msgs::CreateRequest("entity_delete", m->GetScopedName());
      this->requestPub->Publish(*msg, true);
      delete msg;
    }

    // insert all new models
    // std::cout<<"Inserting all new world's models"<<std::endl;
    gazebo::physics::Model_V newModels = gzNewWorld->GetWorld()->Models();
    for (gazebo::physics::Model_V::iterator it = newModels.begin();
         it != newModels.end(); ++it)
    {
      gazebo::physics::ModelPtr m=*it;
      // std::cout<<"Triggering insertion of "<<m->GetScopedName()<<std::endl;
      gazebo::msgs::Model insModelMsg;
      m->FillMsg(insModelMsg);
      this->modelPub->Publish(insModelMsg);
    }
  }

  std::cout<<"Connecting the new world "<<_newWorld->GetName()<<std::endl;
  ConnectOriginalWorld(_newWorld->GetName());
}

void GazeboTopicForwardingMirror::Sync()
{
}
