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

#include <gazebo/transport/TopicManager.hh>
#include <gazebo/transport/TransportIface.hh>
#include <list>
#include <sstream>

using collision_benchmark::GazeboTopicForwarder;
using collision_benchmark::GazeboTopicForwardingMirror;

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

  public: virtual boost::shared_ptr<gazebo::msgs::Request const> Filter
          (const boost::shared_ptr<gazebo::msgs::Request const> &_msg) const
          {
              if (_msg->request() == "entity_delete")
              {
                // std::cout << "Entity deletion not forwarded, "
                //           << "handled privately!"<<std::endl;
                return nullptr;
              }
              // std::cout<<"Got a request: "<<_msg->request()<<std::endl;
              return _msg;
          }
  public: static ConstPtr Instance()
          {
            if (!singleton) singleton.reset(new RequestMessageFilter());
            return singleton;
          }

  private: static RequestMessageFilter::Ptr singleton;
};

RequestMessageFilter::Ptr RequestMessageFilter::singleton;

/**
 * \brief Strategy pattern to modify world statistics messages
 * The paused state has to be modified to the paused state given
 * by PhysicsWorld.
 * \author Jennifer Buehler
 * \date February 2017
 */
class WorldStatMsgFilter:
  public collision_benchmark::MessageFilter<gazebo::msgs::WorldStatistics>
{
  private: typedef  WorldStatMsgFilter Self;
  public: typedef std::shared_ptr<Self> Ptr;
  public: typedef std::shared_ptr<const Self> ConstPtr;

  public: typedef std::shared_ptr<GazeboTopicForwardingMirror> MirrorPtr;
  public: typedef std::weak_ptr<GazeboTopicForwardingMirror> MirrorWeakPtr;

  public: typedef gazebo::msgs::WorldStatistics
                    WorldStatistics;
  public: typedef boost::shared_ptr<WorldStatistics>
                    WorldStatisticsPtr;
  public: typedef boost::shared_ptr<WorldStatistics const>
                    WorldStatisticsConstPtr;

  public: WorldStatMsgFilter(const MirrorWeakPtr &_mirror):
          mirror(_mirror) {}
  public: virtual WorldStatisticsConstPtr
                  Filter(const WorldStatisticsConstPtr &_msg) const
          {
            GZ_ASSERT(_msg, "Message must not be NULL");
            MirrorPtr mirrorPtr = this->mirror.lock();
            if (!mirrorPtr)
            {
              // mirror world has been deleted so filter out message
              std::cout<<"DEBUG ERROR: No mirror world set!";
              return nullptr;
            }
            if (!mirrorPtr->GetOriginalWorld())
            {
              std::cout<<"DEBUG: Mirror world must have original world, ";
              std::cout<<"this could happen when messages arrive during ";
              std::cout<<"initializaion process."<<std::endl;
              return nullptr;
            }
            WorldStatisticsPtr msgCopy(new WorldStatistics(*_msg));
            msgCopy->set_paused(mirrorPtr->GetOriginalWorld()->IsPaused());
            return msgCopy;
          }
  private: MirrorWeakPtr mirror;
};

GazeboTopicForwardingMirror::GazeboTopicForwardingMirror
    (const std::string& worldname):
      worldName(worldname),
      initialized(false)
{
  // register the topic namespace first off, in order to allow gzclient
  // to connect to it. This should be done before Init(), which can only
  // be called once a shared pointer to this object exists.
  // But Init() may be called later.
  RegisterNamespace(this->worldName);

  // initialize node
  this->node = gazebo::transport::NodePtr(new gazebo::transport::Node());
  this->node->Init(this->worldName);

  // initialize service forwarder (required before Init() so that
  // we can buffer incoming requests before call of Init())
  this->origServiceFwd.reset(new GazeboServiceForwarder
                             ("~/request", "~/response",
                              1000, 0));


  this->origServiceFwd->BufferRequests("~/request", this->node, true);
}

void GazeboTopicForwardingMirror::RegisterNamespace
      (const std::string& wldName) const
{
  std::cout << "Registering gazebo namespace "
            << wldName << " for mirror world."<<std::endl;

  std::list<std::string> topicNames;
  gazebo::transport::TopicManager::Instance()->GetTopicNamespaces(topicNames);
  if (!topicNames.empty())
  {
    std::stringstream str;
    str<<"Topics already have been registered before mirror world, which";
    str<<"means gzclient is not going to connect with mirror world!"<<std::endl;
    gzwarn<<str.str();
  }
  std::cout<<"Registering mirror world name "
           <<wldName<<" as topic"<<std::endl;
  gazebo::transport::TopicManager::Instance()->RegisterTopicNamespace(wldName);

  // Wait for namespaces to make sure the mirror world name has arrived.
  gazebo::common::Time waitTime(1, 0);
  int waitCount = 0;
  int maxWaitCount = 10;
  while (!gazebo::transport::waitForNamespaces(waitTime) &&
      (waitCount++) < maxWaitCount)
  {
    gzwarn << "Waited " << waitTime.Double() << "seconds for namespaces.\n";
  }
  if (waitCount >= maxWaitCount)
  {
    THROW_EXCEPTION("Waited " << (waitTime * waitCount).Double()
      << " seconds for namespaces. Giving up.");
  }

  // Make sure the mirror world was in fact the first to arrive
  topicNames.clear();
  gazebo::transport::TopicManager::Instance()->GetTopicNamespaces(topicNames);
  if (topicNames.empty())
    THROW_EXCEPTION("There must have been at least one "
                    << "topic namespace recevied");

  if (topicNames.front() != wldName)
  {
    std::stringstream str;
    str<<"Topic "<<topicNames.front()<<" already has been registered ";
    str<<"before mirror world "<<wldName<<", which means gzclient ";
    str<<"is not going to connect with mirror world!"<<std::endl;
    gzwarn<<str.str();
  }
}

GazeboTopicForwardingMirror::~GazeboTopicForwardingMirror()
{
}



void GazeboTopicForwardingMirror::Init()
{
  std::cout<<"Initializing GazeboTopicForwardingMirror."<<std::endl;

  // initialize topic block printers (for user information printing)
  ////////////////////////////////////////////////
  bool verbose=true;  // later replace by global static variable or so
  if (verbose)
  {
    static const std::string printPrefix="GazeboTopicForwardingMirror";

    GazeboTopicBlockPrinterInterface::Ptr physicsBlock
      (new GazeboTopicBlockPrinter<gazebo::msgs::Physics>
        (printPrefix, "~/physics", this->node));
    this->blockPrinters.push_back(physicsBlock);

    GazeboTopicBlockPrinterInterface::Ptr lightModBlock
      (new GazeboTopicBlockPrinter<gazebo::msgs::Light>(printPrefix,
            "~/light/modify", this->node));
    this->blockPrinters.push_back(lightModBlock);

    GazeboTopicBlockPrinterInterface::Ptr atmosphereBlock
      (new GazeboTopicBlockPrinter<gazebo::msgs::Atmosphere>(printPrefix,
          "~/atmosphere", this->node));
    this->blockPrinters.push_back(atmosphereBlock);

    GazeboTopicBlockPrinterInterface::Ptr factoryBlock
      (new GazeboTopicBlockPrinter<gazebo::msgs::Factory>(printPrefix,
            "~/factory", this->node));
    this->blockPrinters.push_back(factoryBlock);

    GazeboTopicBlockPrinterInterface::Ptr factoryLightBlock
      (new GazeboTopicBlockPrinter<gazebo::msgs::Light>(printPrefix,
            "~/factory/light", this->node));
    this->blockPrinters.push_back(factoryLightBlock);

    GazeboTopicBlockPrinterInterface::Ptr logControlBlock
      (new GazeboTopicBlockPrinter<gazebo::msgs::LogControl>(printPrefix,
            "~/log/control", this->node));
    this->blockPrinters.push_back(logControlBlock);

    GazeboTopicBlockPrinterInterface::Ptr logStatusBlock
      (new GazeboTopicBlockPrinter<gazebo::msgs::LogStatus>(printPrefix,
            "~/log/status", this->node));
    this->blockPrinters.push_back(logStatusBlock);

    GazeboTopicBlockPrinterInterface::Ptr logPlaybackCtrlBlock
      (new GazeboTopicBlockPrinter<gazebo::msgs::LogPlaybackControl>
        (printPrefix, "~/playback_control", this->node));
    this->blockPrinters.push_back(logPlaybackCtrlBlock);


    GazeboTopicBlockPrinterInterface::Ptr modelModifyBlock
      (new GazeboTopicBlockPrinter<gazebo::msgs::Model>(printPrefix,
          "~/model/modify", this->node));
    this->blockPrinters.push_back(modelModifyBlock);

    GazeboTopicBlockPrinterInterface::Ptr poseModifyBlock
      (new GazeboTopicBlockPrinter<gazebo::msgs::Pose>(printPrefix,
          "~/pose/modify", this->node));
    this->blockPrinters.push_back(poseModifyBlock);

    GazeboTopicBlockPrinterInterface::Ptr undoRedoBlock
      (new GazeboTopicBlockPrinter<gazebo::msgs::UndoRedo>(printPrefix,
            "~/undo_redo", this->node));
    this->blockPrinters.push_back(undoRedoBlock);

    GazeboTopicBlockPrinterInterface::Ptr windBlock
      (new GazeboTopicBlockPrinter<gazebo::msgs::Wind>(printPrefix, "~/wind",
                                                          this->node));
    this->blockPrinters.push_back(windBlock);

    GazeboTopicBlockPrinterInterface::Ptr worldControlBlock
      (new GazeboTopicBlockPrinter<gazebo::msgs::WorldControl>
            (printPrefix, "~/world_control",this->node));
    this->blockPrinters.push_back(worldControlBlock);

    GazeboTopicBlockPrinterInterface::Ptr userCmdBlock
      (new GazeboTopicBlockPrinter<gazebo::msgs::UserCmd>
            (printPrefix, "~/user_cmd",this->node));
    this->blockPrinters.push_back(userCmdBlock);
  }


  // initialize topic forwarders
  ////////////////////////////////////////////////
  int verboseLevel=0;
  try
  {
    this->statFwd.reset(new GazeboTopicForwarder<gazebo::msgs::WorldStatistics>
                        ("~/world_stats", this->node, 1000, 0,
                         WorldStatMsgFilter::Ptr
                          (new WorldStatMsgFilter(shared_from_this())),
                         verboseLevel>2));
  }
  catch(std::bad_weak_ptr& e)
  {
    THROW_EXCEPTION("Can only initialize GazeboTopicForwarder \
                    if there is already a shared pointer for it");
  }

  this->modelFwd.reset(new GazeboTopicForwarder<gazebo::msgs::Model>
                      ("~/model/info", this->node, 1000, 0,
                       nullptr, verboseLevel>0));

  this->poseFwd.reset(new GazeboTopicForwarder<gazebo::msgs::PosesStamped>
                      ("~/pose/info", this->node, 1000, 0,
                       nullptr, verboseLevel>1));

  this->guiFwd.reset(new GazeboTopicForwarder<gazebo::msgs::GUI>
                      ("~/gui", this->node, 1000, 0, nullptr,
                       verboseLevel>0));

  this->jointFwd.reset(new GazeboTopicForwarder<gazebo::msgs::Joint>
                      ("~/joint", this->node, 1000, 0, nullptr,
                       verboseLevel>0));

  this->contactFwd.reset(new GazeboTopicForwarder<gazebo::msgs::Contacts>
                      ("~/physics/contacts", this->node, 1000, 0,
                       nullptr, verboseLevel>2));

  this->visualFwd.reset(new GazeboTopicForwarder<gazebo::msgs::Visual>
                      ("~/visual", this->node, 1000, 0, nullptr,
                       verboseLevel>0));

  this->roadFwd.reset(new GazeboTopicForwarder<gazebo::msgs::Road>
                      ("~/roads", this->node, 1000, 0, nullptr,
                       verboseLevel>0));

  this->poseAnimFwd.reset(new GazeboTopicForwarder<gazebo::msgs::PoseAnimation>
                      ("~/skeleton_pose/info", this->node,
                       1000, 0, nullptr, verboseLevel>0));


  // create the helper publishers
  ////////////////////////////////////////////////
  this->requestPub = this->node->Advertise<gazebo::msgs::Request>("~/request");
  this->modelPub = this->node->Advertise<gazebo::msgs::Model>("~/model/info");
  std::cout<<"GazeboTopicForwardingMirror initialized."<<std::endl;
  this->initialized = true;
}


void GazeboTopicForwardingMirror::ConnectOriginalWorld
      (const std::string origWorldName)
{
  if (!this->initialized) Init();

  std::cout<<"Mirror is connecting to world '"<<origWorldName<<"'"<<std::endl;

  // connect services
  assert(this->origServiceFwd);
  this->origServiceFwd->Forward("/gazebo/"+origWorldName+"/request",
                                "/gazebo/"+origWorldName+"/response",
                                RequestMessageFilter::Instance(),
                                this->node);
  // connect topic forwarders
  bool latch=false;
  assert(this->statFwd);
  this->statFwd->ForwardFrom("/gazebo/"+origWorldName+"/world_stats",
                             this->node, latch);

  assert(this->modelFwd);
  this->modelFwd->ForwardFrom("/gazebo/"+origWorldName+"/model/info",
                              this->node, latch);

  assert(this->poseFwd);
  this->poseFwd->ForwardFrom("/gazebo/"+origWorldName+"/pose/info",
                             this->node, latch);

  assert(this->guiFwd);
  this->guiFwd->ForwardFrom("/gazebo/"+origWorldName+"/gui",
                            this->node, latch);

  assert(this->jointFwd);
  this->jointFwd->ForwardFrom("/gazebo/"+origWorldName+"/joint",
                              this->node, latch);

  assert(this->contactFwd);
  this->contactFwd->ForwardFrom("/gazebo/"+origWorldName+"/physics/contacts",
                                this->node, latch);

  assert(this->visualFwd);
  this->visualFwd->ForwardFrom("/gazebo/"+origWorldName+"/visual",
                               this->node, latch);

  assert(this->roadFwd);
  this->roadFwd->ForwardFrom("/gazebo/"+origWorldName+"/roads",
                             this->node, latch);

  assert(this->poseAnimFwd);
  this->poseAnimFwd->ForwardFrom("/gazebo/" +origWorldName +
                                 "/skeleton_pose/info", this->node, latch);

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



void GazeboTopicForwardingMirror::NotifyOriginalWorldChange
          (const OriginalWorldPtr &_newWorld)
{
  // std::cout<<"GazeboTopicForwardingMirror::NotifyOriginalWorldChange"<<std::endl;

  if (!this->initialized) Init();

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
      auto msg = gazebo::msgs::CreateRequest("entity_delete",
                                             m->GetScopedName());
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
