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
#include <test/MultipleWorldsTestFramework.hh>
#include <collision_benchmark/PhysicsWorldInterfaces.hh>
#include <collision_benchmark/MirrorWorld.hh>
#include <collision_benchmark/BasicTypes.hh>

using collision_benchmark::MirrorWorld;
using collision_benchmark::PhysicsWorldBaseInterface;
using collision_benchmark::BasicState;
using collision_benchmark::Shape;
using collision_benchmark::GazeboMultipleWorlds;

////////////////////////////////////////////////////////////////
void MultipleWorldsTestFramework::SetUp()
{
  server.reset(new GazeboMultipleWorlds());
}

////////////////////////////////////////////////////////////////
void MultipleWorldsTestFramework::TearDown()
{
  if (server) server->Stop();
}

////////////////////////////////////////////////////////////////
void
MultipleWorldsTestFramework::Init(const bool interactiveMode,
                                const std::vector<std::string> &additionalGuis)
{
  ASSERT_NE(server, nullptr) << "server must have been created";
  bool loadMirror = true;
  bool allowControlViaMirror = false;
  bool enforceContactCalc = true;
  server->Init(loadMirror, enforceContactCalc,
               allowControlViaMirror, interactiveMode, additionalGuis);
  // ensure that the server is returned properly
  GzMultipleWorldsServer::Ptr mServer = GetServer();
  ASSERT_NE(mServer.get(), nullptr) << "Could not create and start server";
}

////////////////////////////////////////////////////////////////
void MultipleWorldsTestFramework::InitMultipleEngines
        (const std::vector<std::string>& engines,
         const bool interactiveMode,
         const std::vector<std::string> &additionalGuis)
{
  Init(interactiveMode, additionalGuis);

  GzMultipleWorldsServer::Ptr mServer = GetServer();
  ASSERT_NE(mServer.get(), nullptr) << "Could not create and start server";
  GzWorldManager::Ptr worldManager = mServer->GetWorldManager();
  ASSERT_NE(worldManager.get(), nullptr) << "No valid world manager created";

  // world to load
  std::string worldfile = "test_worlds/void.world";
  int numWorlds = mServer->Load(worldfile, engines);
  ASSERT_EQ(numWorlds, engines.size()) << "Could not prepare all engines";
}

////////////////////////////////////////////////////////////////
void MultipleWorldsTestFramework::InitOneEngine(const std::string &engine,
                                        const unsigned int numWorlds,
                                        const bool interactiveMode,
                                        const std::vector<std::string>
                                            &additionalGuis)
{
  Init(interactiveMode, additionalGuis);

  GzMultipleWorldsServer::Ptr mServer = GetServer();
  ASSERT_NE(mServer.get(), nullptr) << "Could not create and start server";
  GzWorldManager::Ptr worldManager = mServer->GetWorldManager();
  ASSERT_NE(worldManager.get(), nullptr) << "No valid world manager created";

  int numWorldsInMgr = worldManager->GetNumWorlds();
  ASSERT_EQ(numWorldsInMgr, 0) << "No worlds should be loaded yet.";

  LoadOneEngine(engine, numWorlds);
}

////////////////////////////////////////////////////////////////
void MultipleWorldsTestFramework::LoadOneEngine(const std::string &engine,
                                        const unsigned int numWorlds)
{
  GzMultipleWorldsServer::Ptr mServer = GetServer();
  ASSERT_NE(mServer.get(), nullptr) << "Could not create and start server";
  GzWorldManager::Ptr worldManager = mServer->GetWorldManager();
  ASSERT_NE(worldManager.get(), nullptr) << "No valid world manager created";

  int numWorldsInMgr = worldManager->GetNumWorlds();

  // world to load
  std::string worldfile = "test_worlds/void.world";
  for (int i = 0; i < numWorlds; ++i)
  {
    std::stringstream _worldname;
    _worldname << "world_" << i << "_" << engine;
    std::string worldname = _worldname.str();
    mServer->Load(worldfile, engine, worldname);
  }
  int numWorldsInMgrNew = worldManager->GetNumWorlds();
  ASSERT_EQ(numWorldsInMgrNew, numWorldsInMgr + numWorlds)
    << "Could not load up world " << numWorlds << " times.";
}

////////////////////////////////////////////////////////////////
void MultipleWorldsTestFramework::LoadShape(const Shape::Ptr &shape,
                                    const std::string &modelName)
{
  GzMultipleWorldsServer::Ptr mServer = GetServer();
  ASSERT_NE(mServer.get(), nullptr) << "Could not create and start server";
  GzWorldManager::Ptr worldManager = mServer->GetWorldManager();
  ASSERT_NE(worldManager.get(), nullptr) << "No valid world manager created";

  int numWorlds = worldManager->GetNumWorlds();

  // Load model
  typedef GzWorldManager::ModelLoadResult ModelLoadResult;
  std::vector<ModelLoadResult> res
    = worldManager->AddModelFromShape(modelName, shape, shape);
  ASSERT_EQ(res.size(), numWorlds)
    << "Model must have been loaded in all worlds";

  for (std::vector<ModelLoadResult>::iterator it = res.begin();
       it != res.end(); ++it)
  {
    const ModelLoadResult &mlRes=*it;
    ASSERT_EQ(mlRes.opResult, collision_benchmark::SUCCESS)
      << "Could not load model";
    ASSERT_EQ(mlRes.modelID, modelName)
      << "Model names should be equal";
  }
}

////////////////////////////////////////////////////////////////
void MultipleWorldsTestFramework::LoadShape(const Shape::Ptr &shape,
                                    const std::string &modelName,
                                    const unsigned int worldIdx)
{
  GzMultipleWorldsServer::Ptr mServer = GetServer();
  ASSERT_NE(mServer.get(), nullptr) << "Could not create and start server";
  GzWorldManager::Ptr worldManager = mServer->GetWorldManager();
  ASSERT_NE(worldManager.get(), nullptr) << "No valid world manager created";

  // get the world
  PhysicsWorldBaseInterface::Ptr world = worldManager->GetWorld(worldIdx);
  ASSERT_NE(world.get(), nullptr) << "No world at index " << worldIdx;

  GzWorldManager::PhysicsWorldModelInterfacePtr mWorld =
    GzWorldManager::ToWorldWithModel(world);
  ASSERT_NE(mWorld.get(), nullptr) << "Cast failure";

  // Load model
  typedef GzWorldManager::ModelLoadResult ModelLoadResult;
  ModelLoadResult res =
    mWorld->AddModelFromShape(modelName, shape, shape);

  ASSERT_EQ(res.opResult, collision_benchmark::SUCCESS)
    << "Could not load model";

  ASSERT_EQ(res.modelID, modelName)
    << "Model names should be equal";
}

////////////////////////////////////////////////////////////////
void MultipleWorldsTestFramework::LoadModel(const std::string &modelFile,
                                            const std::string &modelName)
{
  GzMultipleWorldsServer::Ptr mServer = GetServer();
  ASSERT_NE(mServer.get(), nullptr) << "Could not create and start server";
  GzWorldManager::Ptr worldManager = mServer->GetWorldManager();
  ASSERT_NE(worldManager.get(), nullptr) << "No valid world manager created";

  int numWorlds = worldManager->GetNumWorlds();

  // Load model
  typedef GzWorldManager::ModelLoadResult ModelLoadResult;
  std::vector<ModelLoadResult> res
    = worldManager->AddModelFromFile(modelFile, modelName);
  ASSERT_EQ(res.size(), numWorlds)
    << "Model must have been loaded in all worlds";

  for (std::vector<ModelLoadResult>::iterator it = res.begin();
       it != res.end(); ++it)
  {
    const ModelLoadResult &mlRes=*it;
    ASSERT_EQ(mlRes.opResult, collision_benchmark::SUCCESS)
      << "Could not load model";
    ASSERT_EQ(mlRes.modelID, modelName)
      << "Model names should be equal";
  }
}

////////////////////////////////////////////////////////////////
void MultipleWorldsTestFramework::LoadModel(const std::string &modelFile,
                                            const std::string &modelName,
                                            const unsigned int worldIdx)
{
  GzMultipleWorldsServer::Ptr mServer = GetServer();
  ASSERT_NE(mServer.get(), nullptr) << "Could not create and start server";
  GzWorldManager::Ptr worldManager = mServer->GetWorldManager();
  ASSERT_NE(worldManager.get(), nullptr) << "No valid world manager created";

  // get the world
  PhysicsWorldBaseInterface::Ptr world = worldManager->GetWorld(worldIdx);
  ASSERT_NE(world.get(), nullptr) << "No world at index " << worldIdx;

  GzWorldManager::PhysicsWorldModelInterfacePtr mWorld =
    GzWorldManager::ToWorldWithModel(world);
  ASSERT_NE(mWorld.get(), nullptr) << "Cast failure";

  // Load model
  typedef GzWorldManager::ModelLoadResult ModelLoadResult;
  ModelLoadResult res =
    mWorld->AddModelFromFile(modelFile, modelName);

  ASSERT_EQ(res.opResult, collision_benchmark::SUCCESS)
    << "Could not load model";

  ASSERT_EQ(res.modelID, modelName)
    << "Model names should be equal";
}


////////////////////////////////////////////////////////////////
bool MultipleWorldsTestFramework::GetAABBs(const std::string &modelName1,
                                   const std::string &modelName2,
                                   const double bbTol,
                                   collision_benchmark::GzAABB &m1,
                                   collision_benchmark::GzAABB &m2)
{
  GzMultipleWorldsServer::Ptr mServer = GetServer();
  if (!mServer) return false;
  GzWorldManager::Ptr worldManager = mServer->GetWorldManager();
  if (!worldManager) return false;

  return collision_benchmark::GetConsistentAABB(modelName1, worldManager,
                                                bbTol, m1)
         &&
         collision_benchmark::GetConsistentAABB(modelName2, worldManager,
                                                bbTol, m2);
}



////////////////////////////////////////////////////////////////
bool MultipleWorldsTestFramework::RefreshClient(const double timeoutSecs)
{
  GzMultipleWorldsServer::Ptr mServer = GetServer();
  if (!mServer) return false;
  GzWorldManager::Ptr worldManager = mServer->GetWorldManager();
  if (!worldManager) return false;
  MirrorWorld::ConstPtr mirrorWorld = worldManager->GetMirrorWorld();
  if (!mirrorWorld) return false;
  std::string mirrorName = mirrorWorld->GetName();
  std::cout << "Refreshing client with mirror " << mirrorName << std::endl;

  // initialize node
  if (!node)
  {
    node.reset(new gazebo::transport::Node());
    node->Init(mirrorName);
  }

#if 0
  // This approach will send a msgs::WorldModify::create to
  // the "/gazebo/world/modify"
  // topic, using the mirror world name.
  // This should cause the clients to refresh
  // their state to the most current one.
  // PROBLEM: We would first need to delete the scene by sending out a
  // msgs::WorldModify::delete message (which currently segfaults with
  // gzclient), otherwise the scene does
  // not get re-created completely in rendering::create_scene().
  // If no new scene is created, then rendering::Scene::Init() is not
  // called, and this is needed to send out a "scene_info" request for the
  // scene to get updated information. This is necessary for a call to
  // rendering::Scene::OnResponse() to be triggered to add a scene
  // message (with updated model poses etc) to be processed.
  // An alternative would be to send a scene message to "~/scene",
  // and it will then arrive to rendering::Scene::OnScene() and be processed
  // as well. However in this case we would need to get *all* the scene
  // information of the world, pack it in a scene message, and publish.
  // This is a bit of overkill given that the model pose itself only is the
  // problem (the only one being throttled. So why not just publish the
  // current model poses manually.

  gazebo::transport::PublisherPtr pub =
    node->Advertise<gazebo::msgs::WorldModify>("/gazebo/world/modify");

  if (!pub)
  {
    std::cerr << "Could not create publisher" << std::endl;
    return false;
  }

  gazebo::common::Time timeout(timeoutSecs);
  pub->WaitForConnection(timeout);

  gazebo::msgs::WorldModify worldMsg;
  worldMsg.set_world_name(mirrorName);
//  worldMsg.set_remove(true);
//  pub->Publish(worldMsg, true);
//  worldMsg.set_removed(false);
  worldMsg.set_create(true);

  // block until the message has been sent out
  pub->Publish(worldMsg, true);
  // update worlds with one step to make sure the request is processed
  worldManager->Update(1);
#else
  // simply get all model poses and re-publish.
  // This will only work for the models which are children of the
  // world directly (no nested models and link poses), but for the test case
  // this should be sufficient because there won't be nested models, or if there
  // are, it is only important that the parent model pose will be updated.
  // Reason why it won't work for nested models: Pose messages are always
  // published with the relative pose to the parent. We can make it work
  // by casting the world to GazeboPhysicsWorld and then publish the pose
  // as in physics::World::ProcessMessages(), but for now we want to avoid
  // casting - we will stick to the PhysicsWorld interface only.

  if (!pub) pub = node->Advertise<gazebo::msgs::PosesStamped>
                          ("/gazebo/"+mirrorName+"/pose/info");
  if (!pub)
  {
    std::cerr << "Could not create publisher" << std::endl;
    return false;
  }

  gazebo::common::Time timeout(timeoutSecs);
  pub->WaitForConnection(timeout);

  // Take the mirror worlds original world - this should have the
  // pose information which we want to forward to the client.
  PhysicsWorldBaseInterface::Ptr origWorld = mirrorWorld->GetOriginalWorld();
  GzWorldManager::PhysicsWorldPtr world =
    GzWorldManager::ToPhysicsWorld(origWorld);
  if (!world)
  {
    std::cerr << "No mirror world loaded" << std::endl;
    return false;
  }

  // a bit of a cumbersome way to access the world time is to get the
  // world state - could consider putting this into the PhysicsWorld interface
  // instead.
  gazebo::physics::WorldState worldState = world->GetWorldState();
  // get all models and add their poses
  std::vector<std::string> allModels = world->GetAllModelIDs();
  if (allModels.empty())
  {
    std::cout << "DEBUG: NO MODELS TO UPDATE" << __FILE__ <<std::endl;
    return true;
  }
  for (std::vector<std::string>::iterator it = allModels.begin();
       it != allModels.end(); ++it)
  {
    // message of all poses to be published
    gazebo::msgs::PosesStamped msg;
    gazebo::msgs::Set(msg.mutable_time(), worldState.GetSimTime());

    BasicState state;
    if (!world->GetBasicModelState(*it, state))
    {
      std::cerr << "Could not get basic model state for " << *it << std::endl;
      continue;
    }
    int intID = world->GetIntegerModelID(*it);
    if (intID < 0)
    {
      std::cerr << "Negative model ID: Pose update won't work." << std::endl;
      continue;
    }
    gazebo::msgs::Pose * poseMsg = msg.add_pose();
    poseMsg->set_name(*it);
    poseMsg->set_id(intID);
    const ignition::math::Pose3d ignPose(state.position.x, state.position.y,
                                         state.position.z, state.rotation.w,
                                         state.rotation.x, state.rotation.y,
                                         state.rotation.z);

    gazebo::msgs::Set(poseMsg, ignPose);
    // publish the pose and block until the message has been written out.
    // std::cout << "Publish " << msg.DebugString() << std::endl;
    pub->Publish(msg, true);
  }
#endif
  // if we don't call SendMessage() until pub->GetOutgoingCount is 0,
  // any left-over messages which could not be sent immediately
  // will remain in the message queue in transport::Publisher and won't
  // arrive at client.
  while (pub->GetOutgoingCount() > 0)
  {
    std::cout << "Getting out last messages, got "
      <<pub->GetOutgoingCount() << " left." << std::endl;
     pub->SendMessage();
    gazebo::common::Time::MSleep(200);
  }

  return true;
}
