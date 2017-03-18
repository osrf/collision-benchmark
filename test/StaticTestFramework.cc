#include <test/StaticTestFramework.hh>
#include <collision_benchmark/PrimitiveShape.hh>
#include <collision_benchmark/SimpleTriMeshShape.hh>
#include <collision_benchmark/BasicTypes.hh>
#include <collision_benchmark/MirrorWorld.hh>

#include <ignition/math/Vector3.hh>

#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>

#include "MultipleWorldsTestFramework.hh"

#include <sstream>
#include <thread>
#include <atomic>

using collision_benchmark::Shape;
using collision_benchmark::BasicState;
using collision_benchmark::Vector3;
using collision_benchmark::Quaternion;
using collision_benchmark::PhysicsWorldBaseInterface;
using collision_benchmark::MirrorWorld;

std::atomic<bool> g_keypressed(false);

// waits until Enter has been pressed and sets g_keypressed to true
void WaitForEnterImpl()
{
  int key = getchar();
  g_keypressed=true;
}

// waits until Enter key was pressed and
// meanwhile updates the worlds in the world manager
void WaitForEnter(StaticTestFramework::GzWorldManager::Ptr& worlds)
{
  g_keypressed = false;
  std::thread * t = new std::thread(WaitForEnterImpl);
  t->detach();  // detach so it can be terminated
  while (!g_keypressed)
  {
    worlds->Update(1);
  }
  delete t;
}

////////////////////////////////////////////////////////////////
template<typename T>
std::string VectorToString(const std::vector<T>& v)
{
  std::stringstream str;
  str << "[";
  for (typename std::vector<T>::const_iterator it = v.begin();
       it != v.end(); ++it)
  {
    if (it != v.begin()) str << ", ";
    str<<*it;
  }
  str << "]";
  return str.str();
}

////////////////////////////////////////////////////////////////
template<typename T>
std::string VectorPtrToString(const std::vector<T>& v)
{
  std::stringstream str;
  str << "[";
  for (typename std::vector<T>::const_iterator it = v.begin();
       it != v.end(); ++it)
  {
    if (it != v.begin()) str << ", ";
    str<<**it;
  }
  str << "]";
  return str.str();
}

////////////////////////////////////////////////////////////////
void StaticTestFramework::PrepareWorld(const std::vector<std::string>& engines)
{
  // world to load
  std::string worldfile = "test_worlds/void.world";

  GzMultipleWorldsServer::Ptr mServer = GetServer();
  ASSERT_NE(mServer.get(), nullptr) << "Could not create and start server";

  bool loadMirror = true;
  std::string mirrorName = "";
  if (loadMirror) mirrorName = "mirror";
  // with the tests, the mirror can be used to watch the test,
  // but not to manipulate the worlds.
  bool allowControlViaMirror = false;
  mServer->Init(mirrorName, allowControlViaMirror);
  int numWorlds = mServer->Load(worldfile, engines);

  GzWorldManager::Ptr worldManager = mServer->GetWorldManager();
  ASSERT_NE(worldManager.get(), nullptr) << "No valid world manager created";

  ASSERT_EQ(numWorlds, engines.size()) << "Could not prepare all engines";

/*  GzWorldManager::ControlServerPtr controlServer =
    worldManager->GetControlServer();

  if (controlServer)
  {
    controlServer->RegisterPauseCallback(std::bind(pauseCallback,
                                                   std::placeholders::_1));
  }*/
}

////////////////////////////////////////////////////////////////
void StaticTestFramework::LoadShapes(const Shape::Ptr& shape1,
                                    const std::string& modelName1,
                                    const Shape::Ptr& shape2,
                                    const std::string& modelName2)
{
  GzMultipleWorldsServer::Ptr mServer = GetServer();
  ASSERT_NE(mServer.get(), nullptr) << "Could not create and start server";
  GzWorldManager::Ptr worldManager = mServer->GetWorldManager();
  ASSERT_NE(worldManager.get(), nullptr) << "No valid world manager created";

  int numWorlds = worldManager->GetNumWorlds();

  // Load model 1
  typedef GzWorldManager::ModelLoadResult ModelLoadResult;
  std::vector<ModelLoadResult> res1
    = worldManager->AddModelFromShape(modelName1, shape1, shape1);
  ASSERT_EQ(res1.size(), numWorlds)
    << "Model must have been loaded in all worlds";
  for (std::vector<ModelLoadResult>::iterator it = res1.begin();
       it != res1.end(); ++it)
  {
    const ModelLoadResult& mlRes=*it;
    ASSERT_EQ(mlRes.opResult, collision_benchmark::SUCCESS)
      << "Could not load model";
    ASSERT_EQ(mlRes.modelID, modelName1)
      << "Model names should be equal";
  }

  // Load model 2
  std::vector<ModelLoadResult> res2
    = worldManager->AddModelFromShape(modelName2, shape2, shape2);
  ASSERT_EQ(res2.size(), numWorlds)
    << "Model must have been loaded in all worlds";
  for (std::vector<ModelLoadResult>::iterator it = res2.begin();
       it != res2.end(); ++it)
  {
    const ModelLoadResult& mlRes=*it;
    ASSERT_EQ(mlRes.opResult, collision_benchmark::SUCCESS)
      << "Could not load model";
    ASSERT_EQ(mlRes.modelID, modelName2)
      << "Model names should be equal";
  }
}

////////////////////////////////////////////////////////////////
bool StaticTestFramework::GetAABBs(const std::string& modelName1,
                                   const std::string& modelName2,
                                   AABB& m1, AABB& m2)
{
  GzMultipleWorldsServer::Ptr mServer = GetServer();
  if (!mServer) return false;
  GzWorldManager::Ptr worldManager = mServer->GetWorldManager();
  if (!worldManager) return false;

  std::vector<GzWorldManager::PhysicsWorldModelInterfacePtr>
    worlds = worldManager->GetModelPhysicsWorlds();

  // AABB's from all worlds: need to be equal or this function
  // must return false.
  std::vector<AABB> aabbs1, aabbs2;

  std::vector<GzWorldManager::PhysicsWorldModelInterfacePtr>::iterator it;
  for (it = worlds.begin(); it != worlds.end(); ++it)
  {
    GzWorldManager::PhysicsWorldModelInterfacePtr w = *it;
    AABB aabb1, aabb2;
    bool ret1 = w->GetAABB(modelName1, aabb1.min, aabb1.max);
    bool ret2 = w->GetAABB(modelName2, aabb2.min, aabb2.max);
    if (!ret1 || !ret2)
    {
      std::cerr<<"Model 1 or model 2 not found"<<std::endl;
      return false;
    }
    aabbs1.push_back(aabb1);
    aabbs2.push_back(aabb2);
  }

  if (aabbs1.size() != aabbs2.size())
  {
    std::cerr << "AABB numbers for both worlds "
              << "should be the same" << std::endl;
    return false;
  }

  if (aabbs1.empty())
  {
    std::cerr << "At least one bounding box should "
              << "have been returned" << std::endl;
    return false;
  }

  // epsilon for vector comparison
  const static float eps = 5e-02;

  // Check that all AABBs in aabb1 are the same
  std::vector<AABB>::iterator itAABB;
  AABB lastAABB;
  for (itAABB = aabbs1.begin();  itAABB != aabbs1.end(); ++itAABB)
  {
    const AABB& aabb = *itAABB;
    if (itAABB != aabbs1.begin())
    {
      if (!aabb.min.Equal(lastAABB.min, eps) ||
          !aabb.max.Equal(lastAABB.max, eps))
      {
        std::cerr << "Bounding boxes should be of the same size: "
                  <<  aabb.min << ", " << aabb.max << " --- "
                  <<  lastAABB.min << ", " << lastAABB.max;
        return false;
      }
    }
    lastAABB = aabb;
  }
  // Check that all AABBs in aabb2 are the same
  for (itAABB = aabbs2.begin();  itAABB != aabbs2.end(); ++itAABB)
  {
    const AABB& aabb = *itAABB;
    if (itAABB != aabbs2.begin())
    {
      if (!aabb.min.Equal(lastAABB.min, eps) ||
          !aabb.max.Equal(lastAABB.max, eps))
      {
        std::cerr << "Bounding boxes should be of the same size: "
                  <<  aabb.min << ", " << aabb.max << " --- "
                  <<  lastAABB.min << ", " << lastAABB.max;
        return false;
      }
    }
    lastAABB = aabb;
  }

  m1 = aabbs1.front();
  m2 = aabbs2.front();
  return true;
}


////////////////////////////////////////////////////////////////
std::vector<StaticTestFramework::ContactInfoPtr>
StaticTestFramework::GetContactInfo(const std::string& modelName1,
                                    const std::string& modelName2,
                                    const std::string& worldName)
{
  std::vector<ContactInfoPtr> ret;
  GzMultipleWorldsServer::Ptr mServer = GetServer();
  assert(mServer);
  GzWorldManager::Ptr worldManager = mServer->GetWorldManager();
  assert(worldManager);

  PhysicsWorldBaseInterface::Ptr w = worldManager->GetWorld(worldName);
  assert(w);

  GzWorldManager::PhysicsWorldPtr pWorld = worldManager->ToPhysicsWorld(w);
  assert(pWorld);

  return pWorld->GetContactInfo(modelName1, modelName2);
}


////////////////////////////////////////////////////////////////
bool StaticTestFramework::CollisionState(const std::string& modelName1,
                                         const std::string& modelName2,
                                         std::vector<std::string>& colliding,
                                         std::vector<std::string>& notColliding,
                                         double& maxNegDepth)
{
  colliding.clear();
  notColliding.clear();
  maxNegDepth = 0;
  GzMultipleWorldsServer::Ptr mServer = GetServer();
  if (!mServer) return false;
  GzWorldManager::Ptr worldManager = mServer->GetWorldManager();
  if (!worldManager) return false;

  std::vector<GzWorldManager::PhysicsWorldPtr>
    worlds = worldManager->GetPhysicsWorlds();

  std::vector<GzWorldManager::PhysicsWorldPtr>::iterator it;
  for (it = worlds.begin(); it != worlds.end(); ++it)
  {
    GzWorldManager::PhysicsWorldPtr w = *it;
    if (!w->SupportsContacts())
    {
      std::cout<<"A world does not support contact calculation"<<std::endl;
      return false;
    }

    std::vector<ContactInfoPtr> contacts =
      w->GetContactInfo(modelName1, modelName2);
    if (!contacts.empty())
    {
      colliding.push_back(w->GetName());
      for (typename std::vector<ContactInfoPtr>::const_iterator
           cit = contacts.begin(); cit != contacts.end(); ++cit)
      {
        ContactInfoPtr c = *cit;
        if (c->minDepth() < maxNegDepth) maxNegDepth = c->minDepth();
      }
    }
    else
    {
      notColliding.push_back(w->GetName());
    }
  }
  return true;
}

////////////////////////////////////////////////////////////////
bool StaticTestFramework::RefreshClient(const double timeoutSecs)
{
  GzMultipleWorldsServer::Ptr mServer = GetServer();
  if (!mServer) return false;
  GzWorldManager::Ptr worldManager = mServer->GetWorldManager();
  if (!worldManager) return false;
  MirrorWorld::ConstPtr mirrorWorld = worldManager->GetMirrorWorld();
  if (!mirrorWorld) return false;
  std::string mirrorName = mirrorWorld->GetName();
  std::cout<<"Refreshing client with mirror "<<mirrorName<<std::endl;

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
    std::cout<<"DEBUG: NO MODELS TO UPDATE" << __FILE__ <<std::endl;
    return true;
  }
  for (std::vector<std::string>::iterator it = allModels.begin();
       it != allModels.end(); ++it)
  {
    // message of all poses to be published
    gazebo::msgs::PosesStamped msg;
    gazebo::msgs::Set(msg.mutable_time(), worldState.GetSimTime());

    BasicState state;
    if (!world->GetBasicModelState(*it,state))
    {
      std::cerr<<"Could not get basic model state for " << *it << std::endl;
      continue;
    }
    int intID = world->GetIntegerModelID(*it);
    if (intID < 0)
    {
      std::cerr<<"Negative model ID: Pose update won't work."<<std::endl;
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
    // std::cout<<"Publish "<<msg.DebugString()<<std::endl;
    pub->Publish(msg, true);
  }
#endif
  // if we don't call SendMessage() until pub->GetOutgoingCount is 0,
  // any left-over messages which could not be sent immediately
  // will remain in the message queue in transport::Publisher and won't
  // arrive at client.
  while (pub->GetOutgoingCount() > 0)
  {
    std::cout<<"Getting out last messages, got "
      <<pub->GetOutgoingCount() << " left." << std::endl;
     pub->SendMessage();
    gazebo::common::Time::MSleep(200);
  }

  return true;
}


////////////////////////////////////////////////////////////////
void StaticTestFramework::TwoModels(const std::string& modelName1,
                                    const std::string& modelName2,
                                    const float cellSizeFactor,
                                    const bool interactive,
                                    const std::string& outputPath)
{
  ASSERT_GT(cellSizeFactor, 1e-07) << "Cell size factor too small";

  GzMultipleWorldsServer::Ptr mServer = GetServer();
  ASSERT_NE(mServer.get(), nullptr) << "Could not create and start server";
  GzWorldManager::Ptr worldManager = mServer->GetWorldManager();
  ASSERT_NE(worldManager.get(), nullptr) << "No valid world manager created";

  worldManager->SetDynamicsEnabled(false);
  worldManager->SetPaused(true);

  int numWorlds = worldManager->GetNumWorlds();

  worldManager->SetPaused(false);

  // set models to their initial pose

  AABB aabb1, aabb2;
  ASSERT_TRUE(GetAABBs(modelName1, modelName2, aabb1, aabb2));

  // std::cout<<"Got AABB 1: " <<  aabb1.min << ", " << aabb1.max << std::endl;
  // std::cout<<"Got AABB 2: " <<  aabb2.min << ", " << aabb2.max << std::endl;

  AABB grid = aabb1;
  grid.min -= aabb2.size() / 2;
  grid.max += aabb2.size() / 2;

  // place model 2 at start position
  BasicState bstate2;
  bstate2.SetPosition(Vector3(grid.min.X(), grid.min.Y(), grid.min.Z()));
  int cnt = worldManager->SetBasicModelState(modelName2, bstate2);
  ASSERT_EQ(cnt, numWorlds) << "All worlds should have been updated";


  float cellSizeX = grid.size().X() * cellSizeFactor;
  float cellSizeY = grid.size().Y() * cellSizeFactor;
  float cellSizeZ = grid.size().Z() * cellSizeFactor;
  /* std::cout << "GRID : " <<  grid.min << ", " << grid.max << std::endl;
  std::cout << "cell size : " <<  cellSizeX << ", " <<cellSizeY << ", "
            << cellSizeZ << std::endl; */

  if (interactive)
  {
    std::cout << "Now start gzclient if you would like "
              << "to view the test. "<<std::endl;
    std::cout << "Press [Enter] to continue."<<std::endl;
    getchar();
  }

  // start the update loop
  std::cout << "Now starting to update worlds."<<std::endl;

  int msSleep = 0;  // delay for running the test
  double eps = 1e-07;
  unsigned int itCnt = 0;
  unsigned int failCnt = 0;
  for (double x = grid.min.X(); x < grid.max.X()+eps; x += cellSizeX)
  for (double y = grid.min.Y(); y < grid.max.Y()+eps; y += cellSizeY)
  for (double z = grid.min.Z(); z < grid.max.Z()+eps; z += cellSizeZ)
  {
    ++itCnt;
    // std::cout<<"Placing model 2 at "<<x<<", "<<y<<", "<<z<<std::endl;
    bstate2.position.x = x;
    bstate2.position.y = y;
    bstate2.position.z = z;
    cnt = worldManager->SetBasicModelState(modelName2, bstate2);
    ASSERT_EQ(cnt, numWorlds) << "All worlds should have been updated";

    int numSteps=1;
    worldManager->Update(numSteps);
    if (msSleep > 0) gazebo::common::Time::MSleep(msSleep);

    std::vector<std::string> colliding, notColliding;
    double maxNegDepth;
    ASSERT_TRUE(CollisionState(modelName1, modelName2,
                               colliding, notColliding, maxNegDepth));

    static const double zeroDepthTol = 1e-02;
    if (!colliding.empty() && (maxNegDepth >= -zeroDepthTol))
    {
      // if contacts were found but they are just surface contacts,
      // skip this because engines are actually allowed to disagree.
      // std::cout << "DEBUG-INFO: Not considering case of maximum depth 0 "
      //          << "because this is a borderline case" << std::endl;
      continue;
    }

    size_t total = colliding.size() + notColliding.size();

    ASSERT_EQ(numWorlds, total) << "All worlds must have voted";
    ASSERT_GT(total, 0 ) << "This should have been caught before";

    double negative = notColliding.size() / (double) total;
    double positive= colliding.size() / (double) total;

    const static double minAgree = 1.1; //0.999;
    if (((positive > negative) && (positive < minAgree)) ||
        ((positive <= negative) && (negative < minAgree)))
    {
      std::stringstream str;
      std::cout<<"Agreement: "<<positive<<", "<<negative<<std::endl;
      str << "Minimum agreement not reached.";

      // str << " Collision: "<< VectorToString(colliding) << ", no collision: "
      //     << VectorToString(notColliding) << ".";

      str << std::endl << "Colliding: " << std::endl << " ------ " << std::endl;
      for (std::vector<std::string>::iterator it = colliding.begin();
           it != colliding.end(); ++it)
      {
        if (it != colliding.begin()) str << std::endl;
        std::vector<ContactInfoPtr> contacts =
          GetContactInfo(modelName1, modelName2, *it);
        str << *it << ": " << VectorPtrToString(contacts);
      }
      str << std::endl << "Not colliding: " << std::endl
          << " ------ " << std::endl;
      for (std::vector<std::string>::iterator it = notColliding.begin();
           it != notColliding.end(); ++it)
      {
        if (it != notColliding.begin()) str << std::endl;
        std::vector<ContactInfoPtr> contacts =
          GetContactInfo(modelName1, modelName2, *it);
        str << *it << ": " << VectorPtrToString(contacts);
      }
      str << std::endl;

      if (!outputPath.empty())
      {
        std::stringstream namePrefix;
        namePrefix << "Static_fail_" << failCnt << "_";
        int nFails = worldManager->SaveAllWorlds(outputPath, namePrefix.str());
        std::cout << "Worlds written to " << outputPath
                  << " (failed: "<< nFails < ")" <<std::endl;
      }

      if (interactive)
      {
        std::cout << str.str() << std::endl
                  << "Press [Enter] to continue."<<std::endl;
        RefreshClient(5);
        WaitForEnter(worldManager);
      }
      else
      {
        // trigger a test failure
        EXPECT_TRUE(false) << str.str();
      }
      ++failCnt;
    }
  }
  std::cout<<"TwoModels test finished. "<<std::endl;
}
