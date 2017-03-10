#include <test/StaticTestFramework.hh>
#include <collision_benchmark/PrimitiveShape.hh>
#include <collision_benchmark/SimpleTriMeshShape.hh>
#include <collision_benchmark/BasicTypes.hh>

#include <ignition/math/Vector3.hh>

#include <gazebo/gazebo.hh>

#include "MultipleWorldsTestFramework.hh"

#include <sstream>
#include <thread>
#include <atomic>

using collision_benchmark::Shape;
using collision_benchmark::BasicState;
using collision_benchmark::Vector3;
using collision_benchmark::Quaternion;
using collision_benchmark::PhysicsWorldBaseInterface;

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
  int numWorlds = mServer->Load(worldfile, engines,
                                mirrorName, allowControlViaMirror);

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

  // TwoModels(modelName1, modelName2);
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
  const static float eps = 1e-03;

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
                                         std::vector<std::string>& notColliding)
{
  colliding.clear();
  notColliding.clear();
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
    if (!contacts.empty()) colliding.push_back(w->GetName());
    else notColliding.push_back(w->GetName());
  }
  return true;
}


////////////////////////////////////////////////////////////////
void StaticTestFramework::TwoModels(const std::string& modelName1,
                                    const std::string& modelName2,
                                    float cellSizeFactor)
{
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
  /*std::cout << "GRID : " <<  grid.min << ", " << grid.max << std::endl;
  std::cout << "cell size : " <<  cellSizeX << ", " <<cellSizeY << ", "
            << cellSizeZ << std::endl;*/

  std::cout << "Now start gzclient if you would like "
            << "to view the test. "<<std::endl;
  std::cout << "Press [Enter] to continue."<<std::endl;
  getchar();

  // start the update loop
  std::cout << "Now starting to update worlds."<<std::endl;

  int msSleep = 0;  // delay for running the test
  const static bool interactive = true;
  double eps = 1e-07;
  unsigned int itCnt = 0;
  for (double x = grid.min.X(); x < grid.max.X()+eps; x += cellSizeX)
  for (double y = grid.min.Y(); y < grid.max.Y()+eps; y += cellSizeY)
  for (double z = grid.min.Z(); z < grid.max.Z()+eps; z += cellSizeZ)
  {
    std::cout<<"Pose change " << itCnt << std::endl;
    ++itCnt;
    std::cout<<"Placing model 2 at "<<x<<", "<<y<<", "<<z<<std::endl;
    bstate2.position.x = x;
    bstate2.position.y = y;
    bstate2.position.z = z;
    cnt = worldManager->SetBasicModelState(modelName2, bstate2);
    ASSERT_EQ(cnt, numWorlds) << "All worlds should have been updated";

    int numSteps=1;
    worldManager->Update(numSteps);
    if (msSleep > 0) gazebo::common::Time::MSleep(msSleep);

    std::vector<std::string> colliding, notColliding;
    ASSERT_TRUE(CollisionState(modelName1, modelName2,
                               colliding, notColliding));

    size_t total = colliding.size() + notColliding.size();

    ASSERT_EQ(numWorlds, total) << "All worlds must have voted";
    ASSERT_GT(total, 0 ) << "This should have been caught before";

    double negative = notColliding.size() / (double) total;
    double positive= colliding.size() / (double) total;

    const static double minAgree = 1.0;
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

      if (interactive)
      {
        std::cout << str.str() << std::endl
                  << "Press [Enter] to continue."<<std::endl;
        WaitForEnter(worldManager);
      }
      else
      {
        EXPECT_TRUE(false) << str.str();
      }
    }
  }
}
