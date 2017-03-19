#include <test/TestUtils.hh>
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
using collision_benchmark::GzWorldManager;

std::atomic<bool> g_keypressed(false);

// waits until Enter has been pressed and sets g_keypressed to true
void WaitForEnterImpl()
{
  int key = getchar();
  g_keypressed=true;
}

////////////////////////////////////////////////////////////////
void collision_benchmark::UpdateUntilEnter(GzWorldManager::Ptr& worlds)
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
bool collision_benchmark::GetConsistentAABB(const std::string& modelName,
                                  const GzWorldManager::Ptr& worldManager,
                                  GzAABB& mAABB)
{
  std::vector<GzWorldManager::PhysicsWorldModelInterfacePtr>
    worlds = worldManager->GetModelPhysicsWorlds();

  // AABB's from all worlds: need to be equal or this function
  // must return false.
  std::vector<GzAABB> aabbs;

  std::vector<GzWorldManager::PhysicsWorldModelInterfacePtr>::iterator it;
  for (it = worlds.begin(); it != worlds.end(); ++it)
  {
    GzWorldManager::PhysicsWorldModelInterfacePtr w = *it;
    GzAABB aabb;
    if (!w->GetAABB(modelName, aabb.min, aabb.max))
    {
      std::cerr << "Model " << modelName << " AABB could not be retrieved"
                << std::endl;
      return false;
    }
    aabbs.push_back(aabb);
  }

  if (aabbs.empty())
  {
    std::cerr << "At least one bounding box should "
              << "have been returned" << std::endl;
    return false;
  }

  // epsilon for vector comparison
  const static float eps = 5e-02;

  // Check that all AABBs are the same
  std::vector<GzAABB>::iterator itAABB;
  GzAABB lastAABB;
  for (itAABB = aabbs.begin();  itAABB != aabbs.end(); ++itAABB)
  {
    const GzAABB& aabb = *itAABB;
    if (itAABB != aabbs.begin())
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

  mAABB = aabbs.front();
  return true;
}


////////////////////////////////////////////////////////////////
std::vector<collision_benchmark::GzContactInfoPtr>
collision_benchmark::GetContactInfo(const std::string& modelName1,
                                    const std::string& modelName2,
                                    const std::string& worldName,
                                    const GzWorldManager::Ptr& worldManager)
{
  std::vector<GzContactInfoPtr> ret;

  PhysicsWorldBaseInterface::Ptr w = worldManager->GetWorld(worldName);
  assert(w);

  GzWorldManager::PhysicsWorldPtr pWorld = worldManager->ToPhysicsWorld(w);
  assert(pWorld);

  return pWorld->GetContactInfo(modelName1, modelName2);
}


////////////////////////////////////////////////////////////////
bool collision_benchmark::CollisionState(const std::string& modelName1,
                                       const std::string& modelName2,
                                       const GzWorldManager::Ptr& worldManager,
                                       std::vector<std::string>& colliding,
                                       std::vector<std::string>& notColliding,
                                       double& maxNegDepth)
{
  colliding.clear();
  notColliding.clear();
  maxNegDepth = 0;
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

    std::vector<GzContactInfoPtr> contacts =
      w->GetContactInfo(modelName1, modelName2);
    if (!contacts.empty())
    {
      colliding.push_back(w->GetName());
      for (typename std::vector<GzContactInfoPtr>::const_iterator
           cit = contacts.begin(); cit != contacts.end(); ++cit)
      {
        GzContactInfoPtr c = *cit;
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
