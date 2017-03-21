#include <test/TestUtils.hh>
#include <collision_benchmark/BasicTypes.hh>
#include <collision_benchmark/MirrorWorld.hh>

#include <ignition/math/Vector3.hh>

#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>

#include <boost/filesystem.hpp>

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
////////////////////////////////////////////////////////////////
void WaitForEnterImpl()
{
  int key = getchar();
  g_keypressed=true;
}


////////////////////////////////////////////////////////////////
bool collision_benchmark::makeDirectoryIfNeeded(const std::string& dPath)
{
  if (boost::filesystem::exists(dPath)) return true;
  try
  {
    boost::filesystem::path dir(dPath);
    boost::filesystem::path buildPath;

    for (boost::filesystem::path::iterator it(dir.begin()),
       it_end(dir.end()); it != it_end; ++it)
    {
      buildPath /= *it;
      //std::cout << buildPath << std::endl;

      if (!boost::filesystem::exists(buildPath) &&
        !boost::filesystem::create_directory(buildPath))
      {
        std::cerr << "Could not create directory " << buildPath << std::endl;
        return false;
      }
    }
  }
  catch (const boost::filesystem::filesystem_error& ex)
  {
    std::cerr << ex.what() << std::endl;
    return false;
  }
  return true;
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
                                  const double bbTol,
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

  // Check that all AABBs are the same
  std::vector<GzAABB>::iterator itAABB;
  GzAABB lastAABB;
  for (itAABB = aabbs.begin();  itAABB != aabbs.end(); ++itAABB)
  {
    const GzAABB& aabb = *itAABB;
    if (itAABB != aabbs.begin())
    {
      if (!aabb.min.Equal(lastAABB.min, bbTol) ||
          !aabb.max.Equal(lastAABB.max, bbTol))
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
                                       double& maxDepth)
{
  colliding.clear();
  notColliding.clear();
  maxDepth = 0;
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
        // std::cout << "World " << w->GetName() <<" Contact: " << *c << std::endl;
        double tmpMax;
        if (c->maxDepth(tmpMax) && tmpMax > maxDepth)
          maxDepth = tmpMax;
      }
      // std::cout << "Max depth: " << maxDepth << std::endl;
    }
    else
    {
      notColliding.push_back(w->GetName());
    }
  }
  return true;
}
