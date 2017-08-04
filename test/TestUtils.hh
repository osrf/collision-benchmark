/**
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#ifndef COLLISION_BENCHMARK_TEST_TESTUTILS_H
#define COLLISION_BENCHMARK_TEST_TESTUTILS_H

#include <collision_benchmark/WorldManager.hh>
// support only for gazebo types at the moment.
// Make the helpers header-only to support all types.
#include <collision_benchmark/GazeboPhysicsWorld.hh>

#include <string>
#include <vector>

namespace collision_benchmark
{
  struct GzAABB
  {
    typedef ignition::math::Vector3d Vec3;
    Vec3 size()
    {
      return Vec3(max.X() - min.X(),
                  max.Y() - min.Y(),
                  max.Z() - min.Z());
    }
    Vec3 min, max;
  };

  typedef collision_benchmark::WorldManager<GazeboPhysicsWorldTypes::WorldState,
                       GazeboPhysicsWorldTypes::ModelID,
                       GazeboPhysicsWorldTypes::ModelPartID,
                       GazeboPhysicsWorldTypes::Vector3,
                       GazeboPhysicsWorldTypes::Wrench>
            GzWorldManager;

  typedef GzWorldManager::PhysicsWorldContactInterfaceT::ContactInfo
            GzContactInfo;
  typedef GzContactInfo::Ptr GzContactInfoPtr;



  // Tests if the worlds agree about the collision states
  // between the two models. The names of engines detecting a collision are
  // returned in \e colliding and the others in \e notColliding
  // \param[in] modelName1 name of model 1
  // \param[in] modelName2 name of model 2
  // \param[in] worldManager the world manager which has all the worlds
  // \param[out] colliding names of all engines which determine collision
  // \param[out] notColliding names of all engines which determine no collision
  // \param[out] maxDepth largest depth recorded amongst
  //    all \colliding
  // \return false if there was an inconsistency or error in querying
  //    the collision states in any world
  bool CollisionState(const std::string &modelName1,
                      const std::string &modelName2,
                      const GzWorldManager::Ptr &worldManager,
                      std::vector<std::string>& colliding,
                      std::vector<std::string>& notColliding,
                      double &maxDepth);

  // checks that AABB of model 1 is the same in all worlds in
  // \e worldManager and returns the AABBs of the model if it is
  // the same in all worlds.
  // \param bbTol tolerance for comparison of bounding box sizes. The min/max
  //    coordinates (per x,y,z) are allowed to vary by this much in the worlds.
  // \return true if models have same AABB in both worlds, false otherwise
  bool GetConsistentAABB(const std::string &modelName,
                         const GzWorldManager::Ptr &worldManager,
                         const double bbTol,
                         GzAABB &aabb);

  // Helper: returns contact info between model 1 and 2 in this world.
  std::vector<GzContactInfoPtr> GetContactInfo(const std::string &modelName1,
                                      const std::string &modelName2,
                                      const std::string &worldName,
                                      const GzWorldManager::Ptr &worldManager);


  // waits for the [Enter] key to be pressed, and while it's waiting,
  // updates the worlds
  void UpdateUntilEnter(GzWorldManager::Ptr &worlds);

  // helper which builds a string of the vector. T has to be a non-pointer
  // type and support the << operator.
  template<typename T>
  std::string VectorToString(const std::vector<T>& v)
  {
    std::stringstream str;
    str << "[";
    for (typename std::vector<T>::const_iterator it = v.begin();
         it != v.end(); ++it)
    {
      if (it != v.begin()) str << ", ";
      str << *it;
    }
    str << "]";
    return str.str();
  }

  // helper which builds a string of the vector. T has to be a pointer
  // type and support the << operator.
  template<typename T>
  std::string VectorPtrToString(const std::vector<T>& v)
  {
    std::stringstream str;
    str << "[";
    for (typename std::vector<T>::const_iterator it = v.begin();
         it != v.end(); ++it)
    {
      if (it != v.begin()) str << ", ";
      str << **it;
    }
    str << "]";
    return str.str();
  }
}

#endif  // COLLISION_BENCHMARK_TEST_TESTUTILS_H
