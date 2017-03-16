/*
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
#ifndef COLLISION_BENCHMARK_TEST_STATICTESTFRAMEWORK_H
#define COLLISION_BENCHMARK_TEST_STATICTESTFRAMEWORK_H

#include <collision_benchmark/Shape.hh>
#include "MultipleWorldsTestFramework.hh"

#include <string>
#include <vector>

class StaticTestFramework : public MultipleWorldsTestFramework {
protected:
  struct AABB
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

  typedef GzWorldManager::PhysicsWorldContactInterfaceT::ContactInfo
            ContactInfo;
  typedef ContactInfo::Ptr ContactInfoPtr;

  StaticTestFramework():
    MultipleWorldsTestFramework()
  {}
  virtual ~StaticTestFramework()
  {}

  // Loads the two shapes. Call PrepareWorld() before.
  //
  // Throws gtest assertions so needs to be called from top-level
  // test function (nested function calls will not work correctly)
  void LoadShapes(const collision_benchmark::Shape::Ptr& shape1,
                 const std::string& modelName1,
                 const collision_benchmark::Shape::Ptr& shape2,
                 const std::string& modelName2);

  // loads the empty world with all the given engines and creates
  // the world manager.
  //
  // Throws gtest assertions so needs to be called from top-level
  // test function (nested function calls will not work correctly)
  void PrepareWorld(const std::vector<std::string>& engines);



  // Does the static test in which the two models (must already have
  // been loaded) are moved relative to each other, and all engines have to
  // agree on the collision state (boolean collision).
  // You need to use a loading function such as LoadShapes() before.
  //
  // Model 1 will remain stationary, while model 2 will
  // be moved along the 3D grid which is formed by the AABB of model 1,
  // expanded by half the dimensions of the AABB of model 2.
  //
  // Throws gtest assertions so needs to be called from top-level
  // test function (nested function calls will not work correctly)
  //
  // \param[in] cellSizeFactor the proportion of the 3D grid
  //    that will be used to determine the cell size, which is the size
  //    of cells that the models will be moved through.
  // \param[in] interactive if true, the test will be run interactively,
  //    which means the user gets the chance to start gzclient, and each
  //    test failure the test will be paused so they can look at the result.
  void TwoModels(const std::string& modelName1,
                 const std::string& modelName2,
                 const float cellSizeFactor = 0.1,
                 const bool interactive = false);

  // Tests if the worlds agree about the collision states
  // between the two models. The names of engines detecting a collision are
  // returned in \e colliding and the others in \e notColliding
  // \param[in] modelName1 name of model 1
  // \param[in] modelName2 name of model 2
  // \param[out] colliding names of all engines which determine collision
  // \param[out] notColliding names of all engines which determine no collision
  // \param[out] maxNegDepth largest negative depth recorded amongst
  //    all \colliding
  // \return false if there was an inconsistency or error in querying
  //    the collision states in any world
  bool CollisionState(const std::string& modelName1,
                      const std::string& modelName2,
                      std::vector<std::string>& colliding,
                      std::vector<std::string>& notColliding,
                      double& maxNegDepth);

private:

  // checks that AABB of model 1 and 2 are the same in all worlds and
  // returns the two AABBs
  // \return true if worlds are consistent, falsle otherwise
  bool GetAABBs(const std::string& modelName1,
                const std::string& modelName2,
                AABB& m1, AABB& m2);

  // returns contact info between model 1 and 2 in this world.
  std::vector<ContactInfoPtr> GetContactInfo(const std::string& modelName1,
                                             const std::string& modelName2,
                                             const std::string& worldName);

  // Forces an update of the gazebo client which may be used to view the world
  // during testing. This is to help alleviate the following issue:
  // The poses of the models are not always updated in the client, some
  // pose messages will be skipped for the GazeboPhysicsWorld worlds,
  // because in physics::World::posePub is throttled. Instead of allowing to
  // remove the throttling rate altoegther (which would be useful but the
  // throttling rate has a purpose after all), this function can be used
  // to make the Gazebo clients(s) completely refresh the scene, which causes
  // them to re-request all information about all models.
  // \param[in] timoutSecs maximum timeout wait in seconds to wait for a
  //  client connection. Use negative value to wait forever.
  // \return false if there was an error preventing the refreshing.
  bool RefreshClient(const double timeoutSecs=-1);

  // node needed in RefreshClient()
  gazebo::transport::NodePtr node;
  // publisher needed in RefreshClient()
  gazebo::transport::PublisherPtr pub;
};

#endif  // COLLISION_BENCHMARK_TEST_STATICTESTFRAMEWORK_H
