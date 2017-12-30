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
#ifndef COLLISION_BENCHMARK_TEST_MULTIPLEWORLDSTESTFRAMEWORK_H
#define COLLISION_BENCHMARK_TEST_MULTIPLEWORLDSTESTFRAMEWORK_H

#include <collision_benchmark/GazeboMultipleWorlds.hh>
#include <collision_benchmark/GazeboMultipleWorldsServer.hh>
#include <collision_benchmark/GazeboWorldLoader.hh>
#include <collision_benchmark/GazeboHelpers.hh>

#include <test/TestUtils.hh>

#include <gtest/gtest.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

using collision_benchmark::GazeboPhysicsWorldTypes;

class MultipleWorldsTestFramework : public ::testing::Test
{
  public:

  typedef collision_benchmark::MultipleWorldsServer<
                               GazeboPhysicsWorldTypes::WorldState,
                               GazeboPhysicsWorldTypes::ModelID,
                               GazeboPhysicsWorldTypes::ModelPartID,
                               GazeboPhysicsWorldTypes::Vector3,
                               GazeboPhysicsWorldTypes::Wrench>
                                  GzMultipleWorldsServer;

  typedef collision_benchmark::WorldManager<GazeboPhysicsWorldTypes::WorldState,
                       GazeboPhysicsWorldTypes::ModelID,
                       GazeboPhysicsWorldTypes::ModelPartID,
                       GazeboPhysicsWorldTypes::Vector3,
                       GazeboPhysicsWorldTypes::Wrench>
            GzWorldManager;

  GzMultipleWorldsServer::Ptr GetServer() { return server; }

  protected:

  MultipleWorldsTestFramework()
  :fakeProgramName("MultipleWorldsTestFramework")
  {
  }
  virtual ~MultipleWorldsTestFramework()
  {
  }

  virtual void SetUp();

  virtual void TearDown();


  // Forces an update of the gazebo client which may be used to view the world
  // during testing. This is to help alleviate the following issue:
  // The poses of the models are not always updated in the client - some
  // pose messages will be skipped for the GazeboPhysicsWorld worlds,
  // because physics::World::posePub is throttled. Instead of allowing to
  // remove the throttling rate altoegther (which would be useful, but the
  // throttling rate has a purpose after all), this function can be used
  // to make the Gazebo clients(s) completely refresh the scene, which causes
  // them to re-request all information about all models.
  // \param[in] timoutSecs maximum timeout wait in seconds to wait for a
  //  client connection. Use negative value to wait forever.
  // \return false if there was an error preventing the refreshing.
  bool RefreshClient(const double timeoutSecs=-1);

  // \brief Initializes the framework and creates the world manager, but no
  // worlds are added to it.
  //
  // Throws gtest assertions so needs to be called from top-level
  // test function (nested function calls will not work correctly)
  void Init();

  // Called from Init(), to be used by subclasses.
  virtual void InitSpecific() {}

  // \brief Calls Init() and loads the empty world with all the given engines.
  //
  // Throws gtest assertions so needs to be called from top-level
  // test function (nested function calls will not work correctly)
  void InitMultipleEngines(const std::vector<std::string>& engines);

  // \brief Calls Init() and loads the empty world \e numWorld times
  // with the given engine by calling LoadOneEngine().
  // Each world can be accessed in the world
  // manager given the index [0..numWorlds-1].
  //
  // Throws gtest assertions so needs to be called from top-level
  // test function (nested function calls will not work correctly)
  void InitOneEngine(const std::string &engine,
                     const unsigned int numWorlds);


  // \brief Loads the empty world \e numWorld times with the given engine.
  // If the world manager previously had \e n engines, then it will then have
  // \e n + \e numWorld empty worlds loaded after this call.
  // Each world can be accessed in the world
  // manager given the index ``[n-1..n+numWorlds-1]``.
  //
  // Throws gtest assertions so needs to be called from top-level
  // test function (nested function calls will not work correctly)
  void LoadOneEngine(const std::string &engine,
                     const unsigned int numWorlds);

  // \brief Loads a shape into *all* worlds.
  // You must call Init(), InitMultipleEngines() or InitOneEngine()
  // before you can use this.
  //
  // Throws gtest assertions so needs to be called from top-level
  // test function (nested function calls will not work correctly)
  void LoadShape(const collision_benchmark::Shape::Ptr &shape,
                 const std::string &modelName);


  // \brief Loads a shape into the worlds at the given index \e worldIdx.
  // You must call Init(), InitMultipleEngines() or InitOneEngine()
  // before you can use this.
  //
  // Throws gtest assertions so needs to be called from top-level
  // test function (nested function calls will not work correctly)
  void LoadShape(const collision_benchmark::Shape::Ptr &shape,
                 const std::string &modelName,
                 const unsigned int worldIdx);

  // \brief Loads a model into *all* worlds.
  // You must call Init(), InitMultipleEngines() or InitOneEngine()
  // before you can use this.
  //
  // Throws gtest assertions so needs to be called from top-level
  // test function (nested function calls will not work correctly)
  void LoadModel(const std::string &modelFile,
                 const std::string &modelName);

  // \brief Loads a model into the worlds at the given index \e worldIdx.
  // You must call Init(), InitMultipleEngines() or InitOneEngine()
  // before you can use this.
  //
  // Throws gtest assertions so needs to be called from top-level
  // test function (nested function calls will not work correctly)
  void LoadModel(const std::string &modelFile,
                 const std::string &modelName,
                 const unsigned int worldIdx);

  // checks that AABB of model 1 and 2 are the same in all worlds and
  // returns the two AABBs
  // \param bbTol tolerance for comparison of bounding box sizes. The min/max
  //    coordinates (per x,y,z) are allowed to vary by this much in the worlds.
  // \return true if worlds are consistent, falsle otherwise
  bool GetAABBs(const std::string &modelName1,
                const std::string &modelName2,
                const double bbTol,
                collision_benchmark::GzAABB &m1,
                collision_benchmark::GzAABB &m2);
  private:

  const char * fakeProgramName;

  // the server, in case of non-interactive testing without gzclient
  // (otherwise set to NULL and using interactiveServer instead).
  GzMultipleWorldsServer::Ptr server;

  // the multiple worlds server with a client, in case of interactive
  // testing with gzclient (otherwise set to NULL and using server instead).
  GazeboMultipleWorlds::Ptr interactiveServer;

  // node needed in RefreshClient()
  gazebo::transport::NodePtr node;
  // publisher needed in RefreshClient()
  gazebo::transport::PublisherPtr pub;
};


#endif  // COLLISION_BENCHMARK_TEST_MULTIPLEWORLDSTESTFRAMEWORK_H

