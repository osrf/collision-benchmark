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

#include <collision_benchmark/GazeboMultipleWorldsServer.hh>
#include <collision_benchmark/GazeboWorldLoader.hh>
#include <collision_benchmark/GazeboHelpers.hh>

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

  virtual void SetUp()
  {
    bool enforceContactCalc = true;
    GzMultipleWorldsServer::WorldLoader_M loaders =
      collision_benchmark::GetSupportedGazeboWorldLoaders(enforceContactCalc);

    if (loaders.empty())
    {
      std::cerr << "Could not get support for any engine." << std::endl;
      return;
    }

    server.reset(new collision_benchmark::GazeboMultipleWorldsServer(loaders));
    server->Start(1, &fakeProgramName);
  }

  virtual void TearDown()
  {
    if (server) server->Stop();
  }


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

  private:

  const char * fakeProgramName;
  GzMultipleWorldsServer::Ptr server;

  // node needed in RefreshClient()
  gazebo::transport::NodePtr node;
  // publisher needed in RefreshClient()
  gazebo::transport::PublisherPtr pub;
};


#endif  // COLLISION_BENCHMARK_TEST_MULTIPLEWORLDSTESTFRAMEWORK_H

