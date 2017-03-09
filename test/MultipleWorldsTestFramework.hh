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

/*using collision_benchmark::MultipleWorldsServer;
using collision_benchmark::GazeboMultipleWorldsServer;
using collision_benchmark::WorldLoader;
using collision_benchmark::WorldManager;
using collision_benchmark::GazeboWorldLoader;*/
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
    bool enforceContactCalc=true;
    std::set<std::string> engines =
      collision_benchmark::GetSupportedPhysicsEngines();
    GzMultipleWorldsServer::WorldLoader_M loaders;
    for (std::set<std::string>::const_iterator
         it = engines.begin(); it != engines.end(); ++it)
    {
      std::string engine = *it;
      try
      {
        loaders[engine] =
          collision_benchmark::WorldLoader::ConstPtr
            (new collision_benchmark::GazeboWorldLoader(engine,
                                                        enforceContactCalc));
      }
      catch (collision_benchmark::Exception& e)
      {
        std::cerr << "Could not add support for engine "
                  <<engine << ": " << e.what() << std::endl;
        continue;
      }
    }

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

  public: GzMultipleWorldsServer::Ptr GetServer() { return server; }

  private:
  const char * fakeProgramName;
  GzMultipleWorldsServer::Ptr server;
};


#endif  // COLLISION_BENCHMARK_TEST_MULTIPLEWORLDSTESTFRAMEWORK_H

