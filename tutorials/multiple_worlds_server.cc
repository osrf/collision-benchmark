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
/*
 * Author: Jennifer Buehler
 * Date: March 2018
 */

#include <collision_benchmark/PhysicsWorldInterfaces.hh>
#include <collision_benchmark/GazeboPhysicsWorld.hh>
#include <collision_benchmark/GazeboWorldState.hh>
#include <collision_benchmark/GazeboTopicForwardingMirror.hh>
#include <collision_benchmark/boost_std_conversion.hh>
#include <collision_benchmark/GazeboHelpers.hh>
#include <collision_benchmark/WorldManager.hh>
#include <collision_benchmark/GazeboControlServer.hh>

#include <collision_benchmark/GazeboMultipleWorldsServer.hh>
#include <collision_benchmark/WorldLoader.hh>

#include <collision_benchmark/StartWaiter.hh>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/SensorsIface.hh>

using collision_benchmark::PhysicsWorldBaseInterface;
using collision_benchmark::PhysicsWorldStateInterface;
using collision_benchmark::PhysicsWorld;
using collision_benchmark::GazeboPhysicsWorld;
using collision_benchmark::GazeboPhysicsWorldTypes;
using collision_benchmark::MirrorWorld;
using collision_benchmark::GazeboTopicForwardingMirror;
using collision_benchmark::WorldManager;
using collision_benchmark::GazeboControlServer;

using collision_benchmark::WorldLoader;
using collision_benchmark::MultipleWorldsServer;
using collision_benchmark::GazeboMultipleWorldsServer;
using collision_benchmark::StartWaiter;

typedef MultipleWorldsServer<GazeboPhysicsWorldTypes::WorldState,
                             GazeboPhysicsWorldTypes::ModelID,
                             GazeboPhysicsWorldTypes::ModelPartID,
                             GazeboPhysicsWorldTypes::Vector3,
                             GazeboPhysicsWorldTypes::Wrench>
                                GzMultipleWorldsServer;

typedef WorldManager<GazeboPhysicsWorldTypes::WorldState,
                     GazeboPhysicsWorldTypes::ModelID,
                     GazeboPhysicsWorldTypes::ModelPartID,
                     GazeboPhysicsWorldTypes::Vector3,
                     GazeboPhysicsWorldTypes::Wrench>
          GzWorldManager;

// the server
GzMultipleWorldsServer::Ptr g_server;

// Initializes the multiple worlds server
// \param loadMirror whether to load a mirror world or not
// \param allowControlViaMirror allow control of the worlds via mirror.
//        See also constructor parameter \e activeControl in WorldManager.
// \param enforceContactCalc by default, contacts in Gazebo are only
//    computed if there is at least one subscriber to the contacts topic.
//    Use this flag to enforce contacts computation in any case.
//    See also constructor parameter \e enforceContactComputation in
//    GazeboPhysicsWorld.
void Init(const bool loadMirror,
          const bool allowControlViaMirror,
          const bool enforceContactCalc)
{
  // Get all WorldLoaders which are supported for gazebo.
  // A WorldLoader helps to load a world with a specific physics engine.
  GzMultipleWorldsServer::WorldLoader_M loaders =
    collision_benchmark::GetSupportedGazeboWorldLoaders(enforceContactCalc);
  if (loaders.empty())
  {
    throw std::runtime_error("Could not get support for any engine.");
  }

  // Create the multiple worlds server and use the Gazebo implementation.
  g_server.reset(new GazeboMultipleWorldsServer(loaders));

  // Set the mirror name to something meaningful if a mirror is to be used.
  // If the mirror name is empty, no MirrorWorld will be used.
  std::string mirrorName = "";
  if (loadMirror) mirrorName = "mirror";

  // Start the multiple worlds server. Command line parameters may be supported
  // by some MultipleWorldsServer implementations, but we will not use them
  // here because all settings are to be configured by accessing the
  // MultipleWorldsServer interface directly.
  int argc = 1;
  const char * fake_argv = "MultipleWorldsServer";
  g_server->Start(mirrorName, allowControlViaMirror, argc, &fake_argv);
}

// Runs the multiple worlds server
bool Run()
{
  GzWorldManager::Ptr worldManager = g_server->GetWorldManager();
  if (!worldManager) return false;

  worldManager->SetPaused(false);

  std::cout << "Starting to update worlds." << std::endl;
  std::cout << "You can the worlds with gzclient --gui-client-plugin "
            << "libcollision_benchmark_gui.so" << std::endl;
  int iter = 0;
  // TODO: at this point, we can only stop the program with Ctrl+C
  // which is not great. Find a better way to do this.
  while (true)
  {
    int numSteps = 1;
    worldManager->Update(numSteps);
    ++iter;
  }
  g_server->Stop();
  return true;
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  // Add some world files to be loaded.
  // Gazebo finds the worlds in the GAZEBO_WORLD_PATH, if we wanted to be
  // more universal for other physics worlds, we should use the absolute
  // path instead here.
  // The physics world implementation will need to support SDF for this to work.
  std::vector<std::string> worldFiles =
  {
    "worlds/rubble.world",
  };

  // Specify the engines to load
  std::vector<std::string> enginesToLoad =
  {
    "ode",
    "bullet"
  };

  // Initialize the server
  bool loadMirror = true;
  bool enforceContactCalc = false;
  bool allowControlViaMirror = true;
  Init(loadMirror, allowControlViaMirror, enforceContactCalc);
  assert(g_server);

  // load the worlds as given in command line arguments
  // with the engine names given
  int i = 0;
  for (std::vector<std::string>::iterator it = worldFiles.begin();
       it != worldFiles.end(); ++it, ++i)
  {
    std::string worldfile = *it;
    std::cout << "Loading world " << worldfile <<std::endl;

    std::stringstream worldPrefix;
    worldPrefix << "world" << "_" << i;

    // If there are no engines to load, we can try to read the physics engine
    // specification wihtin the world file. This only succeeds if the
    // implementation supports it, which is the case with
    // GazeboMultipleWorldsServer.
    if (enginesToLoad.empty())
    {
      if (g_server->AutoLoad(worldfile, worldPrefix.str()) < 0)
      {
        std::cerr << "Could not auto-load world " << worldfile << std::endl;
      }
    }
    else
    {
      g_server->Load(worldfile, enginesToLoad, worldPrefix.str());
    }
  }

  if (!Run())
  {
    std::cerr << "Could not run the multiple worlds server" << std::endl;
    return 1;
  }
  return 0;
}
