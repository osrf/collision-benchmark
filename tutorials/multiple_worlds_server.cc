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
#include <collision_benchmark/GazeboMultipleWorldsServer.hh>

using collision_benchmark::PhysicsWorldBaseInterface;
using collision_benchmark::PhysicsWorldStateInterface;
using collision_benchmark::GazeboPhysicsWorldTypes;
using collision_benchmark::WorldManager;
using collision_benchmark::WorldLoader;
using collision_benchmark::MultipleWorldsServer;
using collision_benchmark::GazeboMultipleWorldsServer;

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

// \brief Initializes the multiple worlds server
// \param[in] loadMirror whether to load a MirrorWorld or not
// \param[in] allowControlViaMirror allow control of the worlds via mirror.
//        See also constructor parameter \e activeControl in WorldManager.
// \param[in] enforceContactCalc by default, contacts in Gazebo are only
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

// \brief main method
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

  // get the world manager
  GzWorldManager::Ptr worldManager = g_server->GetWorldManager();
  assert(worldManager);

  // Set the paused state of the worlds. We will for now pause the
  // worlds and you can un-pause them using gzclient
  // (unless you set allowControlViaMirror to false, then it won't work)
  worldManager->SetPaused(true);

  // print a status on screen so that we know that the server is running
  // and we can now start the client.
  std::cout << "Starting to update worlds." << std::endl;
  std::cout << "You can the worlds with gzclient --gui-client-plugin "
            << "libcollision_benchmark_gui.so" << std::endl;

  // Main loop is an endless loop here for demonstration purposes,
  // but you can use any abort criterion here.
  int iter = 0;
  while (true)
  {
    int numSteps = 1;
    worldManager->Update(numSteps);
    ++iter;
  }

  // If you add a proper exit criterion for the main loop and the
  // code here is actually executed, you can stop the server here.
  // However GazeboMultipleWorldsServer stops itself in its destructor
  // too so in this particular case, we don't need to to this.
  g_server->Stop();
  return 0;
}
