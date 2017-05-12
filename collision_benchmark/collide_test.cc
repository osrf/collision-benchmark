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

#include <collision_benchmark/GazeboWorldLoader.hh>
#include <collision_benchmark/PhysicsWorld.hh>
#include <collision_benchmark/GazeboPhysicsWorld.hh>
#include <collision_benchmark/GazeboWorldState.hh>
#include <collision_benchmark/GazeboTopicForwardingMirror.hh>
#include <collision_benchmark/GazeboPhysicsWorld.hh>
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

#include <boost/program_options.hpp>

#include <unistd.h>
#include <sys/wait.h>

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
using collision_benchmark::GazeboWorldLoader;
using collision_benchmark::MultipleWorldsServer;
using collision_benchmark::GazeboMultipleWorldsServer;
using collision_benchmark::StartWaiter;

namespace po = boost::program_options;

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

// the child process ID for running gzclient
pid_t g_gzclient_pid;


// will be called at each server update iteration
void LoopIter(int iter)
{
}

// Initializes the multiple worlds server
bool Init(const bool loadMirror,
          const bool allowControlViaMirror,
          const bool enforceContactCalc)
{
  GzMultipleWorldsServer::WorldLoader_M loaders =
    collision_benchmark::GetSupportedGazeboWorldLoaders(enforceContactCalc);

  if (loaders.empty())
  {
    std::cerr << "Could not get support for any engine." << std::endl;
    return false;
  }

  WorldLoader::Ptr universalLoader(new GazeboWorldLoader(enforceContactCalc));

  g_server.reset(new GazeboMultipleWorldsServer(loaders, universalLoader));

  int argc = 1;
  const char * argv = "MultipleWorldsServer";
  g_server->Start(argc, &argv);

  std::string mirrorName = "";
  if (loadMirror) mirrorName = "mirror";

  g_server->Init(mirrorName, allowControlViaMirror);

  GzWorldManager::Ptr worldManager = g_server->GetWorldManager();
  if (!worldManager) return false;
  return true;
}

bool isClientRunning()
{
  if (g_gzclient_pid == 0)
    throw std::runtime_error("CONSISTENCY: This must be the parent process!");
  int child_status;
  // result will be 0 if child is still running
  pid_t result = waitpid(g_gzclient_pid, &child_status, WNOHANG);
  if (result != 0) // child has stopped (client closed)
  {
    return false;
  }
  return true;
}

// Runs the multiple worlds server
bool Run()
{
  GzWorldManager::Ptr worldManager = g_server->GetWorldManager();
  if (!worldManager) return false;

  GzWorldManager::ControlServerPtr controlServer =
    worldManager->GetControlServer();

  // helper class which will wait for the start signal
  StartWaiter startWaiter;
  startWaiter.SetUnpausedCallback(isClientRunning);

  if (controlServer)
  {
    controlServer->RegisterPauseCallback(std::bind(&StartWaiter::PauseCallback,
                                                   &startWaiter,
                                                   std::placeholders::_1));
  }

  worldManager->SetPaused(true);

  std::cout << "Press [Enter] to continue without gzclient or hit "
            << "the play button in gzclient."<<std::endl;

  // wait until either the [Play] button has been clicked, or [Enter] pressed.
  startWaiter.WaitForUnpause();

  worldManager->SetPaused(false);

  std::cout << "Now starting to update worlds."<<std::endl;
  int iter = 0;
  while(isClientRunning())
  {
    int numSteps=1;
    worldManager->Update(numSteps);
    LoopIter(iter);
    ++iter;
  }
  g_server->Stop();
  return true;
}


/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  g_gzclient_pid = fork();
  if (g_gzclient_pid == 0)
  {
    // child process: Start gzclient with the multiple worlds plugin
    char **argvClient = new char*[4];
    // silly const cast to avoid compiler warning
    argvClient[0] = const_cast<char*>(static_cast<const char*>("gzclient"));
    argvClient[1] = const_cast<char*>(static_cast<const char*>("--g"));
    argvClient[2] = const_cast<char*>
                    (static_cast<const char*>("libcollision_benchmark_gui.so"));
    argvClient[3] = static_cast<char*>(NULL);

    execvp(argvClient[0], argvClient);
    return 0;
  }
  else if (g_gzclient_pid < 0)
  {
    std::cerr << "Failed to fork process." << std::endl;
    return 1;
  }
  // this must be the parent process

  std::vector<std::string> selectedEngines;

  // description for engine options as stream so line doesn't go over 80 chars.
  std::stringstream descEngines;
  descEngines <<  "Specify one or several physics engines. " <<
      "Can contain [ode, bullet, dart, simbody]. Default is [ode].";

  // Declare the supported options.
  po::options_description desc("Allowed options");
  desc.add_options()
    ("help,h", "Produce help message")
    ;

  po::options_description desc_hidden("Positional options");
  desc_hidden.add_options()
    ("engines,e",
      po::value<std::vector<std::string>>(&selectedEngines)->multitoken(),
      descEngines.str().c_str())
    ;

  po::variables_map vm;
  po::positional_options_description p;
  // positional arguments default to "engines" argument
  p.add("engines", -1);

  po::options_description desc_composite;
  desc_composite.add(desc).add(desc_hidden);

  po::command_line_parser parser{argc, argv};
  parser.options(desc_composite).positional(p); // .allow_unregistered();
  po::parsed_options parsedOpt = parser.run();
  po::store(parsedOpt, vm);
  po::notify(vm);

  if (vm.count("help"))
  {
    std::cout << argv[0] <<" <list of engines> " << std::endl;
    std::cout << desc << std::endl;
    return 1;
  }

  if (vm.count("engines"))
  {
    std::cout << "Engines to load: " << std::endl;
    for (std::vector<std::string>::iterator it = selectedEngines.begin();
         it != selectedEngines.end(); ++it)
    {
      std::cout<<*it<<std::endl;
    }
  }
  else
  {
    std::cout << "No engines were given, so using 'ode'" << std::endl;
    selectedEngines.push_back("ode");
  }

  // Initialize server
  bool loadMirror = true;
  bool enforceContactCalc=false;
  bool allowControlViaMirror = true;
  Init(loadMirror, allowControlViaMirror, enforceContactCalc);
  assert(g_server);

  // load the world with the engine names given
  std::string worldPrefix = "collide_world";

  g_server->Load("worlds/empty.world", selectedEngines, worldPrefix);

  Run();

  // need to call server Fini() (or delete the server)
  // because if it's deleted after program exit
  // then it still will try to access static variables
  // which may have been deleted before.
  // g_server.reset();
  g_server->Fini();
  std::cout << "Bye, bye." << std::endl;
  return 0;
}
