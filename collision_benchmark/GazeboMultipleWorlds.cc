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

#include <collision_benchmark/GazeboMultipleWorlds.hh>

#include <collision_benchmark/GazeboWorldLoader.hh>
#include <collision_benchmark/GazeboTopicForwardingMirror.hh>
#include <collision_benchmark/GazeboPhysicsWorld.hh>
#include <collision_benchmark/boost_std_conversion.hh>
#include <collision_benchmark/GazeboHelpers.hh>
#include <collision_benchmark/GazeboControlServer.hh>

#include <collision_benchmark/GazeboMultipleWorldsServer.hh>
#include <collision_benchmark/WorldLoader.hh>

#include <collision_benchmark/StartWaiter.hh>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/SensorsIface.hh>

using collision_benchmark::GazeboMultipleWorlds;

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

///////////////////////////////////////////////////////////////////////////////
bool GazeboMultipleWorlds::Init(const bool loadMirror,
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

  server.reset(new GazeboMultipleWorldsServer(loaders, universalLoader));

  int argc = 1;
  const char * argv = "MultipleWorldsServer";
  server->Start(argc, &argv);

  std::string mirrorName = "";
  if (loadMirror) mirrorName = "mirror";

  server->Init(mirrorName, allowControlViaMirror);

  GzWorldManager::Ptr worldManager = server->GetWorldManager();
  if (!worldManager) return false;
  return true;
}

///////////////////////////////////////////////////////////////////////////////
bool GazeboMultipleWorlds::isClientRunning()
{
  if (gzclient_pid == 0)
    throw std::runtime_error("CONSISTENCY: This must be the parent process!");
  int child_status;
  // result will be 0 if child is still running
  pid_t result = waitpid(gzclient_pid, &child_status, WNOHANG);
  if (result != 0) // child has stopped (client closed)
  {
    return false;
  }
  return true;
}

///////////////////////////////////////////////////////////////////////////////
bool GazeboMultipleWorlds::Run(bool physicsEnabled,
                               bool waitForStartSignal,
                               const std::function<void(int)>& loopCallback)

{
  if (!IsChild() && !IsParent())
  {
    std::cerr << "Must initialize correctly before running." << std::endl;
    return false;
  }
  if (IsChild())
  {
    std::cout <<"######## Child process is already running." << std::endl;
    return true;
  }

  GzWorldManager::Ptr worldManager = server->GetWorldManager();
  if (!worldManager) return false;

  worldManager->SetDynamicsEnabled(physicsEnabled);

  // wait until the client is running before starting the simulation.
  std::cout << "Waiting for client to come up... " << std::endl;
  while (!isClientRunning())
  {
    gazebo::common::Time::MSleep(100);
  }
  std::cout << "Client has started." << std::endl;

  GzWorldManager::ControlServerPtr controlServer =
    worldManager->GetControlServer();

  if (waitForStartSignal)
  {
    // helper class which will wait for the start signal
    StartWaiter startWaiter;
    // XXX REMOVE ME (old implementation) use this to wait until
    // gzclient has started up, though we do this above already.
    // startWaiter.SetUnpausedCallback
    //  (std::bind(&GazeboMultipleWorlds::isClientRunning, this));

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
  }

  std::cout << "Now starting to update worlds."<<std::endl;
  int iter = 0;
  while(isClientRunning())
  {
    int numSteps=1;
    worldManager->Update(numSteps);
    if (loopCallback) loopCallback(iter);
    ++iter;
  }
  server->Stop();

  // need to call server Fini() (or delete the server)
  // because if it's deleted after program exit
  // then it still will try to access static variables
  // which may have been deleted before.
  // server.reset();
  server->Fini();
  return true;
}


bool GazeboMultipleWorlds::IsChild() const
{
  return gzclient_pid == 0;
}
bool GazeboMultipleWorlds::IsParent() const
{
  return gzclient_pid > 0;
}

///////////////////////////////////////////////////////////////////////////////
bool GazeboMultipleWorlds::Load(const std::vector<std::string>& selectedEngines,
                                bool loadMirror,
                                bool enforceContactCalc,
                                bool allowControlViaMirror)
{
  gzclient_pid = fork();
  if (gzclient_pid == 0)
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
    return true;
  }
  else if (gzclient_pid < 0)
  {
    std::cerr << "Failed to fork process." << std::endl;
    return false;
  }

  // this must be the parent process

  Init(loadMirror, allowControlViaMirror, enforceContactCalc);
  assert(server);

  // load the world with the engine names given
  std::string worldPrefix = "collide_world";

  server->Load("worlds/empty.world", selectedEngines, worldPrefix);

  return true;
}
