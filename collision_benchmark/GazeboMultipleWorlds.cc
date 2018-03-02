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
 * Date: May 2017
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

const std::string GazeboMultipleWorlds::MirrorName = "mirror";

GazeboMultipleWorlds::GazeboMultipleWorlds()
  : started(false)
{
}

GazeboMultipleWorlds::~GazeboMultipleWorlds()
{
  Stop();
}

///////////////////////////////////////////////////////////////////////////////
bool GazeboMultipleWorlds::InitServer(const bool loadMirror,
                                      const bool allowControlViaMirror,
                                      const bool enforceContactCalc)
{
  started = false;
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
  std::string mirrorName = "";
  if (loadMirror) mirrorName = MirrorName;
  server->Start(mirrorName, allowControlViaMirror, argc, &argv);

  GzWorldManager::Ptr worldManager = server->GetWorldManager();
  if (!worldManager) return false;

  return true;
}

///////////////////////////////////////////////////////////////////////////////
bool GazeboMultipleWorlds::IsClientClosed() const
{
  return !IsClientRunning();
}

///////////////////////////////////////////////////////////////////////////////
bool GazeboMultipleWorlds::IsClientRunning() const
{
  if (progPID == 0)
    throw std::runtime_error("CONSISTENCY: This must be the parent process!");
  int child_status;
  // result will be 0 if child is still running
  pid_t result = waitpid(progPID, &child_status, WNOHANG);
  if (result != 0)
  {
    // child has stopped (client closed)
    return false;
  }
  return true;
}

///////////////////////////////////////////////////////////////////////////////
void GazeboMultipleWorlds::KillClient()
{
  if (IsParent())
  {
    kill(progPID, SIGKILL);
  }
}

///////////////////////////////////////////////////////////////////////////////
void GazeboMultipleWorlds::Stop()
{
  if (IsClientRunning())
  {
    KillClient();
  }
  // XXX TODO: interrupt Run() if it was called in blocking mode
  ShutdownServer();
}

///////////////////////////////////////////////////////////////////////////////
GazeboMultipleWorlds::GzWorldManager::Ptr
GazeboMultipleWorlds::GetWorldManager()
{
  if (!server) return nullptr;
  return server->GetWorldManager();
}

///////////////////////////////////////////////////////////////////////////////
bool GazeboMultipleWorlds::HasStarted() const
{
  return started;
}

///////////////////////////////////////////////////////////////////////////////
void GazeboMultipleWorlds::ShutdownServer()
{
  if (!server) return;
  server->Stop();
  // need to call server Fini() (or delete the server)
  // because if it's deleted after program exit
  // then it still will try to access static variables
  // which may have been deleted before.
  // server.reset();
  server->Fini();
}

///////////////////////////////////////////////////////////////////////////////
bool GazeboMultipleWorlds::IsChild() const
{
  return progPID == 0;
}
///////////////////////////////////////////////////////////////////////////////
bool GazeboMultipleWorlds::IsParent() const
{
  return progPID > 0;
}

///////////////////////////////////////////////////////////////////////////////
// handler for SIGSEV to kill all child processes
pid_t gPGid;
void handler(int sig)
{
  if (sig == SIGSEGV)
  {
    std::cout << "Segmentation fault - caught SIGSEV and killing "
              << "child process(es). " << __FILE__
              << ", " << __LINE__ << std::endl;
    kill(-gPGid, SIGTERM);     // kill the whole process group
  }
}

void initProcessGroup()
{
  setpgid(0, 0);       // Make new process group, if needed
  gPGid = getpgid(0);
  signal(SIGSEGV, handler);
}


///////////////////////////////////////////////////////////////////////////////
void StartClient(const bool verbose,
                 const std::vector<std::string>& additionalGuis)
{
  char **argvClient = new char*[4 +
                                additionalGuis.size() * 2 +
                                (verbose ? 1 : 0)];
  // silly const cast to avoid compiler warning
  argvClient[0] =
    const_cast<char*>(static_cast<const char*>("gzclient"));
  argvClient[1] =
    const_cast<char*>(static_cast<const char*>("--gui-client-plugin"));
  argvClient[2] =
    const_cast<char*>(static_cast<const char*>
                       ("libcollision_benchmark_gui.so"));
  int i = 3;
  for (int g = 0; g < additionalGuis.size(); ++g)
  {
    argvClient[i] =
      const_cast<char*>(static_cast<const char*>("--gui-client-plugin"));
    ++i;
    argvClient[i] =
      const_cast<char*>(static_cast<const char*>(additionalGuis[g].c_str()));
    ++i;
  }

  if (verbose)
  {
    argvClient[i] = const_cast<char*>(static_cast<const char*>("--verbose"));
    ++i;
  }

  argvClient[i] = static_cast<char*>(NULL);
  execvp(argvClient[0], argvClient);
}

///////////////////////////////////////////////////////////////////////////////
bool GazeboMultipleWorlds::Run(bool waitForStartSignal,
                               bool blocking,
                               const std::function<void(int)>& loopCallback)

{
  if (!IsParent())
  {
    std::cerr << "Must initialize correctly before running." << std::endl;
    return false;
  }
  if (IsChild())
  {
    std::cerr << "Consistency: Calling from child process, "
              << "this should not happen. " << std::endl;
    return false;
  }

  GzWorldManager::Ptr worldManager = server->GetWorldManager();
  if (!worldManager) return false;

  // wait until the client is running before starting the simulation.
  std::cout << "GazeboMultipleWorlds: Waiting for client to come up... "
            << std::endl;
  while (!IsClientRunning())
  {
    gazebo::common::Time::MSleep(100);
  }
  std::cout << "GazeboMultipleWorlds: ...client has connected." << std::endl;

  GzWorldManager::ControlServerPtr controlServer =
    worldManager->GetControlServer();

  if (waitForStartSignal)
  {
    // helper class which will wait for the start signal
    StartWaiter startWaiter;
    // XXX REMOVE ME (old implementation) use this to wait until
    // gzclient has started up, though we do this above already.
    // startWaiter.SetUnpausedCallback
    //  (std::bind(&GazeboMultipleWorlds::IsClientRunning, this));

    if (controlServer)
    {
      controlServer->RegisterPauseCallback
        (std::bind(&StartWaiter::PauseCallback,
                   &startWaiter, std::placeholders::_1));
    }

    // closing the client also serves as an "unpause" signal, so that the
    // thread stops waiting.
    startWaiter.AddUnpausedCallback
      (std::bind(&GazeboMultipleWorlds::IsClientClosed, this));

    worldManager->SetPaused(true);

    std::cout << "Press [Enter] to continue without gzclient or hit "
              << "the play button in gzclient." << std::endl;

    // wait until either the [Play] button has been clicked, or [Enter] pressed.
    startWaiter.WaitForUnpause();

    worldManager->SetPaused(false);
  }
  started = true;

  if (!IsClientRunning())
  {
    std::cout << "Client was closed before simulation was started. "
              << "Will not start simulation. " << std::endl;
    return true;
  }

  // std::cout << "Now starting to update worlds." << std::endl;
  int iter = 0;
  worldManager->SetPaused(false);

  // for non-blocking calls, don't do the simulation loop in here.
  if (!blocking) return true;

  while (IsClientRunning())
  {
    int numSteps = 1;
    worldManager->Update(numSteps);
    if (loopCallback) loopCallback(iter);
    ++iter;
  }
  ShutdownServer();
  return true;
}

///////////////////////////////////////////////////////////////////////////////
bool GazeboMultipleWorlds::Init(const bool loadMirror,
                                const bool enforceContactCalc,
                                const bool allowControlViaMirror,
                                const bool useInteractiveMode,
                                const std::vector<std::string>& additionalGuis
                                )
{
  interactiveMode = useInteractiveMode;

  // only interactive mode loads up gzclient and needs fork().
  if (InteractiveMode())
  {
    initProcessGroup();
    progPID = fork();
  }
  else
  {
    // only set the PID to the parent PID
    progPID = getpid();
  }
  if (IsChild())
  {
    bool verbose = false;
    // child process: Start gzclient with the multiple worlds plugin
    StartClient(verbose, additionalGuis);
    return true;
  }
  else if (progPID < 0)
  {
    std::cerr << "Failed to fork process." << std::endl;
    return false;
  }

  // this must be the parent process
  InitServer(loadMirror, allowControlViaMirror, enforceContactCalc);
  assert(server);
  return true;
}

///////////////////////////////////////////////////////////////////////////////
bool GazeboMultipleWorlds::LoadEngines
      (const std::vector<std::string>& selectedEngines, bool physicsEnabled)
{
  assert(server);
  if (!server) return false;

  // load the world with the engine names given
  std::string worldPrefix = "collide_world";

  server->Load("test_worlds/void.world", selectedEngines, worldPrefix);
  // server->Load("worlds/empty.world", selectedEngines, worldPrefix);

  GzWorldManager::Ptr worldManager = server->GetWorldManager();
  if (!worldManager) return false;

  worldManager->SetDynamicsEnabled(physicsEnabled);
  return true;
}
