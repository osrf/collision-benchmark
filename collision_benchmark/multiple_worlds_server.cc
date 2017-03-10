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

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/SensorsIface.hh>

#include <atomic>

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

// typedef GazeboPhysicsWorld::PhysicsWorldTypes GazeboPhysicsWorldTypes;
// typedef PhysicsWorld<GazeboPhysicsWorldTypes> GzPhysicsWorld;

// test is paused or not
std::atomic<bool> g_unpaused(false);
std::atomic<bool> g_keypressed(false);


// waits until enter has been pressed and sets g_keypressed to true
void WaitForEnter()
{
  int key = getchar();
  g_keypressed=true;
}

// waits for either g_unpaused is set to
// true or until enter key was pressed.
void WaitForUnpause()
{
  g_keypressed = false;
  std::thread * t = new std::thread(WaitForEnter);
  t->detach();  // detach so it can be terminated
  while (!g_unpaused && !g_keypressed)
  {
    gazebo::common::Time::MSleep(100);
  }
  delete t;
}

void pauseCallback(bool pause)
{
  //std::cout<<"############ Pause callback: "<<pause<<std::endl;
  g_unpaused = !pause;
}


// old implementation
#if 0

// Main method to play the test, later to be replaced by a dedicated
// structure (without command line arument params)
bool Run(int argc, char **argv)
{
  if (argc < 3)
  {
    std::cerr << "Usage: "<<argv[0]
              << " <world file> <list of physics engines>"<<std::endl;
    std::cerr << "<list of physics engines> can contain the following: "
              << "[ode, bullet, dart, simbody] "<<std::endl;
    return false;
  }

  // first, load the mirror world (it has to be loaded first for
  // gzclient to connect to this one instead of the others.
  // To see why, refer to transport::Node::Init(), called
  // from gui::MainWindow constructor with the empty string)
  GazeboTopicForwardingMirror::Ptr
    mirrorWorld(new GazeboTopicForwardingMirror("mirror_world"));
  GazeboControlServer::Ptr
    controlServer(new GazeboControlServer("mirror_world"));
  controlServer->RegisterPauseCallback(std::bind(pauseCallback,
                                                 std::placeholders::_1));

  // now, load the worlds as given in command line arguments
  // with the different engines given
  std::string worldfile = argv[1];

  std::vector<std::string> selectedEngines;
  for (int i = 2; i < argc; ++i)
    selectedEngines.push_back(argv[i]);

  std::map<std::string,std::string> physicsEngines =
    collision_benchmark::getPhysicsSettingsSdfFor(selectedEngines);

  std::cout << "Loading world " << worldfile << " with "
            << physicsEngines.size()<<" engines."<<std::endl;

  GzWorldManager worldManager(mirrorWorld, controlServer);
  int i=1;
  for (std::map<std::string,std::string>::iterator
       it = physicsEngines.begin(); it!=physicsEngines.end(); ++it, ++i)
  {
    std::string engine = it->first;
    std::string physicsSDF = it->second;

    std::stringstream _worldname;
    _worldname << "world_" << i << "_" << engine;
    std::string worldname=_worldname.str();

    std::cout << "Loading with physics engine " << engine
              << " (named as '" << worldname << "')" << std::endl;

    std::cout<<"Loading physics from "<<physicsSDF<<std::endl;
    sdf::ElementPtr physics =
      collision_benchmark::GetPhysicsFromSDF(physicsSDF);
    if (!physics.get())
    {
      std::cerr << "Could not get phyiscs engine from "
                << physicsSDF << std::endl;
      continue;
    }

    // std::cout<<"Physics: "<<physics->ToString("")<<std::endl;
    std::cout<<"Loading world from "<<worldfile<<std::endl;
    gazebo::physics::WorldPtr gzworld =
      collision_benchmark::LoadWorldFromFile(worldfile, worldname, physics);
    if (!gzworld)
    {
      std::cout<<"Error loading world "<<worldfile<<std::endl;
      return false;
    }

    std::cout<<"Creating GazeboPhysicsWorld. "<<std::endl;
    // Create the GazeboPhysicsWorld object
    bool enforceContactCalc=true;
    GazeboPhysicsWorld::Ptr
      gzPhysicsWorld(new GazeboPhysicsWorld(enforceContactCalc));
    gzPhysicsWorld->SetWorld
      (collision_benchmark::to_std_ptr<gazebo::physics::World>(gzworld));
    worldManager.AddPhysicsWorld(gzPhysicsWorld);
  }

//  worldManager.SetDynamicsEnabled(false);
  worldManager.SetPaused(true);

  std::cout << "Now start gzclient if you would like "
            << "to view the test. "<<std::endl;
  std::cout << "Press [Enter] to continue without gzclient or hit "
            << "the play button in gzclient."<<std::endl;
  WaitForUnpause();

  worldManager.SetPaused(false);

  std::cout << "Now starting to update worlds."<<std::endl;
  //const int printCnt=100;
  //int print=printCnt;
  while(true)
  {
    int numSteps=1;
    worldManager.Update(numSteps);

    // while (true) gazebo::common::Time::MSleep(1000);
  /*  PhysicsWorldBaseInterface::Ptr mirroredWorld
        = worldManager.GetMirroredWorld();
    GzPhysicsWorld::Ptr mirroredWorldCast =
        std::dynamic_pointer_cast<GzPhysicsWorld>(mirroredWorld);
    assert(mirroredWorldCast);
    std::vector<GzPhysicsWorld::ContactInfoPtr> contacts =
        mirroredWorldCast->GetContactInfo();
    std::cout<<"Number of contacts: "<<contacts.size()<<std::endl;
    getchar();*/

    // temporary fix: wen paused, idle a little,
    // to not make this print too often
    /*if (!g_unpaused)
      gazebo::common::Time::MSleep(100);
    if (print <= 0)
    {
      std::cout<<"Updating world still going."<<std::endl;
      print=printCnt;
    }
    --print;*/
  }
  return true;
}


/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  gazebo::common::Console::SetQuiet(false);

  // Initialize gazebo.
  try
  {
    gazebo::setupServer(argc, argv);
  }
  catch (...)
  {
    std::cerr<<"Could not setup server"<<std::endl;
    return 1;
  }

  // See also gazebo::Server::Run() which is a procedure we probably
  // like to mimick here.
  // See also the hack required for the sensors, explained
  // in LoadWorldFromSDF() - using  gazebo::sensors::run_once(true) here
  // will have no effect as it does in gazebo::Server.

  Run(argc, argv);

  std::cout << "Shutting down..." <<std::endl;

  gazebo::shutdown();

  std::cout << "Multi-world server ended." << std::endl;
  return 0;
}

#else

bool Run(const std::string& worldfile,
         const std::vector<std::string>& selectedEngines,
         const bool loadMirror,
         const bool allowControlViaMirror,
         const bool enforceContactCalc)
{
  GzMultipleWorldsServer::WorldLoader_M loaders;
  for (std::vector<std::string>::const_iterator
       it = selectedEngines.begin(); it != selectedEngines.end(); ++it)
  {
    std::string engine = *it;
    try
    {
      loaders[engine] =
        WorldLoader::ConstPtr(new GazeboWorldLoader(engine,
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
    return false;
  }


  GzMultipleWorldsServer::Ptr
    server(new GazeboMultipleWorldsServer(loaders));

  int argc = 1;
  const char * argv = "MultipleWorldsServer";
  server->Start(argc, &argv);

  std::string mirrorName = "";
  if (loadMirror) mirrorName = "mirror";
  int numWorlds = server->Load(worldfile, selectedEngines,
                               mirrorName, allowControlViaMirror);

  GzWorldManager::Ptr worldManager = server->GetWorldManager();
  assert(worldManager);

  GzWorldManager::ControlServerPtr controlServer =
    worldManager->GetControlServer();

  if (controlServer)
  {
    controlServer->RegisterPauseCallback(std::bind(pauseCallback,
                                                   std::placeholders::_1));
  }

//  worldManager.SetDynamicsEnabled(false);
  worldManager->SetPaused(true);

  std::cout << "Now start gzclient if you would like "
            << "to view the test. "<<std::endl;
  std::cout << "Press [Enter] to continue without gzclient or hit "
            << "the play button in gzclient."<<std::endl;
  WaitForUnpause();

  worldManager->SetPaused(false);

  std::cout << "Now starting to update worlds."<<std::endl;
  //const int printCnt=100;
  //int print=printCnt;
  while(true)
  {
    int numSteps=1;
    worldManager->Update(numSteps);

    // while (true) gazebo::common::Time::MSleep(1000);
  /*  PhysicsWorldBaseInterface::Ptr mirroredWorld
        = worldManager.GetMirroredWorld();
    GzPhysicsWorld::Ptr mirroredWorldCast =
        std::dynamic_pointer_cast<GzPhysicsWorld>(mirroredWorld);
    assert(mirroredWorldCast);
    std::vector<GzPhysicsWorld::ContactInfoPtr> contacts =
        mirroredWorldCast->GetContactInfo();
    std::cout<<"Number of contacts: "<<contacts.size()<<std::endl;
    getchar();*/

    // temporary fix: wen paused, idle a little,
    // to not make this print too often
    /*if (!g_unpaused)
      gazebo::common::Time::MSleep(100);
    if (print <= 0)
    {
      std::cout<<"Updating world still going."<<std::endl;
      print=printCnt;
    }
    --print;*/
  }
  server->Stop();
  return true;
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  if (argc < 3)
  {
    std::cerr << "Usage: "<<argv[0]
              << " <world file> <list of physics engines>"<<std::endl;
    std::cerr << "<list of physics engines> can contain the following: "
              << "[ode, bullet, dart, simbody] "<<std::endl;
    return false;
  }

  // now, load the worlds as given in command line arguments
  // with the different engines given
  std::string worldfile = argv[1];

  std::vector<std::string> selectedEngines;
  for (int i = 2; i < argc; ++i)
    selectedEngines.push_back(argv[i]);

  bool loadMirror = true;
  bool enforceContactCalc=false;
  bool allowControlViaMirror = true;
  Run(worldfile, selectedEngines, loadMirror, allowControlViaMirror,
      enforceContactCalc);
}


#endif
