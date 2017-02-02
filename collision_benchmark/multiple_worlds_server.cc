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
#include <collision_benchmark/GazeboWorldState.hh>
#include <collision_benchmark/GazeboMirrorWorld.hh>
#include <collision_benchmark/GazeboPhysicsWorld.hh>
#include <collision_benchmark/boost_std_conversion.hh>
#include <collision_benchmark/GazeboHelpers.hh>
#include <collision_benchmark/WorldManager.hh>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

using collision_benchmark::PhysicsWorldBaseInterface;
using collision_benchmark::PhysicsWorld;
using collision_benchmark::GazeboPhysicsWorld;
using collision_benchmark::MirrorWorld;
using collision_benchmark::GazeboMirrorWorld;
using collision_benchmark::WorldManager;

typedef PhysicsWorldBaseInterface<gazebo::physics::WorldState> GzPhysicsWorldBase;
typedef WorldManager<gazebo::physics::WorldState> GzWorldManager;
typedef GazeboPhysicsWorld::PhysicsWorldTypes GazeboPhysicsWorldTypes;
typedef PhysicsWorld<GazeboPhysicsWorldTypes> GzPhysicsWorld;

// loads the mirror world. This should be loaded before all other Gazebo worlds, so that
// gzclient connects to this one.
GazeboMirrorWorld::Ptr setupMirrorWorld()
{
    std::cout << "Setting up mirror world..." << std::endl;
    std::string mirrorName = "mirror_world";
    gazebo::physics::WorldPtr _mirrorWorld = collision_benchmark::LoadWorld("worlds/empty.world", mirrorName);
    if (!_mirrorWorld)
    {
        std::cerr<<"Could not load mirror world"<<mirrorName<<std::endl;
        return GazeboMirrorWorld::Ptr();
    }

    std::cout<<"Creating mirror world object."<<std::endl;
    GazeboMirrorWorld::Ptr mirrorWorld(new GazeboMirrorWorld(_mirrorWorld));
    return mirrorWorld;
}

// Main method to play the test, later to be replaced by a dedicated structure (without command line arument params)
bool Run(int argc, char **argv)
{
  if (argc < 3)
  {
    std::cerr<<"Usage: "<<argv[0]<<" <world file> <list of physics engines>"<<std::endl;
    std::cerr<<"<list of physics engines> can contain the following: [ode, bullet, dart, simbody] "<<std::endl;
    return false;
  }

  // first, load the mirror world (it has to be loaded first for gzclient to connect to it)

  GazeboMirrorWorld::Ptr mirrorWorld = setupMirrorWorld();
  if(!mirrorWorld)
  {
    std::cerr<<"Could not load mirror world."<<std::endl;
    return false;
  }

  // now, load the worlds as given in command line arguments with the different engines given
  std::string worldfile = argv[1];

  std::vector<std::string> selectedEngines;
  for (int i = 2; i < argc; ++i)
    selectedEngines.push_back(argv[i]);

  std::map<std::string,std::string> physicsEngines = collision_benchmark::getPhysicsSettingsSdfFor(selectedEngines);

  std::cout << "Loading world " << worldfile << " with "<<physicsEngines.size()<<" engines."<<std::endl;

  GzWorldManager worldManager(mirrorWorld);
  int i=1;
  for (std::map<std::string,std::string>::iterator it = physicsEngines.begin(); it!=physicsEngines.end(); ++it, ++i)
  {
    std::string engine = it->first;
    std::string physicsSDF = it->second;

    std::stringstream _worldname;
    _worldname << "world_" << i << "_" << engine;
    std::string worldname=_worldname.str();

    std::cout << "Loading with physics engine " << engine << " (named as '" << worldname << "')" << std::endl;

    std::cout<<"Loading physics from "<<physicsSDF<<std::endl;
    sdf::ElementPtr physics = collision_benchmark::GetPhysicsFromSDF(physicsSDF);
    if (!physics.get())
    {
      std::cerr << "Could not get phyiscs engine from " << physicsSDF << std::endl;
      continue;
    }

    // std::cout<<"Physics: "<<physics->ToString("")<<std::endl;
    std::cout<<"Loading world from "<<worldfile<<std::endl;
    gazebo::physics::WorldPtr gzworld = collision_benchmark::LoadWorldFromFile(worldfile, worldname, physics);

    if (!gzworld)
    {
      std::cout<<"Error loading world "<<worldfile<<std::endl;
      return false;
    }
    // Create the GazeboPhysicsWorld object
    bool enforceContactCalc=true;
    GazeboPhysicsWorld::Ptr gzPhysicsWorld(new GazeboPhysicsWorld(enforceContactCalc));
    gzPhysicsWorld->SetWorld(collision_benchmark::to_std_ptr<gazebo::physics::World>(gzworld));
    worldManager.AddPhysicsWorld(gzPhysicsWorld);
  }

//  worldManager.SetDynamicsEnabled(false);

  std::cout << "Now start gzclient if you would like to view the test. Press [Enter] to continue."<<std::endl;
  getchar();
  std::cout << "Now starting to update worlds."<<std::endl;
  while(true)
  {
    int numSteps=1;
    worldManager.Update(numSteps);
    // gazebo::common::Time::MSleep(100);
    GzWorldManager::PhysicsWorldBasePtr mirroredWorld = worldManager.GetMirroredWorld();
    GzPhysicsWorld::Ptr mirroredWorldCast = std::dynamic_pointer_cast<GzPhysicsWorld>(mirroredWorld);
    assert(mirroredWorldCast);
    std::vector<GzPhysicsWorld::ContactInfoPtr> contacts = mirroredWorldCast->GetContactInfo();
    std::cout<<"Number of contacts: "<<contacts.size()<<std::endl;
    getchar();
  }
  return true;
}


/////////////////////////////////////////////////
int main(int argc, char **argv)
{
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

  Run(argc, argv);

  std::cout << "Shutting down..." <<std::endl;

  gazebo::shutdown();

  std::cout << "Multi-world server ended." << std::endl;
  return 0;
}
