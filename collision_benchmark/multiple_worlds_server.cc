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

using collision_benchmark::PhysicsWorldBase;
using collision_benchmark::PhysicsWorld;
using collision_benchmark::GazeboPhysicsWorld;
using collision_benchmark::MirrorWorld;
using collision_benchmark::GazeboMirrorWorld;
using collision_benchmark::WorldManager;

typedef PhysicsWorldBase<gazebo::physics::WorldState> GzPhysicsWorldBase;
typedef WorldManager<gazebo::physics::WorldState> GzWorldManager;


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

// returns a table with the filenames to use for each of the physcis engines. Only supported
// engines are returned. Key of the returned map is the engine name as given in \e engines,
// value is the path to the SDF file with the physics settings
// \param engines can contain "ode", "bullet", "dart", "simbody"
std::map<std::string,std::string> getPhysicsSettingsSdfFor(const std::vector<std::string>& engines)
{
  std::map<std::string, std::string> physics_filenames;
  std::set<std::string> supported_engines=collision_benchmark::GetSupportedPhysicsEngines();

  for (std::vector<std::string>::const_iterator eit=engines.begin(); eit!=engines.end(); ++eit)
  {
    std::string e=*eit;
    if (!supported_engines.count(*eit)) continue;

    if (e=="bullet")
      physics_filenames["bullet"]="physics_settings/bullet_default.sdf";
    else if (e=="dart")
      physics_filenames["dart"]="physics_settings/dart_default.sdf";
    else if (e=="ode")
      physics_filenames["ode"]="physics_settings/ode_default.sdf";
    else if (e=="simbody")
      // XXX TODO add the empty_simbody.world file
      physics_filenames["simbody"] = "../physics_settings/simbody_default.world";
  }
  return physics_filenames;
}

// Main method to play the test, later to be replaced by a dedicated structure (without command line arument params)
bool PlayTest(int argc, char **argv)
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

  std::map<std::string,std::string> physicsEngines = getPhysicsSettingsSdfFor(selectedEngines);

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
    GazeboPhysicsWorld::Ptr gzPhysicsWorld(new GazeboPhysicsWorld());
    gzPhysicsWorld->SetWorld(collision_benchmark::to_std_ptr<gazebo::physics::World>(gzworld));
    worldManager.AddPhysicsWorld(gzPhysicsWorld);
  }

  std::cout << "Now start gzclient if you would like to view the test. Press [Enter] to continue."<<std::endl;
  getchar();
  std::cout << "Now starting to update worlds."<<std::endl;
  while(true)
  {
    int numSteps=1;
    worldManager.Update(numSteps);
    // gazebo::common::Time::MSleep(100);
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

  PlayTest(argc, argv);

  std::cout << "Shutting down..." <<std::endl;

  gazebo::shutdown();

  std::cout << "Test ended." << std::endl;
  return 0;
}
