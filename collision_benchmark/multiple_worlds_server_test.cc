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

#include <collision_benchmark/WorldLoader.hh>
#include <collision_benchmark/PhysicsWorld.hh>
#include <collision_benchmark/GazeboWorldState.hh>
#include <collision_benchmark/GazeboMirrorWorld.hh>
#include <collision_benchmark/GazeboPhysicsWorld.hh>
#include <collision_benchmark/boost_std_conversion.hh>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

using collision_benchmark::PhysicsWorldBase;
using collision_benchmark::PhysicsWorld;
using collision_benchmark::GazeboPhysicsWorld;
using collision_benchmark::GazeboMirrorWorld;

typedef PhysicsWorldBase<gazebo::physics::WorldState> GzPhysicsWorldBase;

/**
 * Convenience function to call GazeboPhysicsWorld::Update(steps) on several worlds \e iter times
 * \param iter number of iterations (world updates)
 * \param worlds all worlds that have to be updated
 * \param mirrorWorld optional: if not NULL, the method GazeboMirrorWorld::Sync() is called at each iteration
 */
void RunWorlds(int iter, int steps, const std::vector<GzPhysicsWorldBase::Ptr>& worlds, const GazeboMirrorWorld::Ptr& mirrorWorld=GazeboMirrorWorld::Ptr(NULL))
{
  for (unsigned int i = 0; i < iter; ++i)
  {
    // std::cout << "##### Running world(s), iter=" << i << std::endl;
    for (std::vector<GzPhysicsWorldBase::Ptr>::const_iterator it = worlds.begin();
        it != worlds.end(); ++it)
    {
      GzPhysicsWorldBase::Ptr w=*it;
      w->Update(steps);
    }

    if (mirrorWorld)
    {
      mirrorWorld->Sync();
      // std::cout<<"Running mirror world. Paused? "<<mirrorWorld->GetMirrorWorld()->IsPaused()<<std::endl;
      mirrorWorld->Update(steps);
    }
  }
}

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
bool PlayTest(int argc, char **argv)
{
  if (argc < 3)
  {
    std::cerr<<"Usage: "<<argv[0]<<" <number iterations> <list of world filenames>"<<std::endl;
    return false;
  }

  // first, load the mirror world (it has to be loaded first for gzclient to connect to it)

  GazeboMirrorWorld::Ptr mirrorWorld = setupMirrorWorld();
  if(!mirrorWorld)
  {
    std::cerr<<"Could not load mirror world."<<std::endl;
    return false;
  }

  // now, load all worlds as given in command line arguments

  int numIters = atoi(argv[1]);

  std::cout << "Loading worlds..." << std::endl;
  std::vector<GzPhysicsWorldBase::Ptr> worlds;
  for (int i = 2; i < argc; ++i)
  {
    std::string worldfile = std::string(argv[i]);
    std::stringstream _worldname;
    _worldname << "world_" << i - 1;
    std::string worldname=_worldname.str();

    std::cout << "Loading world " << worldfile << " (named as '" << worldname << "')" << std::endl;
    GazeboPhysicsWorld::Ptr gzWorld(new GazeboPhysicsWorld());
    if (gzWorld->LoadFromFile(worldfile, worldname)!=GzPhysicsWorldBase::SUCCESS)
    {
      std::cerr << "Could not load world " << worldfile << std::endl;
      continue;
    }
    GzPhysicsWorldBase::Ptr newWorld(gzWorld);
    worlds.push_back(newWorld);
  }

  // Go through all worlds and print info
  bool printState = false;
  if (printState)
  {
    std::cout << "##### States of all worlds before running:" << std::endl;
    collision_benchmark::PrintWorldStates(worlds);
  }

  std::cout << "Now start gzclient if you would like to view the test. Press [Enter] to continue."<<std::endl;
  getchar();

  // Go through all worlds, mirroring each for the given number of iterations.
  for (int i = 0; i < worlds.size(); ++i)
  {
    std::cout << "+++++++++++++++++++++++++++++++++++++++++" << std::endl;
    std::cout << "Now mirroring " << worlds[i]->GetName() << std::endl;
    std::cout << "+++++++++++++++++++++++++++++++++++++++++" << std::endl;
    mirrorWorld->SetOriginalWorld(worlds[i]);
    const int steps=1;
    RunWorlds(numIters, steps, worlds, mirrorWorld);
  }

  if (printState)
  {
    std::cout << "##### States of all worlds after running:" << std::endl;
    collision_benchmark::PrintWorldStates(worlds);
  }

  return true;
}


/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  // Initialize gazebo.
  gazebo::setupServer(argc, argv);

  PlayTest(argc, argv);

  gazebo::shutdown();

  std::cout << "Test ended." << std::endl;
}
