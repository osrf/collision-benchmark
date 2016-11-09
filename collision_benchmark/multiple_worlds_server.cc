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
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

//#define WORLDSTATES_QUIET

void PrintWorldState(const gazebo::physics::WorldPtr world)
{
  std::cout << "## State of world " << world->GetName() << std::endl;
  gazebo::physics::WorldState _state(world);
  std::cout << _state << std::endl;
}

void PrintWorldStates(const std::vector<gazebo::physics::WorldPtr>& worlds)
{
  std::cout << "## World states ###" << std::endl;
  for (std::vector<gazebo::physics::WorldPtr>::const_iterator w = worlds.begin();
      w != worlds.end(); ++w)
  {
    PrintWorldState(*w);
  }
  std::cout << "#####" << std::endl;
}

/// Convenience function to call gazebo::runWorld() on several worlds
void RunWorlds(int iter, const std::vector<gazebo::physics::WorldPtr>& worlds)
{
#ifndef WORLDSTATES_QUIET
  std::cout << "##### States of all worlds before running:" << std::endl;
  PrintWorldStates(worlds);
#endif
  int steps = 1;
  for (unsigned int i = 0; i < iter; ++i)
  {
    // std::cout << "##### Running world(s), iter=" << i << std::endl;
    for (std::vector<gazebo::physics::WorldPtr>::const_iterator w = worlds.begin();
        w != worlds.end(); ++w)
    {
      gazebo::physics::WorldPtr world = *w;
      std::cout<<"World "<<world->GetName()<<" physics engine: "<<world->GetPhysicsEngine()->GetType()<<std::endl;
      // Run simulation for given number of steps.
      // This method calls world->RunBlocking(_iterations);
      gazebo::runWorld(world, steps);
    }
  }
#ifndef WORLDSTATES_QUIET
  std::cout << "##### States of all worlds after running:" << std::endl;
  PrintWorldStates(worlds);
#endif
}

// Main method to play the test, later to be replaced by a dedicated structure (without command line arument params)
bool PlayTest(int argc, char **argv)
{
  if (argc < 3)
  {
    std::cerr<<"Usage: "<<argv[0]<<" <number iterations> <list of world filenames>"<<std::endl;
    return false;
  }

  // list of worlds to be loaded
  std::set<collision_benchmark::Worldfile> worldsToLoad;

  int numIters = atoi(argv[1]);

  for (int i = 2; i < argc; ++i)
  {
    std::string worldfile = std::string(argv[i]);
    std::stringstream worldname;
    worldname << "world_" << i - 1;
    worldsToLoad.insert(collision_benchmark::Worldfile(worldfile,worldname.str()));
  }

  std::cout << "Loading worlds..." << std::endl;
  std::vector<gazebo::physics::WorldPtr> worlds = collision_benchmark::LoadWorlds(worldsToLoad);
  if (worlds.size()!=worldsToLoad.size())
  {
    std::cerr << "Could not load worlds." << std::endl;
    return false;
  }

  // Go through all worlds and print info
  for (int i = 0; i < worlds.size(); ++i)
  {
    RunWorlds(numIters, worlds);
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
