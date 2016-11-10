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
// #include <collision_benchmark/GazeboWorldState.hh> // TODO to be added in next PR
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>


/// Convenience function to call gazebo::runWorld() on several worlds \e iter times
void RunWorlds(int iter, const std::vector<gazebo::physics::WorldPtr>& worlds)
{
  for (unsigned int i = 0; i < iter; ++i)
  {
    // std::cout << "##### Running world(s), iter=" << i << std::endl;
    for (std::vector<gazebo::physics::WorldPtr>::const_iterator w = worlds.begin();
        w != worlds.end(); ++w)
    {
      gazebo::physics::WorldPtr world = *w;
      std::cout<<"World "<<world->GetName()<<" physics engine: "<<world->GetPhysicsEngine()->GetType()<<std::endl;
      // Run simulation for given number of steps.
      // This method calls world->RunBlocking();
      int steps = 1;
      gazebo::runWorld(world, steps);
    }
  }
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
  std::vector<collision_benchmark::Worldfile> worldsToLoad;

  int numIters = atoi(argv[1]);

  for (int i = 2; i < argc; ++i)
  {
    std::string worldfile = std::string(argv[i]);
    std::stringstream worldname;
    worldname << "world_" << i - 1;
    worldsToLoad.push_back(collision_benchmark::Worldfile(worldfile,worldname.str()));
  }

  std::cout << "Loading worlds..." << std::endl;
  std::vector<gazebo::physics::WorldPtr> worlds = collision_benchmark::LoadWorlds(worldsToLoad);
  if (worlds.size()!=worldsToLoad.size())
  {
    std::cerr << "Could not load worlds." << std::endl;
    return false;
  }

  // Go through all worlds and print info
  /* TODO: This will be added in the next PR
  bool printState = false;
  if (printState)
  {
    std::cout << "##### States of all worlds before running:" << std::endl;
    collision_benchmark::PrintWorldStates(worlds);
  }*/

  RunWorlds(numIters, worlds);

  /*if (printState)
  {
    std::cout << "##### States of all worlds after running:" << std::endl;
    collision_benchmark::PrintWorldStates(worlds);
  }*/

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
