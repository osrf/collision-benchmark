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
#include <collision_benchmark/GazeboWorldState.hh>
#include <collision_benchmark/GazeboPhysicsWorld.hh>
#include <collision_benchmark/boost_std_conversion.hh>
#include <collision_benchmark/GazeboTopicForwardingMirror.hh>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

using collision_benchmark::MirrorWorld;
using collision_benchmark::GazeboTopicForwardingMirror;
using collision_benchmark::GazeboPhysicsWorld;

/**
 * Convenience function to call PhysicsWorld::Update(1) on several worlds \e iter times
 * \param iter number of iterations (world updates)
 * \param worlds all worlds that have to be updated
 * \param mirrorWorld optional: if not NULL, the method GazeboMirrorWorld::Sync() is called at each iteration
 */
void RunWorlds(int iter, const std::vector<GazeboPhysicsWorld::Ptr>& worlds, const MirrorWorld::Ptr& mirrorWorld=MirrorWorld::Ptr(NULL))
{
  for (unsigned int i = 0; i < iter; ++i)
  {
    static const int steps = 1;
    // std::cout << "##### Running world(s), iter=" << i << std::endl;
    for (std::vector<GazeboPhysicsWorld::Ptr>::const_iterator w = worlds.begin();
        w != worlds.end(); ++w)
    {
      GazeboPhysicsWorld::Ptr world = *w;
      // std::cout<<"Running "<<steps<<" steps for world "<<world->GetName()<<", physics engine: "
      //     <<world->GetWorld()->Physics()->GetType()<<std::endl;
      world->Update(1);
    }

    if (mirrorWorld)
    {
      mirrorWorld->Sync();
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

  gazebo::common::Console::SetQuiet(false);

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

  // first, load the mirror world (it has to be loaded first for gzclient to connect to it,
  // see transport::Node::Init(), called from gui::MainWindow constructor with empty string)
  GazeboTopicForwardingMirror::Ptr mirrorWorld(new GazeboTopicForwardingMirror("mirror_world"));

  if(!mirrorWorld)
  {
    std::cerr<<"Could not load mirror world."<<std::endl;
    return false;
  }

  std::cout << "Loading worlds..." << std::endl;
  std::vector<gazebo::physics::WorldPtr> gzworlds = collision_benchmark::LoadWorlds(worldsToLoad);
  if (gzworlds.size()!=worldsToLoad.size())
  {
    std::cerr << "Could not load worlds." << std::endl;
    return false;
  }

  // the worlds wrapped in the PhysicsWorld interface. Same size as gzworlds.
  // XXX TODO Eventually gzworlds is going to become obsolete and exchanged by objects of PhysicsWorld,
  // it is only used still in this test program.
  std::vector<GazeboPhysicsWorld::Ptr> worlds;
  for (int i = 0; i < gzworlds.size(); ++i)
  {
    GazeboPhysicsWorld::Ptr gzPhysicsWorld(new GazeboPhysicsWorld(false));
    gzPhysicsWorld->SetWorld(collision_benchmark::to_std_ptr<gazebo::physics::World>(gzworlds[i]));
    worlds.push_back(gzPhysicsWorld);
  }
  assert(worlds.size() == gzworlds.size());

  // Go through all worlds and print info
  bool printState = false;
  if (printState)
  {
    std::cout << "##### States of all worlds before running:" << std::endl;
    collision_benchmark::PrintWorldStates(gzworlds);
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
      RunWorlds(numIters, worlds, mirrorWorld);
  }

  if (printState)
  {
    std::cout << "##### States of all worlds after running:" << std::endl;
    collision_benchmark::PrintWorldStates(gzworlds);
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
