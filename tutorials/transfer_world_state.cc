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
 * Date: January 2017
 */

#include <collision_benchmark/PhysicsWorldInterfaces.hh>
#include <collision_benchmark/GazeboPhysicsWorld.hh>
#include <collision_benchmark/GazeboStateCompare.hh>

#include <gazebo/gazebo.hh>

using collision_benchmark::PhysicsWorldBaseInterface;
using collision_benchmark::PhysicsWorld;
using collision_benchmark::GazeboPhysicsWorld;
using collision_benchmark::GazeboStateCompare;

// fake command line argument
const char * g_fakeProgramName="tutorial";
void StartGzServer()
{
  // we want to use the command line parameters of our main program for
  // other purposes, so we will need to set fake parameters for starting
  // the Gazebo server.
  // Cast away constness required but not dangerous in this case
  // as we won't chang eg_fakeProgramName.
  gazebo::setupServer(1, (char**)&g_fakeProgramName);
}

void ShutdownGzServer()
{
  gazebo::shutdown();
}

int main(int argc, char** argv)
{
  // start up the Gazebo server
  StartGzServer();

  // set the number of iterations after which we will set the state of
  // the first world loaded to the same state of the second world.
  int switchAfterIter = 1000;
  // run the test for this many iterations
  int runForIter = 10000;
  // set this to false to skip a comparison test in the main loop below.
  // More about this later.
  bool doTestComparison = false;
  if (argc >= 2)
  {
    switchAfterIter = atoi(argv[1]);
    if (argc >=3)
      runForIter = atoi(argv[2]);
  }

  // typedef for the basic interface depending only on the gazebo world
  // state type.
  // Create this for convenience so you don't have to type the full
  // template type each time.
  typedef collision_benchmark::PhysicsWorldStateInterface
            < gazebo::physics::WorldState > GzPhysicsWorldStateInterface;

  // two worlds with the full interface of collision_engine::PhysicsEngineWorld.
  // (only required for instantiating other interfaces next)
  GazeboPhysicsWorld::Ptr gazeboWorld1(new GazeboPhysicsWorld());
  GazeboPhysicsWorld::Ptr gazeboWorld2(new GazeboPhysicsWorld());

  // pointers to access the worlds via the GzPhysicsWorldStateInterface
  // interface depending only on the gazebo state.
  GzPhysicsWorldStateInterface::Ptr gzStateWorld1(gazeboWorld1);
  GzPhysicsWorldStateInterface::Ptr gzStateWorld2(gazeboWorld2);

  // pointers to access the worlds via the basic interface
  // PhysicsWorldBaseInterface.
  PhysicsWorldBaseInterface::Ptr world1(gazeboWorld1);
  PhysicsWorldBaseInterface::Ptr world2(gazeboWorld2);

  //////////////////////////////////////
  // From here on, code only depends on
  // PhysicsWorldBaseInterface and
  // GzPhysicsWorldStateInterface
  /////////////////////////////////////

  // Load the empty world.
  //
  // *** The first world loaded will be the world which is displayed
  // *** in the gzclient, so that will be this world.
  //
  // While we will use the basic interface PhysicsWorldBaseInterface for loading
  // worlds, the actual loading of the world will only work for
  // implementations which support SDF files (which is the rubble world), and
  // for implementations which look for the files in the GAZEBO_RESOURCE_PATH
  // (or we would need to specify the full path here).
  // Because we instantiate the world with a GazeboPhysicsWorld, it will work.

  if (world1->LoadFromFile("worlds/empty.world")
      != collision_benchmark::SUCCESS)
  {
    std::cerr << "Could not load empty world" << std::endl;
    return 1;
  }

  // Disable the physics engine in the first world:
  // We will set the state of this world manually in this example.
  // The physics engine can be disabled to enforce that the state is always the
  // state we set manually (otherwise the world would jitter between when we
  // set the state and when its physics engine reacts to it).
  world1->SetDynamicsEnabled(false);

  // In the second world, load the rubble world. Name it 'rubble', because
  // the name in the SDF file is 'default', just as for the empty world loaded
  // before... and loading two worlds with the same name won't work.
  if (world2->LoadFromFile("worlds/rubble.world", "rubble")
      != collision_benchmark::SUCCESS)
  {
    std::cerr << "Could not load rubble world" << std::endl;
    return 1;
  }

  // print a messsage to notify you that you can now start the client
  // to view the world.
  std::cout << "You may now start gzclient to view the first world. "
           <<"Press [Enter] to start the simulation after gzclient "
           <<"has started." << std::endl;
  getchar();

  // Now run the worlds until you hit Ctrl+C. Starting from *after*
  // <switchAfterIter> iterations, we will set world1 to the state of world2,
  // so world1 should display the rubble world as well after a while.
  unsigned int iter = 0;
  while (iter < runForIter)
  {
    if (iter >= switchAfterIter)
    {
      if (iter == switchAfterIter)
        std::cout << "Now starting to set the world to the rubble state"
          << std::endl;

      // get the rubble world state
      gazebo::physics::WorldState rubbleState = gzStateWorld2->GetWorldState();
      // set the first world to the same state
      gzStateWorld1->SetWorldState(rubbleState);

      // do a test: the states of both worlds should be the same,
      // or something went wrong!
      // The class GazeboStateCompare can help us with this.
      if (doTestComparison)
      {
        // First get the new state of the first world:
        gazebo::physics::WorldState newState1 = gzStateWorld1->GetWorldState();
        // Now, set the tolerances we want to use for comparing.
        // We will just use the default tolerances.
        GazeboStateCompare::Tolerances t =
          GazeboStateCompare::Tolerances::Default;
        // However, in the tolerances we disable the check for the dynamic
        // properties, because we disabled the physics engine in world1.
        // The accelerations of the links will not be correct.
        // Disabling dynamics checks skips the check of accelerations,
        // velocities and wrenches.
        t.CheckDynamics = false;
        if (!GazeboStateCompare::Equal(newState1, rubbleState, t))
        {
          std::cerr << "There is a problem: Target state was not set to "
                    << "the rubble world!" << std::endl;
          /*std::cerr << "Rubble state:" << std::endl;
          std::cerr << rubbleState << std::endl;
          std::cerr << "Cloned state:" << std::endl;
          std::cerr << newState1<<std::endl;*/
        }
      }
    }

    // update both worlds with one step
    world1->Update(1);
    world2->Update(1);
    ++iter;
  }

  // End of test. Shut down the server.

  std::cout << "Test is finished, world won't be updated "
            << "in gzclient any more. " << std::endl;
  std::cout << "Set higher iteration count via command line parameters "
            << "if you wish to run the test for longer." << std::endl;
  std::cout << "Shutting down server. Bye bye." << std::endl;

  ShutdownGzServer();

  return 0;
}

