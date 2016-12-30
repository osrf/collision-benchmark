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
/* Desc: World adaptor mirroring another physics::World
 * Author: Jennifer Buehler
 * Date: December 2016
 */
#include <collision_benchmark/GazeboMirrorWorld.hh>
#include <collision_benchmark/GazeboPhysicsWorld.hh>
#include <collision_benchmark/GazeboStateCompare.hh>
#include <collision_benchmark/GazeboWorldState.hh>
#include <collision_benchmark/GazeboHelpers.hh>
#include <gazebo/physics/PhysicsIface.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/ContactManager.hh>

using collision_benchmark::GazeboMirrorWorld;

GazeboMirrorWorld::GazeboMirrorWorld(gazebo::physics::WorldPtr& mirrorWorld_):
  mirrorWorld(mirrorWorld_)
{
  assert(mirrorWorld);
  mirrorWorld->SetPhysicsEnabled(false);
}

GazeboMirrorWorld::~GazeboMirrorWorld()
{
}

gazebo::physics::WorldPtr GazeboMirrorWorld::GetMirrorWorld() const
{
  return mirrorWorld;
}

void GazeboMirrorWorld::ClearModels()
{
  if (mirrorWorld) collision_benchmark::ClearModels(mirrorWorld);
}

void GazeboMirrorWorld::Sync()
{
  assert(originalWorld);

/*
  XXX TODO support Pause and Step functions, though this will have to be done for all worlds,
  so probably rather in WorldManager with a common interface... not of high priority now though.
  Best is probably to re-write mirror world and receive all the Gui commands from a dedicated interface,
  instead of having the whole cloned world.
  GazeboPhysicsWorld::Ptr gzOrigWorld = std::dynamic_pointer_cast<GazeboPhysicsWorld>(originalWorld);
  bool isPaused=false;
  if (gzOrigWorld)
  {
    // set mirror world to paused if the original world is paused too. Only supported for other gazebo worlds.
    bool isPaused = gzOrigWorld->GetWorld()->IsPaused();
    std::cout<<"Paused state: "<<gzOrigWorld->GetWorld()->IsPaused()<<std::endl;;
    mirrorWorld->SetPaused(isPaused);
  }
  if (isPaused) return;*/

  gazebo::physics::WorldState origState = originalWorld->GetWorldState();
  collision_benchmark::SetWorldState(mirrorWorld, origState);
#ifdef DEBUG
  gazebo::physics::WorldState _currentState(mirrorWorld);
  GazeboStateCompare::Tolerances t=GazeboStateCompare::Tolerances::CreateDefault(1e-03);
  t.CheckDynamics=false; // don't check dynamics because the physics engine of the mirror world is disabled
  if (!GazeboStateCompare::Equal(_currentState, origState, t))
  {
    std::cerr<<"Target state was not set as supposed to!!"<<std::endl;
  }
#endif
}

void GazeboMirrorWorld::Update(int iter)
{
  gazebo::runWorld(mirrorWorld, iter);
}
