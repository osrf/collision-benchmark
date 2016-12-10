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
 * Date: May 2016
 */
#include <collision_benchmark/GazeboMirrorWorld.hh>
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
  mirrorWorld->EnablePhysicsEngine(false);
  std::cout<<"GazeboMirrorWorld constructed."<<std::endl;
}

GazeboMirrorWorld::~GazeboMirrorWorld()
{
}

gazebo::physics::WorldPtr GazeboMirrorWorld::GetMirrorWorld() const
{
  return mirrorWorld;
}

GazeboMirrorWorld::OriginalWorldPtr GazeboMirrorWorld::GetOriginalWorld() const
{
  return originalWorld;
}

void GazeboMirrorWorld::SetOriginalWorld(const OriginalWorldPtr& _originalWorld)
{
  originalWorld = _originalWorld;
}

void GazeboMirrorWorld::ClearModels()
{
  if (mirrorWorld) collision_benchmark::ClearModels(mirrorWorld);
}

void GazeboMirrorWorld::Sync()
{
  assert(originalWorld);
  gazebo::physics::WorldState origState = originalWorld->GetWorldState();
  collision_benchmark::SetWorldState(mirrorWorld, origState);
}

void GazeboMirrorWorld::Update(int iter)
{
  throw new gazebo::common::Exception(__FILE__,__LINE__,"Implement me");
}
