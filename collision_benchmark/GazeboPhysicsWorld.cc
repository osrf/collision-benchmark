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
/**
 * Desc: PhysicsEngineWorld implementation for Gazebo which wraps a gazebo::physics::World object
 * Author: Jennifer Buehler
 * Date: May 2016
 */
#include <collision_benchmark/GazeboPhysicsWorld.hh>
//#include <collision_benchmark/GazeboWorldState.hh>
#include <collision_benchmark/WorldLoader.hh>
#include <collision_benchmark/boost_std_conversion.h>

#include <gazebo/physics/physics.hh>

using collision_benchmark::GazeboPhysicsWorld;

GazeboPhysicsWorld::GazeboPhysicsWorld()
{
  std::cout<<"Constructor GazeboPhysicsWorld"<<std::endl;
}

GazeboPhysicsWorld::~GazeboPhysicsWorld()
{
}

bool GazeboPhysicsWorld::SupportsSDF() const
{
  return true;
}

GazeboPhysicsWorld::OpResult GazeboPhysicsWorld::LoadFromSDF(const sdf::ElementPtr& sdf)
{
  throw new gazebo::common::Exception(__FILE__,__LINE__,"Implement me");
}

GazeboPhysicsWorld::OpResult GazeboPhysicsWorld::LoadFromFile(const std::string& filename)
{
  throw new gazebo::common::Exception(__FILE__,__LINE__,"Implement me");
}

GazeboPhysicsWorld::OpResult GazeboPhysicsWorld::LoadFromString(const std::string& str)
{
  throw new gazebo::common::Exception(__FILE__,__LINE__,"Implement me");
}

GazeboPhysicsWorld::ModelLoadResult GazeboPhysicsWorld::AddModelFromFile(const std::string& filename)
{
  throw new gazebo::common::Exception(__FILE__,__LINE__,"Implement me");
}

GazeboPhysicsWorld::ModelLoadResult GazeboPhysicsWorld::AddModelFromString(const std::string& str)
{
  throw new gazebo::common::Exception(__FILE__,__LINE__,"Implement me");
}

GazeboPhysicsWorld::ModelLoadResult GazeboPhysicsWorld::AddModelFromSDF(const sdf::ElementPtr& sdf)
{
  throw new gazebo::common::Exception(__FILE__,__LINE__,"Implement me");
}

bool GazeboPhysicsWorld::SupportsShapes() const
{
  return true;
}

GazeboPhysicsWorld::ModelLoadResult GazeboPhysicsWorld::AddModelFromShape(const ShapePtr& shape, const ShapePtr * collShape)
{
  throw new gazebo::common::Exception(__FILE__,__LINE__,"Implement me");
}

std::vector<GazeboPhysicsWorld::ModelID> GazeboPhysicsWorld::GetAllModelIDs() const
{
  throw new gazebo::common::Exception(__FILE__,__LINE__,"Implement me");
}

bool GazeboPhysicsWorld::RemoveModel(const ModelID& id)
{
  throw new gazebo::common::Exception(__FILE__,__LINE__,"Implement me");
}

void GazeboPhysicsWorld::Clear()
{
    bool pauseState = world->IsPaused();
    world->SetPaused(true);
    // XXX TODO CHECK: this does not clear the lights
    world->ClearModels();
    // need to reset physics engine in order to stop it
    // from publishing old contact points
    gazebo::physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
    if (physics) physics->GetContactManager()->Clear();
    world->SetPaused(pauseState);
}

GazeboPhysicsWorld::WorldState GazeboPhysicsWorld::GetWorldState() const
{
  throw new gazebo::common::Exception(__FILE__,__LINE__,"Implement me");
}

GazeboPhysicsWorld::WorldState GazeboPhysicsWorld::GetWorldStateDiff(const WorldState& other) const
{
  throw new gazebo::common::Exception(__FILE__,__LINE__,"Implement me");
}

GazeboPhysicsWorld::OpResult GazeboPhysicsWorld::SetWorldState(const WorldState& state, bool isDiff)
{
  // XXX TODO To be added in a later PR
  /**
  collision_benchmark::SetWorldState(world, state);
  return PhysicsWorldBase::SUCCESS;
  */
  return PhysicsWorldBase::FAILED;
}

void GazeboPhysicsWorld::Update(int steps)
{
  gazebo::runWorld(world,steps);
}

bool GazeboPhysicsWorld::SupportsContacts() const
{
  throw new gazebo::common::Exception(__FILE__,__LINE__,"Implement me");
}

std::vector<GazeboPhysicsWorld::ContactInfoPtr> GazeboPhysicsWorld::GetContactInfo() const
{
  throw new gazebo::common::Exception(__FILE__,__LINE__,"Implement me");
}

std::vector<GazeboPhysicsWorld::ContactInfoPtr> GazeboPhysicsWorld::GetContactInfo(const ModelID& m1, const ModelID& m2) const
{
  throw new gazebo::common::Exception(__FILE__,__LINE__,"Implement me");
}

const std::vector<GazeboPhysicsWorld::ContactPtr>& GazeboPhysicsWorld::GetContacts() const
{
  throw new gazebo::common::Exception(__FILE__,__LINE__,"Implement me");
}

const std::vector<GazeboPhysicsWorld::ContactPtr> GazeboPhysicsWorld::GetContacts(const ModelID& m1, const ModelID& m2) const
{
  throw new gazebo::common::Exception(__FILE__,__LINE__,"Implement me");
}

bool GazeboPhysicsWorld::IsAdaptor() const
{
  return true;
}

GazeboPhysicsWorld::RefResult GazeboPhysicsWorld::SetWorld(WorldPtr& _world)
{
  world = collision_benchmark::to_boost_ptr<World>(_world);
  return GazeboPhysicsWorld::REFERENCED;
}

GazeboPhysicsWorld::WorldPtr GazeboPhysicsWorld::GetWorld() const
{
  return collision_benchmark::to_std_ptr<World>(world);
}

GazeboPhysicsWorld::ModelPtr GazeboPhysicsWorld::GetModel(const ModelID& model) const
{
  throw new gazebo::common::Exception(__FILE__,__LINE__,"Implement me");
}

GazeboPhysicsWorld::PhysicsEnginePtr GazeboPhysicsWorld::GetPhysicsEngine() const
{
  if (world) return collision_benchmark::to_std_ptr<GazeboPhysicsWorld::PhysicsEngine>(world->GetPhysicsEngine());
  return PhysicsEnginePtr();
}
