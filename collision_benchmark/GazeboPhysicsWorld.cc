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
 * Date: November 2016
 */
#include <collision_benchmark/GazeboPhysicsWorld.hh>
#include <collision_benchmark/GazeboWorldState.hh>
#include <collision_benchmark/GazeboStateCompare.hh>
#include <collision_benchmark/GazeboHelpers.hh>
#include <collision_benchmark/WorldLoader.hh>
#include <collision_benchmark/boost_std_conversion.hh>

#include <gazebo/physics/physics.hh>
//#include <gazebo/physics/PhysicsIface.hh>

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

bool GazeboPhysicsWorld::WaitForNamespace(const gazebo::physics::WorldPtr& gzworld, float maxWait, float waitSleep)
{
  std::string worldNamespace = gzworld->GetName();

  // wait for namespace to be loaded, to make sure the order of namespaces maintained
  // in the transport system eventually will correspond to the same order of the worlds
  if (!collision_benchmark::WaitForNamespace(worldNamespace, maxWait, waitSleep))
  {
    std::cerr << "Namespace of world '" << worldNamespace << "' was not loaded" << std::endl;
    return false;
  }
  return true;
}


GazeboPhysicsWorld::OpResult GazeboPhysicsWorld::LoadFromSDF(const sdf::ElementPtr& sdf, const std::string& worldname)
{
  gazebo::physics::WorldPtr gzworld = collision_benchmark::LoadWorldFromSDF(sdf, worldname);

  if (!gzworld)
    return GazeboPhysicsWorld::FAILED;

  if (OnLoadWaitForNamespace && !WaitForNamespace(gzworld, OnLoadMaxWaitForNamespace, OnLoadWaitForNamespaceSleep))
    return GazeboPhysicsWorld::FAILED;

  SetWorld(collision_benchmark::to_std_ptr<gazebo::physics::World>(gzworld));

  return GazeboPhysicsWorld::SUCCESS;
}

GazeboPhysicsWorld::OpResult GazeboPhysicsWorld::LoadFromFile(const std::string& filename, const std::string& worldname)
{
  gazebo::physics::WorldPtr gzworld = collision_benchmark::LoadWorldFromFile(filename, worldname);

  if (!gzworld)
    return GazeboPhysicsWorld::FAILED;

  if (OnLoadWaitForNamespace && !WaitForNamespace(gzworld, OnLoadMaxWaitForNamespace, OnLoadWaitForNamespaceSleep))
    return GazeboPhysicsWorld::FAILED;

  SetWorld(collision_benchmark::to_std_ptr<gazebo::physics::World>(gzworld));

  return GazeboPhysicsWorld::SUCCESS;
}

GazeboPhysicsWorld::OpResult GazeboPhysicsWorld::LoadFromString(const std::string& str, const std::string& worldname)
{
  throw new gazebo::common::Exception(__FILE__,__LINE__,"Implement me");
}

GazeboPhysicsWorld::ModelLoadResult GazeboPhysicsWorld::AddModelFromFile(const std::string& filename, const std::string& modelname)
{
  throw new gazebo::common::Exception(__FILE__,__LINE__,"Implement me");
}

GazeboPhysicsWorld::ModelLoadResult GazeboPhysicsWorld::AddModelFromString(const std::string& str, const std::string& modelname)
{
  throw new gazebo::common::Exception(__FILE__,__LINE__,"Implement me");
}

GazeboPhysicsWorld::ModelLoadResult GazeboPhysicsWorld::AddModelFromSDF(const sdf::ElementPtr& sdf, const std::string& modelname)
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
  collision_benchmark::ClearModels(world);
}

GazeboPhysicsWorld::WorldState GazeboPhysicsWorld::GetWorldState() const
{
  gazebo::physics::WorldState s(world);
  return s;
}

GazeboPhysicsWorld::WorldState GazeboPhysicsWorld::GetWorldStateDiff(const WorldState& other) const
{
  gazebo::physics::WorldState diffState = other - GetWorldState();
  // std::cout << "Diff state: " << std::endl << diffState << std::endl;
  return diffState;
}

GazeboPhysicsWorld::OpResult GazeboPhysicsWorld::SetWorldState(const WorldState& state, bool isDiff)
{
  collision_benchmark::SetWorldState(world, state);

#ifdef DEBUG
  gazebo::physics::WorldState _currentState(world);
  GazeboStateCompare::Tolerances t=GazeboStateCompare::Tolerances::CreateDefault(1e-03);
  if (!GazeboStateCompare::Equal(_currentState, state, t))
  {
    std::cerr<<"Target state was not set as supposed to!!"<<std::endl;
  }
#endif

  return PhysicsWorldBase::SUCCESS;
}

void GazeboPhysicsWorld::Update(int steps)
{
  std::cout<<"Running "<<steps<<" steps for world "<<world->GetName()<<", physics engine: "<<world->GetPhysicsEngine()->GetType()<<std::endl;
  // Run simulation for given number of steps.
  // This method calls world->RunBlocking();
  gazebo::runWorld(world,steps);
}


std::string GazeboPhysicsWorld::GetName() const
{
  return world->GetName();
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

GazeboPhysicsWorld::RefResult GazeboPhysicsWorld::SetWorld(const WorldPtr& _world)
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
