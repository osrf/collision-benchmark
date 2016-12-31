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
#include <collision_benchmark/GazeboWorldLoader.hh>
#include <collision_benchmark/boost_std_conversion.hh>

#include <gazebo/physics/physics.hh>

using collision_benchmark::GazeboPhysicsWorld;

GazeboPhysicsWorld::GazeboPhysicsWorld()
{
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
  std::string worldNamespace = gzworld->Name();

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
  gazebo::physics::WorldPtr gzworld = collision_benchmark::LoadWorldFromSDFString(str, worldname);

  if (!gzworld)
    return GazeboPhysicsWorld::FAILED;

  if (OnLoadWaitForNamespace && !WaitForNamespace(gzworld, OnLoadMaxWaitForNamespace, OnLoadWaitForNamespaceSleep))
    return GazeboPhysicsWorld::FAILED;

  SetWorld(collision_benchmark::to_std_ptr<gazebo::physics::World>(gzworld));

  return GazeboPhysicsWorld::SUCCESS;

}

GazeboPhysicsWorld::ModelLoadResult GazeboPhysicsWorld::AddModelFromFile(const std::string& filename, const std::string& modelname)
{
  sdf::ElementPtr sdfRoot = collision_benchmark::GetSDFElementFromFile(filename, "model", modelname);
  ModelLoadResult ret;
  ret.opResult=FAILED;
  if (!sdfRoot)
  {
    std::cerr<< " Could not get SDF for model in "<<filename<<std::endl;
    return ret;
  }

  ret = AddModelFromSDF(sdfRoot);
  return ret;
}

GazeboPhysicsWorld::ModelLoadResult GazeboPhysicsWorld::AddModelFromString(const std::string& str, const std::string& modelname)
{
  ModelLoadResult ret;
  ret.opResult=FAILED;

  std::string useStr=str;
  int checkSDF=collision_benchmark::isProperSDFString(useStr);
  if (checkSDF<0)
  {
    if (checkSDF==-2)
    {
      // std::cout<<"Wrapping extra SDF around string. "<<__FILE__<<std::endl;
      collision_benchmark::wrapSDF(useStr);
    }
    else
    {
      std::cerr<<"SDF is not proper so cannot load model from the string. Value: "<<checkSDF<<std::endl;
      return ret;
    }
  }

  sdf::ElementPtr sdfRoot = collision_benchmark::GetSDFElementFromString(useStr, "model", modelname);
  if (!sdfRoot)
  {
    std::cerr<< " Could not get SDF for model."<<std::endl;
    return ret;
  }

  ret = AddModelFromSDF(sdfRoot);
  return ret;

}

GazeboPhysicsWorld::ModelLoadResult GazeboPhysicsWorld::AddModelFromSDF(const sdf::ElementPtr& sdf, const std::string& modelname)
{
  gazebo::physics::ModelPtr model = collision_benchmark::LoadModelFromSDF(sdf, world, modelname);
  ModelLoadResult ret;
  if (!model)
  {
    ret.opResult=FAILED;
    return ret;
  }
  ret.opResult=SUCCESS;
  ret.modelID=model->GetName();
  // WorldState s(world); std::cout<<"World state: "<<s<<std::endl;
  return ret;
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
  std::vector<GazeboPhysicsWorld::ModelID> names;
  int numModels = world->ModelCount();
  for (int i=0; i<numModels; ++i)
  {
    gazebo::physics::ModelPtr m=world->ModelByIndex(i);
    names.push_back(m->GetName());
  }
  return names;
}

bool GazeboPhysicsWorld::RemoveModel(const ModelID& id)
{
  gazebo::physics::ModelPtr m=world->ModelByName(id);
  if (!m) return false;
  world->RemoveModel(m);
  return true;
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
  if (!world->PhysicsEnabled()) t.CheckDynamics=false;
  if (!GazeboStateCompare::Equal(_currentState, state, t))
  {
    std::cerr<<"Target state was not set as supposed to!!"<<std::endl;
  }
#endif

  return PhysicsWorldBase::SUCCESS;
}

void GazeboPhysicsWorld::Update(int steps)
{
  // std::cout<<"Running "<<steps<<" steps for world "<<world->Name()<<", physics engine: "<<world->Physics()->GetType()<<std::endl;
  // Run simulation for given number of steps.
  // This method calls world->RunBlocking();
  gazebo::runWorld(world,steps);
}


std::string GazeboPhysicsWorld::GetName() const
{
  return world->Name();
}

bool GazeboPhysicsWorld::SupportsContacts() const
{
  return true;
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
  gazebo::physics::ModelPtr m=world->ModelByName(model);
  return collision_benchmark::to_std_ptr<gazebo::physics::Model>(m);
}

GazeboPhysicsWorld::PhysicsEnginePtr GazeboPhysicsWorld::GetPhysicsEngine() const
{
  if (world) return collision_benchmark::to_std_ptr<GazeboPhysicsWorld::PhysicsEngine>(world->Physics());
  return PhysicsEnginePtr();
}
