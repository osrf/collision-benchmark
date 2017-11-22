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
#include <collision_benchmark/MathHelpers.hh>
#include <gazebo/common/Timer.hh>
#include "ModelCollider.hh"

using collision_benchmark::ModelCollider;
using collision_benchmark::BasicState;

/////////////////////////////////////////////////////////////////////////////
template<class WM>
ModelCollider<WM>::ModelCollider()
  : collisionAxis(0, 1, 0)
{
}

/////////////////////////////////////////////////////////////////////////////
template<class WM>
bool ModelCollider<WM>::Init(const WorldManagerPtr &wManager,
                         const Vector3 &collAxis,
                         const std::string &modelName1,
                         const std::string &modelName2)
{
  if (!this->worldManager->ModelInAllWorlds(modelName1) ||
      !this->worldManager->ModelInAllWorlds(modelName2))
  {
    std::cerr << "ModelCollider only works if models are loaded in all worlds."
              << std::endl;
    return false;
  }

  if (!wManager)
  {
    std::cerr << "Need to set world manager" << std::endl;
    return false;
  }

  if (!SetCollisionAxis(collAxis))
  {
    std::cerr << "Could not set collision axis" << std::endl;
    return false;
  }

  modelNames[0] = modelName1;
  modelNames[1] = modelName2;
  worldManager = wManager;
  return true;
}

/////////////////////////////////////////////////////////////////////////////
template<class WM>
bool ModelCollider<WM>::SetCollisionAxis(const Vector3 &collAxis)
{
  static const double axEp = 1e-06;
  if (!collision_benchmark::EqualVectors(collAxis,
                                         Vector3(1, 0, 0), axEp) &&
      !collision_benchmark::EqualVectors(collAxis,
                                         Vector3(0, 1, 0), axEp) &&
      !collision_benchmark::EqualVectors(collAxis,
                                         Vector3(0, 0, 1), axEp))
  {
    std::cerr << "At this point, the only collision axes supported are x, y "
              << "or z axis. Is " << collAxis << std::endl;
    return false;
  }
  collisionAxis = collAxis;
  return true;
}

/////////////////////////////////////////////////////////////////////////////
template<class WM>
bool ModelCollider<WM>::PlaceModels(const float modelsGap,
                         const bool modelsGapIsFactor,
                         BasicState &modelState1,
                         BasicState &modelState2)
{
  assert(worldManager);

  // make sure the models are at the origin first (needed to
  // ensure the local coodrdinate system equals the global, to get the
  // AABBs in global reference frame later on).
  modelState1.SetPosition(0, 0, 0);
  modelState1.SetRotation(0, 0, 0, 1);
  modelState2 = BasicState(modelState1);
  if ((worldManager->SetBasicModelState(modelNames[0], modelState1)
       != worldManager->GetNumWorlds()) ||
      (worldManager->SetBasicModelState(modelNames[1], modelState2)
       != worldManager->GetNumWorlds()))
  {
    std::cerr << "Could not set all model poses to origin" << std::endl;
    return false;
  }

  if (!modelState1.PosEnabled() ||
      !modelState2.PosEnabled())
  {
    std::cerr << "Models are expected to have a position" << std::endl;
    return false;
  }

  // Position models such that they
  // are separated by the desired distance
  // between their AABBs.
  ///////////////////////////////

  // First, get the AABB's of the two models.
  Vector3 min1, min2, max1, max2;
  bool local1, local2;
  if ((GetAABB(modelNames[0], worldManager, min1, max1, local1) != 0) ||
      (GetAABB(modelNames[1], worldManager, min2, max2, local2) != 0))
  {
    std::cerr << "Could not get AABBs of models" << std::endl;
    return false;
  }

  /*
  // Note: In case models at some point are *not* first placed at the origin
  // (so when global frame != local frame):
  // If the AABBs are not given in global coordinate frame, we need to transform
  // the AABBs first!
  // Example for first AABB:
  if (local1)
  {
    ignition::math::Matrix4d trans =
      collision_benchmark::GetMatrix<double>(modelState1.position,
                                             modelState1.rotation);
    ignition::math::Vector3d newMin, newMax;
    ignition::math::Vector3d ignMin(collision_benchmark::ConvIgn<double>(min1));
    ignition::math::Vector3d ignMax(collision_benchmark::ConvIgn<double>(max1));
    collision_benchmark::UpdateAABB(ignMin, ignMax, trans, newMin, newMax);
    min1 = newMin;
    max1 = newMax;
  }
  */

  // std::cout << "AABB 1: " << min1 << " -- " << max1 << std::endl;
  // std::cout << "AABB 2: " << min2 << " -- " << max2 << std::endl;

  // Leave model 1 where it is and move model 2 away from it.
  const float aabb1LenOnAxis = (max1-min1).Dot(collisionAxis);
  const float aabb2LenOnAxis = (max2-min2).Dot(collisionAxis);


  // determine the desired distance / gap between the models AABBs
  double desiredDistance = 0;
  if (modelsGap < 0)
  {
    // use default:
    // desired distance between models is \e distFact of the
    // larger AABBs on collisionAxis
    double distFact = 0.5;
    desiredDistance = std::max(aabb1LenOnAxis*distFact,
                               aabb2LenOnAxis*distFact);
  }
  else
  {
    if (modelsGapIsFactor)
      desiredDistance = std::max(aabb1LenOnAxis*modelsGap,
                                 aabb2LenOnAxis*modelsGap);
    else
      desiredDistance = modelsGap;
  }

  // we will move model 2 away from model 1 such that the desired distance
  // is achieved between the AABBs.

  // distance between both AABBs when both models are at the origin
  double aabbDist = min2.Dot(collisionAxis) - max1.Dot(collisionAxis);
  // distance to move model 2 by
  double moveM2Distance = desiredDistance - aabbDist;
  // std::cout << "Move model 2 along axis "
  //         << collisionAxis*moveM2Distance << std::endl;
  Vector3 moveM2AlongAxis = collisionAxis * moveM2Distance;
  // std::cout << "State of model 2: " << modelState2 << std::endl;
  collision_benchmark::Vector3 newModelPos2 = modelState2.position;
  newModelPos2.x += moveM2AlongAxis.X();
  newModelPos2.y += moveM2AlongAxis.Y();
  newModelPos2.z += moveM2AlongAxis.Z();
  modelState2.SetPosition(newModelPos2);
  // move model 2
  worldManager->SetBasicModelState(modelNames[1], modelState2);
  return true;
}

//////////////////////////////////////////////////////////////////////////////
template<class WM>
bool ModelCollider<WM>::ModelsCollide(bool allWorlds) const
{
  assert(worldManager);

  typedef typename WorldManagerT::PhysicsWorldContactInterfacePtr
          PhysicsWorldContactInterfacePtr;
  std::vector<PhysicsWorldContactInterfacePtr>
    contactWorlds = worldManager->GetContactPhysicsWorlds();

  assert(contactWorlds.size() == worldManager->GetNumWorlds());

  int modelsColliding = 0;
  for (typename std::vector<PhysicsWorldContactInterfacePtr>::iterator
       it = contactWorlds.begin(); it != contactWorlds.end(); ++it)
  {
    PhysicsWorldContactInterfacePtr world = *it;
    std::vector<typename WorldManagerT::PhysicsWorldContactInterfaceT::ContactInfoPtr>
      contacts = world->GetContactInfo();
    if (!contacts.empty())
    {
      ++modelsColliding;
    }
  }

  // while collision is not found, move models towards each other
  if (allWorlds)
    // all worlds have to collide
    return modelsColliding == contactWorlds.size();

  // only one world has to collide
  return modelsColliding > 0;
}

//////////////////////////////////////////////////////////////////////////////
template<class WM>
double ModelCollider<WM>::AutoCollide(const bool allWorlds,
                                  const bool moveBoth,
                                  const double stepSize,
                                  const float maxMovePerSec)
{
  assert(worldManager);
  double moved = 0;
  gazebo::common::Timer timer;
  timer.Start();
  // while collision is not found, move models towards each other
  while (!ModelsCollide(allWorlds))
  {
    gazebo::common::Time elapsed = timer.GetElapsed();
    // move the shapes towards each other in steps
    if (moved > 0)
    {
      // slow down the movement if it's too fast.
      // Not the most accurate way to achieve a maximum velocity,
      // but considering this is only for animation purposes, this will do.
      if (moved / elapsed.Double() > maxMovePerSec)
      {
        // slow down the move as we've already moved too far.
        // Sleep a tiny bit.
        gazebo::common::Time::MSleep(10);
        continue;
      }
    }
    MoveModelsAlongAxis(stepSize, moveBoth);
    moved += stepSize;
  }
  return moved;
}

//////////////////////////////////////////////////////////////////////////////
template<class WM>
bool ModelCollider<WM>::MoveModelsAlongAxis(const float moveDist,
                                        const bool moveBoth)
{
  assert(worldManager);
  // get state of both models
  BasicState modelState1, modelState2;
  // get the states of the models as loaded in their original pose
  if (moveBoth &&
      (GetBasicModelState(modelNames[0], worldManager, modelState1) != 0))
  {
    std::cerr << "Could not get BasicModelState." << std::endl;
    return false;
  }
  if (GetBasicModelState(modelNames[1], worldManager, modelState2) != 0)
  {
    std::cerr << "Could not get BasicModelState." << std::endl;
    return false;
  }
  const ignition::math::Vector3d mv = collisionAxis * moveDist;

  if (moveBoth)
  {
    modelState1.SetPosition(modelState1.position.x + mv.X(),
                            modelState1.position.y + mv.Y(),
                            modelState1.position.z + mv.Z());
  }

  modelState2.SetPosition(modelState2.position.x - mv.X(),
                          modelState2.position.y - mv.Y(),
                          modelState2.position.z - mv.Z());

  if ((moveBoth &&
       (worldManager->SetBasicModelState(modelNames[0], modelState1)
        != worldManager->GetNumWorlds())) ||
      (worldManager->SetBasicModelState(modelNames[1], modelState2)
       != worldManager->GetNumWorlds()))
  {
    std::cerr << "Could not set all model poses to origin" << std::endl;
    return false;
  }
  worldManager->Update(1);
  return true;
}

//////////////////////////////////////////////////////////////////////////////
template<class WM>
int ModelCollider<WM>::GetAABB(const std::string &modelName,
            const typename WM::Ptr &worldManager,
            Vector3 &min, Vector3 &max, bool &inLocalFrame)
{
  return GetAABB(modelName, 0, worldManager, min, max, inLocalFrame);
}

//////////////////////////////////////////////////////////////////////////////
template<class WM>
int ModelCollider<WM>::GetAABB(const std::string &modelName,
            const unsigned int idxWorld,
            const typename WM::Ptr &worldManager,
            Vector3 &min, Vector3 &max, bool &inLocalFrame)
{
  std::vector<typename WM::PhysicsWorldModelInterfacePtr >
    worlds = worldManager->GetModelPhysicsWorlds();

  if (worlds.empty() || (worlds.size() <= idxWorld)) return -1;

  typename WM::PhysicsWorldModelInterfacePtr w = worlds[idxWorld];
  if (!w->GetAABB(modelName, min, max, inLocalFrame))
  {
      std::cerr << "Model " << modelName << ": AABB could not be retrieved"
                << std::endl;
      return -2;
  }
  return 0;
}

//////////////////////////////////////////////////////////////////////////////
template<class WM>
int ModelCollider<WM>::GetBasicModelState
    (const std::string &modelName,
     const typename WM::Ptr &worldManager,
     BasicState &state)
{
  return GetBasicModelState(modelName, 0, worldManager, state);
}

//////////////////////////////////////////////////////////////////////////////
template<class WM>
int ModelCollider<WM>::GetBasicModelState
    (const std::string &modelName,
     const unsigned int idxWorld,
     const typename WM::Ptr &worldManager,
     BasicState &state)
{
  std::vector<typename WM::PhysicsWorldModelInterfacePtr >
    worlds = worldManager->GetModelPhysicsWorlds();

  if (worlds.empty() || (worlds.size() <= idxWorld)) return -2;

  typename WM::PhysicsWorldModelInterfacePtr w = worlds[idxWorld];

  if (!w->GetBasicModelState(modelName, state))
  {
      std::cerr << "Model " << modelName << ": state could not be retrieved"
                << std::endl;
      return -1;
  }
  return 0;
}
