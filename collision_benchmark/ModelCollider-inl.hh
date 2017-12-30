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
  :collisionAxis(0, 1, 0)
{
}

/////////////////////////////////////////////////////////////////////////////
template<class WM>
bool ModelCollider<WM>::Init(const WorldManagerPtr &wManager,
                         const ignition::math::Vector3d &collAxis,
                         const std::string &modelName1,
                         const std::string &modelName2)
{
  if (!wManager)
  {
    std::cerr << "Need to set world manager" << std::endl;
    return false;
  }

  if (!wManager->ModelInAllWorlds(modelName1) ||
      !wManager->ModelInAllWorlds(modelName2))
  {
    std::cerr << "ModelCollider only works if models are loaded in all worlds."
              << std::endl;
    return false;
  }

  if (!SetCollisionAxis(collAxis))
  {
    std::cerr << "Could not set collision axis" << std::endl;
    return false;
  }

  this->modelNames[0] = modelName1;
  this->modelNames[1] = modelName2;
  this->worldManager = wManager;
  return true;
}

/////////////////////////////////////////////////////////////////////////////
template<class WM>
bool
ModelCollider<WM>::SetCollisionAxis(const ignition::math::Vector3d &collAxis)
{
  if (collAxis.Length() < 1e-06)
  {
    std::cerr << "Cannot set zero lenght collision axis" << std::endl;
    return false;
  }
  this->collisionAxis = collAxis;
  this->collisionAxis.Normalize();
  return true;
}

/////////////////////////////////////////////////////////////////////////////
template<class WM>
bool ModelCollider<WM>::PlaceModels(const float modelsGap,
                         const bool modelsGapIsFactor,
                         BasicState &modelState1,
                         BasicState &modelState2)
{
  assert(this->worldManager);

  // make sure the models are at the origin first (needed to
  // ensure the local coodrdinate system equals the global, to get the
  // AABBs in global reference frame later on).
  modelState1.SetPosition(0, 0, 0);
  modelState1.SetRotation(0, 0, 0, 1);
  modelState2 = BasicState(modelState1);
  if ((this->worldManager->SetBasicModelState(modelNames[0], modelState1)
       != this->worldManager->GetNumWorlds()) ||
      (this->worldManager->SetBasicModelState(modelNames[1], modelState2)
       != this->worldManager->GetNumWorlds()))
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
  if ((GetAABB(modelNames[0], this->worldManager, min1, max1, local1) != 0) ||
      (GetAABB(modelNames[1], this->worldManager, min2, max2, local2) != 0))
  {
    std::cerr << "Could not get AABBs of models" << std::endl;
    return false;
  }

  /*
  // NOTE
  // In case models at some point are *not* first placed at the origin with
  // identity orientation (as done now), so when global frame != local frame:
  // If the AABBs are not given in global coordinate frame, we need to transform
  // the AABB coordinates first!
  // Example for first AABB:
  if (local1 && !GetAABBInFrame(ignition::math::Quaterniond::Identity,
                                modelNames[0], min1, max1, local1, min1, max1))
  {
    std::cerr << "Error computing AABB in global frame" << std::endl;
  }
  */

  // Re-project min and max points of aabb on collision axis
  double projMin, projMax;
  ignition::math::Vector3d ignMin(collision_benchmark::ConvIgn<double>(min1));
  ignition::math::Vector3d ignMax(collision_benchmark::ConvIgn<double>(max1));
  collision_benchmark::ProjectAABBOnAxis(ignMin, ignMax, this->collisionAxis,
                                         projMin, projMax);
  const float aabb1LenOnAxis = fabs(projMax - projMin);
  min1 = this->collisionAxis * projMin;
  max1 = this->collisionAxis * projMax;

  ignMin = collision_benchmark::ConvIgn<double>(min2);
  ignMax = collision_benchmark::ConvIgn<double>(max2);
  collision_benchmark::ProjectAABBOnAxis(ignMin, ignMax, this->collisionAxis,
                                         projMin, projMax);
  const float aabb2LenOnAxis = fabs(projMax - projMin);
  min2 = this->collisionAxis * projMin;
  max2 = this->collisionAxis * projMax;

  // Leave model 1 where it is and move model 2 away from it.
  //////////////////

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
  ignition::math::Vector3d moveM2AlongAxis
    = this->collisionAxis * moveM2Distance;
  collision_benchmark::Vector3 newModelPos2 = modelState2.position;
  newModelPos2.x += moveM2AlongAxis.X();
  newModelPos2.y += moveM2AlongAxis.Y();
  newModelPos2.z += moveM2AlongAxis.Z();
  modelState2.SetPosition(newModelPos2);
  // move model 2
  this->worldManager->SetBasicModelState(modelNames[1], modelState2);
  return true;
}

//////////////////////////////////////////////////////////////////////////////
template<class WM>
bool ModelCollider<WM>::CollisionExcluded() const
{
  // First, get the AABB's of the two models and ensure they are in global
  // frame
  Vector3 min1, min2, max1, max2;
  bool local1, local2;
  if ((GetAABB(modelNames[0], this->worldManager, min1, max1, local1) != 0) ||
      (GetAABB(modelNames[1], this->worldManager, min2, max2, local2) != 0))
  {
    std::cerr << "Could not get AABBs of models" << std::endl;
    return false;
  }
  assert(local1 == local2);

  // get the AABBS in  a frame such that Z axis is aligned with the collision
  // axis. The models collide along this axis, so we can use X and Y axes
  // as separating axes, because the AABBs in this coordinate frame will
  // either overlap in X or Y when they can collide along the Z axis.
  ignition::math::Vector3d projAxis = ignition::math::Vector3d::UnitZ;
  ignition::math::Quaterniond q;
  q.From2Axes(this->collisionAxis, projAxis);
  if (!GetAABBInFrame(q, modelNames[0], min1, max1, local1, min1, max1) ||
      !GetAABBInFrame(q, modelNames[1], min2, max2, local1, min2, max2))
  {
    std::cerr << "Error computing AABBs in collision axis frame" << std::endl;
    return false;
  }

  double overlapX = -1;
  double overlapY = -1;
  if (!collision_benchmark::SegmentsOverlap(min1.X(), max1.X(),
                                           min2.X(), max2.X(), &overlapX) ||
      !collision_benchmark::SegmentsOverlap(min1.Y(), max1.Y(),
                                           min2.Y(), max2.Y(), &overlapY))
  {
    /*std::cout << "AABB1: " << min1 << ", " << max1 << std::endl;
    std::cout << "AABB2: " << min2 << ", " << max2 << std::endl;
    std::cout << "Segments do not overlap: "
              << overlapX << ", " << overlapY << std::endl;*/
    return true;
  }
  return false;
}


//////////////////////////////////////////////////////////////////////////////
template<class WM>
bool ModelCollider<WM>::ModelsCollide(bool allWorlds) const
{
  assert(this->worldManager);

  typedef typename WorldManagerT::PhysicsWorldContactInterfacePtr
          PhysicsWorldContactInterfacePtr;
  std::vector<PhysicsWorldContactInterfacePtr>
    contactWorlds = this->worldManager->GetContactPhysicsWorlds();

  assert(contactWorlds.size() == this->worldManager->GetNumWorlds());

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
      if (!allWorlds) break;
    }
  }

  // while collision is not found, move models towards each other
  if (allWorlds)
    // all worlds have to collide
    return modelsColliding == contactWorlds.size();

  // only one world has to collide
  return modelsColliding > 0;
}


namespace collision_benchmark
{
/**
 * Simple implementation of a contacts cluster based on the average
 * vector of all the contacts.
 */
template<class Contact>
class ContactsCluster
{
  typedef typename Contact::Vector3 ContVec3;
  typedef ignition::math::Vector3d IgnVec;
  typedef std::pair<Contact, IgnVec> Pair;
  // constructs a cluster with only one point
  public: ContactsCluster(const Contact& c)
          {
            add(c);
          }
  public: ContactsCluster(const ContactsCluster& o):
          contacts(o.contacts),
          avg(o.avg) {}

  // cluster size (radius of the cluster)
  public: static double ToleranceRadius;

  // checks whether the contact is less than the cluster radius
  // tolerance away from the current center point. If the cluster is empty,
  // this always returns true.
  public: bool inside(const Contact& c) const
  {
    if (contacts.empty()) return true;
    IgnVec v = collision_benchmark::ConvIgn<double>(c.position);
    return inside(v);
  }
  // Helper, checks whether \e v is less than ToleranceRadius from avg
  private: bool inside(const IgnVec& v) const
  {
    if (contacts.empty()) return true;
    IgnVec diff = avg - v;
    std::cout << "Dist: " << diff.Length() << std::endl;
    return diff.Length() <= ToleranceRadius;
  }

  public: void add(const Contact& c)
  {
    IgnVec v = collision_benchmark::ConvIgn<double>(c.position);
    // add point and recompute average
    contacts.push_back(std::make_pair(c, v));
    IgnVec accum(0,0,0);
    for (typename std::vector<Pair>::const_iterator it = contacts.begin();
         it != contacts.end(); ++it)
    {
      const Pair cPair = *it;
      accum += cPair.second;
    }
    avg = accum / contacts.size();
  }
/*  public: void merge(const ContactsCluster& c)
  {
    for (typename std::vector<Pair>::const_iterator it = c.contacts.begin();
         it != c.contacts.end(); ++it)
    {
      const Pair cPair = *it;
      // not the most efficient because the average is being re-computed
      // all the time, but for now we don't emphasise efficiency
      add(cPair->first);
    }
  }*/
  // operator returns < if \e c may be merged with this cluster
  // without exceeding ToleranceRadius. The center point may change however.
  public: bool operator<(const ContactsCluster& c) const
  {
    std::cout << "YEEEAH" << std::endl;
    for (typename std::vector<Pair>::const_iterator it = c.contacts.begin();
         it != c.contacts.end(); ++it)
    {
      const Pair cPair = *it;
      if (!inside(cPair.second)) return false;
    }
    std::cout << "Can merge clusters!!" << std::endl;
    return true;
  }
  private: std::vector<Pair> contacts;
  public: IgnVec avg;

}; // end class ContactCluster


template<class Contact>
double ContactsCluster<Contact>::ToleranceRadius = 0.0;

} // end namespace

//////////////////////////////////////////////////////////////////////////////
template<class WM>
std::vector<ignition::math::Vector3d>
ModelCollider<WM>::GetClusteredContacts(const size_t worldIdx,
                                        double clusterSize) const
{
  assert(this->worldManager);
  if (worldIdx >= this->worldManager->GetNumWorlds())
    throw new std::runtime_error("World index out of bounds");

  typedef typename WorldManagerT::PhysicsWorldContactInterfacePtr
          PhysicsWorldContactInterfacePtr;
  typedef typename WorldManagerT::PhysicsWorldContactInterfaceT::ContactInfoPtr
          ContactInfoPtr;
  typedef typename WorldManagerT::PhysicsWorldContactInterfaceT::Contact
          Contact;
  typedef typename Contact::Ptr ContactPtr;

  std::vector<PhysicsWorldContactInterfacePtr>
    contactWorlds = this->worldManager->GetContactPhysicsWorlds();

  assert(contactWorlds.size() == this->worldManager->GetNumWorlds());

  PhysicsWorldContactInterfacePtr world = contactWorlds[worldIdx];
  std::vector<ContactInfoPtr> contactInfo = world->GetContactInfo();
  if (contactInfo.empty())
  {
    // models don't collide
    return std::vector<ignition::math::Vector3d>();
  }

  // temporary for testing, make assert of this soon!
  if (contactInfo.size() != 1)
    throw std::runtime_error("Consistency: All contacts between models should be in one ContactInfo");

  std::vector<Contact> contacts = contactInfo.front()->contacts;

  typedef ContactsCluster<Contact> ContactsClusterT;
  ContactsClusterT::ToleranceRadius = clusterSize;
  std::vector<ContactsClusterT> clusteredContacts;
  for (typename std::vector<Contact>::const_iterator it = contacts.begin();
       it != contacts.end(); ++it)
  {
    // add contact to the first cluster in which it fits (this can be
    // optimized later!). Create a new cluster if it doesn't fit in any.
    const Contact &contact = *it;
    bool added = false;
    for (typename std::vector<ContactsClusterT>::iterator
         cit = clusteredContacts.begin(); cit != clusteredContacts.end(); ++cit)
    {
      ContactsClusterT& cluster = *cit;
      if (cluster.inside(contact))
      {
        cluster.add(contact);
        added = true;
        break;
      }
    }
    if (!added)
    {
      ContactsClusterT single(contact);
      clusteredContacts.push_back(single);
    }
/*    ContactsClusterT single(contact);
    typename std::set<ContactsClusterT>::iterator cIt =
      clusteredContacts.find(single);
    if (cIt != clusteredContacts.end())
    {
      std::cout << "Found a cluster which fits" << std::endl;
      cIt->add(contact);
    }
    else
    {
      clusteredContacts.insert(single);
    }*/
  }
  std::vector<ignition::math::Vector3d> ret;
  for (typename std::vector<ContactsClusterT>::iterator
       cit = clusteredContacts.begin(); cit != clusteredContacts.end(); ++cit)
  {
    ContactsClusterT& cluster = *cit;
    ret.push_back(cluster.avg);
  }
  return ret;
}

//////////////////////////////////////////////////////////////////////////////
template<class WM>
double ModelCollider<WM>::AutoCollide(const bool allWorlds,
                                  const bool moveBoth,
                                  const double stepSize,
                                  const float maxMovePerSec,
                                  const bool stopWhenPassed,
                                  BasicState *ms1,
                                  BasicState *ms2)
{
  assert(this->worldManager);
  double moved = 0;
  gazebo::common::Timer timer;
  if (maxMovePerSec > 0) timer.Start();
  // while collision is not found, move models towards each other
  // XXX TODO: also check if models would collide at all considering
  // their current poses
  while (!ModelsCollide(allWorlds))
  {
    if (maxMovePerSec > 0)
    {
      gazebo::common::Time elapsed = timer.GetElapsed();
      // move the shapes towards each other in steps.
      // Slow down the movement if it's too fast.
      // Not the most accurate way to achieve a maximum velocity,
      // but considering this is only for animation purposes, this will do.
      if ((moved > 0) && (moved / elapsed.Double() > maxMovePerSec))
      {
        // slow down the move as we've already moved too far.
        // Sleep a tiny bit.
        gazebo::common::Time::MSleep(10);
        continue;
      }
    }
    if (MoveModelsAlongAxis(stepSize, moveBoth,
                            stopWhenPassed, true, ms1, ms2) != 0)
    {
//      std::cout << "Stopping Auto-Collide because objects weren't moved"
//                << std::endl;
      break;
    }
    moved += stepSize;
  }
  return moved;
}

//////////////////////////////////////////////////////////////////////////////
template<class WM>
int ModelCollider<WM>::MoveModelsAlongAxis(const double moveDist,
                                           const bool moveBoth,
                                           const bool stopWhenPassed,
                                           const bool worldUpdate,
                                           BasicState *ms1,
                                           BasicState *ms2)
{
  assert(this->worldManager);
  // get state of both models
  BasicState modelState1, modelState2;
  // get the states of the models as loaded in their original pose.
  // We need the state of model 1 only if moveBoth or stopWhenPassed are true.
  if ((moveBoth || stopWhenPassed) &&
      (GetBasicModelState(modelNames[0], this->worldManager, modelState1) != 0))
  {
    std::cerr << "Could not get BasicModelState." << std::endl;
    return -1;
  }
  if (GetBasicModelState(modelNames[1], this->worldManager, modelState2) != 0)
  {
    std::cerr << "Could not get BasicModelState." << std::endl;
    return -1;
  }

  if (stopWhenPassed && (moveDist > 0))
  {
    // project both poses onto collision axis and see if the models
    // center poses have passed each other along the collision axis
    ignition::math::Vector3d
      pos1(collision_benchmark::ConvIgn<double>(modelState1.position));
    ignition::math::Vector3d
      pos2(collision_benchmark::ConvIgn<double>(modelState2.position));
    double dot1 = pos1.Dot(this->collisionAxis);
    double dot2 = pos2.Dot(this->collisionAxis);
    if (dot1 > dot2)
    {
//      std::cout << "Objects centers have passed on collision axis, "
//                << "not moving further. " << std::endl;
      if (moveBoth && ms1) *ms1 = modelState1;
      if (ms2) *ms2 = modelState2;
      return 1;
    }
  }

  const ignition::math::Vector3d mv = this->collisionAxis * moveDist;

  if (moveBoth)
  {
    modelState1.SetPosition(modelState1.position.x + mv.X(),
                            modelState1.position.y + mv.Y(),
                            modelState1.position.z + mv.Z());
    if (ms1) *ms1 = modelState1;
  }

  modelState2.SetPosition(modelState2.position.x - mv.X(),
                          modelState2.position.y - mv.Y(),
                          modelState2.position.z - mv.Z());
  if (ms2) *ms2 = modelState2;

  if ((moveBoth &&
       (this->worldManager->SetBasicModelState(modelNames[0], modelState1)
        != this->worldManager->GetNumWorlds())) ||
      (this->worldManager->SetBasicModelState(modelNames[1], modelState2)
       != this->worldManager->GetNumWorlds()))
  {
    std::cerr << "Could not set all model poses" << std::endl;
    return -1;
  }
  if (worldUpdate) this->worldManager->Update(1);
  return 0;
}

//////////////////////////////////////////////////////////////////////////////
template<class Vec>
bool VectorsCollinear(const Vec &v1, const double v1Len,
                      const Vec &v2, const double tolerance)
{
  return fabs(fabs(v1.Dot(v2))-v1Len) < tolerance;
}

//////////////////////////////////////////////////////////////////////////////
template<class WM>
typename ignition::math::Vector3d
ModelCollider<WM>::GetAxisPerpendicular(const double angle) const
{
  return GetAxisPerpendicular(this->collisionAxis, angle);
}

//////////////////////////////////////////////////////////////////////////////
template<class WM>
typename ignition::math::Vector3d
ModelCollider<WM>::GetAxisPerpendicular(const ignition::math::Vector3d &axis,
                                        const double angle)
{
  static const ignition::math::Vector3d unitX(1,0,0);
  static const ignition::math::Vector3d unitY(0,1,0);
  static const ignition::math::Vector3d unitZ(0,0,1);
  static const double dotTolerance = 1e-02;
  const double axisLength = axis.Length();
  ignition::math::Vector3d ret;
  if (!VectorsCollinear(axis, axisLength, unitX, dotTolerance))
  {
    ret = axis.Cross(unitX);
  }
  else if (!VectorsCollinear(axis, axisLength, unitY, dotTolerance))
  {
    ret = axis.Cross(unitY);
  }
  else if (!VectorsCollinear(axis, axisLength, unitZ, dotTolerance))
  {
    ret = axis.Cross(unitZ);
  }
  else
  {
    // will only get here unless dotTolerance is large or the
    // axis is not long enough. Return any vector in this case.
    ret.Set(1,0,0);
    std::cout << "WARNING: no origin axis found suitable. "
              << __FILE__ << std::endl;
  }

  // need to apply a rotation to the axis
  if (fabs(angle) > std::numeric_limits<double>::epsilon())
  {
    const ignition::math::Quaterniond axisQuat(axis, angle);
    ret = axisQuat * ret;
  }
  return ret;
}


//////////////////////////////////////////////////////////////////////////////
template<class WM>
bool ModelCollider<WM>::MoveModelPerpendicular(const double distance,
                                               const double angle,
                                               const bool model1,
                                               const bool worldUpdate,
                                               const BasicState *fromState,
                                               BasicState *endState) const
{
  assert(this->worldManager);
  const std::string moveModelName =
    model1 ? this->modelNames[0] : this->modelNames[1];
  BasicState modelState;
  if (fromState)
  {
    modelState = *fromState;
  }
  else if (GetBasicModelState(moveModelName, this->worldManager,
                              modelState) != 0)
  {
    std::cerr << "Could not get BasicModelState." << std::endl;
    return false;
  }

  const ignition::math::Vector3d mvAxis =
    GetAxisPerpendicular(this->collisionAxis, angle) * distance;

  modelState.SetPosition(modelState.position.x + mvAxis.X(),
                         modelState.position.y + mvAxis.Y(),
                         modelState.position.z + mvAxis.Z());

//  std::cout << "SET MODEL STATE: " << modelState << std::endl;
//  std::cout << "Moving model state by " << distance << ":" << mvAxis.X()
//            << ", " << mvAxis.Y() << ", " << mvAxis.Z() << std::endl;

  if ((this->worldManager->SetBasicModelState(moveModelName, modelState)
       != this->worldManager->GetNumWorlds()))
  {
    std::cerr << "Could not set all model poses to origin" << std::endl;
    return false;
  }
  if (endState)
  {
    *endState = modelState;
  }
  if (worldUpdate) this->worldManager->Update(1);
  return true;
}


//////////////////////////////////////////////////////////////////////////////
template<class WM>
bool ModelCollider<WM>::RotateModelToPerpendicular(const double angle,
                                    const ignition::math::Vector3d &axisOrigin,
                                    const bool model1,
                                    const bool worldUpdate) const
{
  assert(this->worldManager);
  const std::string moveModelName =
    model1 ? this->modelNames[0] : this->modelNames[1];
  BasicState modelState;
  if (GetBasicModelState(moveModelName, this->worldManager, modelState) != 0)
  {
    std::cerr << "Could not get BasicModelState." << std::endl;
    return false;
  }

  // first, project the model origin on the axis and translate it there.
  ignition::math::Vector3d
    modelPos(collision_benchmark::ConvIgn<double>(modelState.position));
  const double modelProjLen = (modelPos - axisOrigin).Dot(this->collisionAxis);
  const ignition::math::Vector3d
    modelProj = axisOrigin + this->collisionAxis * modelProjLen;
  const ignition::math::Vector3d mvModelTowards = modelProj - modelPos;

  const ignition::math::Vector3d mvModelAway =
    GetAxisPerpendicular(this->collisionAxis, angle) * mvModelTowards.Length();

  const ignition::math::Vector3d mvModel = mvModelTowards + mvModelAway;

  modelState.SetPosition(modelState.position.x + mvModel.X(),
                         modelState.position.y + mvModel.Y(),
                         modelState.position.z + mvModel.Z());

  if ((this->worldManager->SetBasicModelState(moveModelName, modelState)
       != this->worldManager->GetNumWorlds()))
  {
    std::cerr << "Could not set all model poses to origin" << std::endl;
    return false;
  }
  if (worldUpdate) this->worldManager->Update(1);
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
bool ModelCollider<WM>::GetAABBInFrame(const ignition::math::Quaterniond& q,
                                       const std::string &modelName,
                                       const Vector3 &min, const Vector3 &max,
                                       const bool minMaxInLocal,
                                       Vector3 &newMin, Vector3 &newMax) const
{
  ignition::math::Matrix4d globTrans = ignition::math::Matrix4d::Identity;
  if (minMaxInLocal)
  {
    BasicState modelState;
    if ((GetBasicModelState(modelName, this->worldManager, modelState) != 0))
    {
      std::cerr << "Could not get model state for " << modelName << std::endl;
      return false;
    }
    globTrans = collision_benchmark::GetMatrix<double>(modelState.position,
                                                   modelState.rotation);
  }

  ignition::math::Matrix4d qTrans(q);
  ignition::math::Matrix4d trans = qTrans.Inverse() * globTrans;

  ignition::math::Vector3d _newMin, _newMax;
  ignition::math::Vector3d ignMin(collision_benchmark::ConvIgn<double>(min));
  ignition::math::Vector3d ignMax(collision_benchmark::ConvIgn<double>(max));
  collision_benchmark::UpdateAABB(ignMin, ignMax, trans, _newMin, _newMax);
  newMin = _newMin;
  newMax = _newMax;
  return true;
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
