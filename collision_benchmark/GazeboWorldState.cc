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
/* Desc: Helper functions for Gazebo world states
 * Author: Jennifer Buehler
 * Date: October 2016
 */

#include <collision_benchmark/GazeboWorldState.hh>
#include <collision_benchmark/GazeboHelpers.hh>

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>


/**
 * Returns new entities which were added in \e state2 when compared to _state1
 * \param models new models added in \e state2
 * \param models new lights added in \e state2
 */
void GetNewEntities(const gazebo::physics::WorldState& _state1,
          const gazebo::physics::WorldState& _state2,
          std::vector<gazebo::physics::ModelState>& models,
          std::vector<gazebo::physics::LightState>& lights)
{
  const gazebo::physics::ModelState_M& _modelStates1 = _state1.GetModelStates();
  for (gazebo::physics::ModelState_M::const_iterator iter =
        _modelStates1.begin(); iter != _modelStates1.end(); ++iter)
  {
    if (!_state2.HasModelState(iter->second.GetName()))
    {
      models.push_back(iter->second);
    }
  }

  const gazebo::physics::LightState_M& _lightStates1 = _state1.LightStates();
  for (const auto & light : _lightStates1)
  {
    if (!_state2.HasLightState(light.second.GetName()))
    {
      lights.push_back(light.second);
    }
  }
}


// XXX TODO REMOVE: Flags for testing
#define FORCE_TARGET_TIME_VALUES
// #define DEBUGWORLDSTATE
void collision_benchmark::SetWorldState(gazebo::physics::WorldPtr& world, const gazebo::physics::WorldState& targetState)
{
  bool pauseState = world->IsPaused();
  world->SetPaused(true);
  gazebo::physics::WorldState currentState(world);

#ifdef DEBUGWORLDSTATE
  std::cout << "Setting world state. " << std::endl;
  std::cout << "Target state: " << std::endl << targetState << std::endl;
  std::cout << "Current state: " << std::endl << currentState << std::endl;
#endif

  //re-set state times
  // XXX TODO CHECK: should reset all times of the state as well,
  // because the *difference* is going to be added to it
  // Not sure yet how to best handle the times.
  /*  currentState.SetWallTime(common::Time(0));
    currentState.SetRealTime(common::Time(0));
    currentState.SetIterations(0);*/

  // Handle all insertions/deletions of models in a
  // differential state. The result will have the name of
  // origState, adding all the models/lights/etc from origState which are
  // not in currentState
  gazebo::physics::WorldState diffState = targetState - currentState;
#ifdef DEBUGWORLDSTATE
  std::cout << "Diff state: " << std::endl << diffState << std::endl;
#endif

  // diffState now has the list of insertions and deletions required for using
  // in gazebo::World::SetWorldState. However, diffState cannot be used
  // to determine rotations of models, because they are not commutative:
  //   currentState + diffState = target
  // is NOT equivalent to equation used above, if non-commutative!
  // For this reason, do the update in two steps, instead of simply
  // finding the actual new state via newState = currentState + diffState
  // (which is not legal for roataions).
  // Step 1: Handle all insertions and deletions, because they only
  //         can be extracted from the diffState. Apply to current state.
  // Step 2: Now current state should have same models as target state. Can
  //         simply set the target state.

  ///// Step 1: Handle insertions (requires fixing of SDF)
  gazebo::physics::WorldState modelChangeState = currentState;
  std::vector<std::string> insertions = diffState.Insertions();
  wrapSDF(insertions);
  modelChangeState.SetInsertions(insertions);
  std::vector<std::string> deletions = diffState.Deletions();
  modelChangeState.SetDeletions(deletions);
  wrapSDF(deletions);

  // apply the state of Step 1 to the world
  world->SetState(modelChangeState);

  // the insertions still have one drawback: They don't contain
  // the current model pose. So subsequently, the current pose
  // is only published as message in the *next* world update *only if*
  // the pose has changed (from within Model::OnPoseChange by call of World::SetWorldPose).
  // But if the pose within the target state world has not changed,
  // no message is published. So we need to force publishing the poses
  // of the newly inserted models.
  std::vector<gazebo::physics::ModelState> models;
  std::vector<gazebo::physics::LightState> lights;
  GetNewEntities(targetState, currentState, models, lights);

  // now, update the poses of the new models and lights
  for (const auto & model : models)
  {
#ifdef DEBUGWORLDSTATE
    std::cout<<"New model: "<<model.GetName()<<std::endl;
#endif
    gazebo::physics::ModelPtr m = world->ModelByName(model.GetName());
    if (!m)
    {
      throw new gazebo::common::Exception(__FILE__, __LINE__,
                        "Model not found though it should have been inserted.");
    }
    m->SetState(model);
  }

  for (const auto & light : lights)
  {
#ifdef DEBUGWORLDSTATE
    std::cout<<"New light: "<<light.GetName()<<std::endl;
#endif
    gazebo::physics::LightPtr l = world->LightByName(light.GetName());
    if (!l)
    {
      throw new gazebo::common::Exception(__FILE__, __LINE__,
                        "Light not found though it should have been inserted.");
    }
    l->SetState(light);
  }


  currentState = gazebo::physics::WorldState(world);

#ifdef DEBUGWORLDSTATE
  std::cout << "New state after insertions " << std::endl << currentState << std::endl;
#endif

  //// Step 2: Set the target state

  gazebo::physics::WorldState newState = targetState;

  // Force the new state to use certain iteration/time values
  // in order to maintain consistency within the world

  // XXX DEBUG THIS: Causes world to be not properly updated
/*#ifdef FORCE_TARGET_TIME_VALUES
  newState.SetIterations(targetState.GetIterations());
  newState.SetWallTime(targetState.GetWallTime());
  newState.SetRealTime(targetState.GetRealTime());
  newState.SetSimTime(targetState.GetSimTime());
#else
  newState.SetIterations(currentState.GetIterations());
  newState.SetWallTime(currentState.GetWallTime());
  newState.SetRealTime(currentState.GetRealTime());
  newState.SetSimTime(currentState.GetSimTime());
#endif*/

  // apply the state of Step 1 to the world
  world->SetState(newState);

#ifdef DEBUGWORLDSTATE
  std::cout << "State set to:" << std::endl;
  gazebo::physics::WorldState _currentState(world);
  std::cout << _currentState << std::endl;
#endif

  world->SetPaused(pauseState);
}


void collision_benchmark::PrintWorldState(const gazebo::physics::WorldPtr world)
{
  std::cout << "## State of world " << world->Name() << std::endl;
  gazebo::physics::WorldState _state(world);
  std::cout << _state << std::endl;
}

void collision_benchmark::PrintWorldStates(const std::vector<gazebo::physics::WorldPtr>& worlds)
{
  std::cout << "## World states ###" << std::endl;
  for (std::vector<gazebo::physics::WorldPtr>::const_iterator w = worlds.begin();
      w != worlds.end(); ++w)
  {
    PrintWorldState(*w);
  }
  std::cout << "#####" << std::endl;
}

void collision_benchmark::PrintWorldStates(const std::vector<PhysicsWorldBase<gazebo::physics::WorldState>::Ptr>& worlds)
{
  std::cout << "## World states ###" << std::endl;
  for (std::vector<PhysicsWorldBase<gazebo::physics::WorldState>::Ptr>::const_iterator w = worlds.begin();
      w != worlds.end(); ++w)
  {
    gazebo::physics::WorldState s=(*w)->GetWorldState();
    std::cout << s << std::endl;
  }
  std::cout << "#####" << std::endl;
}



