/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#ifndef COLLISION_BENCHMARK_GAZEBOWORLDSTATE_
#define COLLISION_BENCHMARK_GAZEBOWORLDSTATE_

#include <collision_benchmark/PhysicsWorld.hh>
#include <gazebo/physics/World.hh>

namespace collision_benchmark
{

/**
 * Helper function which fixes the SDF format in the string, aimed at being part
 * of the WorldState's <insertions> or <deletions>.
 * Adds the <sdf version='1.6'>...</sdf> tags around the string. This is required
 * for compatibility with World::SetState().
 * Examle where this is required The insertions coming
 * from WorldState::operator- are in a format not compatible and need to be fixed
 * with this fucntion..
 */
void fixSDF(std::string& sdf);

/**
 * Calls fixSDF() for all the sdf's
 */
void fixSDF(std::vector<std::string>& sdf);


/**
 * Sets the \e world to the state \e targetState
 */
void SetWorldState(gazebo::physics::WorldPtr& world, const gazebo::physics::WorldState& targetState);

/**
 * Print the world state. Can be used for testing.
 */
void PrintWorldState(const gazebo::physics::WorldPtr world);

/**
 * Print the world states. Can be used for testing.
 */
void PrintWorldStates(const std::vector<gazebo::physics::WorldPtr>& worlds);

/**
 * Print the world states. Can be used for testing.
 */
void PrintWorldStates(const std::vector<PhysicsWorldBase<gazebo::physics::WorldState>::Ptr>& worlds);

}

#endif   // COLLISION_BENCHMARK_GAZEBOWORLDSTATE_
