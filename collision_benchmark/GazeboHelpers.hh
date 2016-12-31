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
#ifndef COLLISION_BENCHMARK_GAZEBOHELPERS_
#define COLLISION_BENCHMARK_GAZEBOHELPERS_

#include <gazebo/physics/World.hh>
#include <set>
#include <map>
#include <vector>

namespace collision_benchmark
{


/**
 * Clears all models, and all contacts, from the world.
 */
void ClearModels(gazebo::physics::WorldPtr& world);

/**
 * Returns all supported physics engines
 */
std::set<std::string> GetSupportedPhysicsEngines();

/**
 * Returns a table with the filenames to use for each of the supported physcis engines.
 * Only supported engines are returned. Key of the returned map is the engine name as
 * given in \e engines, value is the path to the SDF file with the physics settings.
 *
 * Requirement is that the physics_settings directory is in the GAZEBO_RESOURCE_PATH
 * \param engines can contain "ode", "bullet", "dart", "simbody"
 */
std::map<std::string,std::string> getPhysicsSettingsSdfFor(const std::vector<std::string>& engines);

/**
 * Calls getPhysicsSettingsSdfFor() with all supported engines as returned from GetSupportedPhysicsEngines().
 */
std::map<std::string,std::string> getPhysicsSettingsSdfForAllEngines();

}  // namespace

#endif   // COLLISION_BENCHMARK_GAZEBOHELPERS_
