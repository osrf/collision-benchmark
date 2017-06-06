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
void ClearModels(gazebo::physics::WorldPtr &world);

/**
 * Returns all supported physics engines
 */
std::set<std::string> GetSupportedPhysicsEngines();

/**
 * Returns a table with the filenames to use for
 * each of the supported physics engines.
 * Only supported engines are returned. Key of the returned map is the engine
 * name as given in \e engines, value is the path to the SDF file with the
 * physics settings.
 *
 * Requirement is that the ``physics_settings`` directory is in the
 * GAZEBO_RESOURCE_PATH
 * \param engines can contain "ode", "bullet", "dart", "simbody"
 */
std::map<std::string,std::string>
getPhysicsSettingsSdfFor(const std::vector<std::string>& engines);

/**
 * Returns the filenames to use for the physics engine \e engine.
 * Requirement is that the ``physics_settings`` directory is in the
 * GAZEBO_RESOURCE_PATH.
 *
 * \param engines can contain "ode", "bullet", "dart", "simbody"
 * \return the path to the SDF file with the physics settings,
 *         or empty string if engine not supported.
 */
std::string getPhysicsSettingsSdfFor(const std::string &engine);

/**
 * Calls getPhysicsSettingsSdfFor() with all supported
 * engines as returned from GetSupportedPhysicsEngines().
 */
std::map<std::string,std::string> getPhysicsSettingsSdfForAllEngines();


/**
 * Checks whether the SDF format in the file is proper, which means an outer
 * ``<sdf>`` tag exists and a version is given in an attribute. The version is
 * returned in \e version (if it is not NULL).
 * \retval 0 SDF is proper
 * \retval -1 no version in ``<sdf>`` tag
 * \retval -2 no outer ``<sdf>`` tag
 * \retval -3 file could not be read
 */
int isProperSDFFile(const std::string &filename, std::string* version = NULL);

/**
 * Checks whether the SDF format in the string is proper, which means an outer
 * ``<sdf>`` tag exists and a version is given in an attribute. The version is
 * returned in \e version (if it is not NULL).
 * \retval 0 SDF is proper
 * \retval -1 no version in ``<sdf>`` tag
 * \retval -2 no outer ``<sdf>`` tag
 */
int isProperSDFString(const std::string &str, std::string* version = NULL);

/**
 * Helper function which fixes the SDF format in the string, aimed at being part
 * of the WorldState's <insertions> or <deletions>.
 * Adds the <sdf version='1.6'>...</sdf> tags around the string. This is
 * required for compatibility with World::SetState().
 * Examle where this is required The insertions coming
 * from WorldState::operator- are in a format not compatible and need to be
 * fixed with this function..
 */
void wrapSDF(std::string &sdf);

/**
 * Calls fixSDF() for all the sdf's
 */
void wrapSDF(std::vector<std::string>& sdf);



}  // namespace

#endif   // COLLISION_BENCHMARK_GAZEBOHELPERS_
