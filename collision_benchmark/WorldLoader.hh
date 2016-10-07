#ifndef COLLISION_BENCHMARK_WORLDLOADER
#define COLLISION_BENCHMARK_WORLDLOADER

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


#include <gazebo/gazebo.hh>

namespace collision_benchmark
{

/// returns the root of the world in the SDF
extern sdf::ElementPtr GetWorldFromSDF(const std::string& filename, const std::string& name);

/// loads a world given a SDF element
extern gazebo::physics::WorldPtr LoadWorldFromSDF(const sdf::ElementPtr& sdfRoot, const std::string& name);

/// loads a world from file
extern gazebo::physics::WorldPtr LoadWorldFromFile(const std::string &_worldFile, const std::string& name);

/// Like LoadWorldFromFile(), but does additional error checking and waiting for the
/// namespace to be loaded
extern gazebo::physics::WorldPtr LoadWorld(const std::string& worldfile, const std::string& name);

/// Convenience function to load several worlds at once
/// \param worldNames has to be of same size as \e worldfiles and contains names
///        of the respective worlds to override the name given in the world file.
///        If a names is an empty string, it will instead keep the name in the original world
///        file or use the default name.
/// \param worlds will contain the loaded worlds
extern bool LoadWorlds(const std::vector<std::string>& worldfiles,
                       const std::vector<std::string>& worldNames,
                       std::vector<gazebo::physics::WorldPtr>& worlds);

}  // namespace collision_benchmark

#endif  // COLLISION_BENCHMARK_WORLDLOADER
