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
#ifndef COLLISIONBENCHMARK_WORLDLOADER_
#define COLLISIONBENCHMARK_WORLDLOADER_

#include <gazebo/gazebo.hh>
#include <vector>

namespace collision_benchmark
{

/// returns the root of the world in the SDF
sdf::ElementPtr GetWorldFromSDF(const std::string& filename, const std::string& name);

/// loads a world given a SDF element
/// \param name if not empty string, then this name is used to override the name in \e worldFile
gazebo::physics::WorldPtr LoadWorldFromSDF(const sdf::ElementPtr& sdfRoot, const std::string& name="");

/// loads a world from file
/// \param name if not empty string, then this name is used to override the name in \e worldfile
/// \param overridePhysics if not NULL, this SDF is used to override the physics in the SDF \e worldfile.
gazebo::physics::WorldPtr LoadWorldFromFile(const std::string& worldfile,
                                            const std::string& name="",
                                            const sdf::ElementPtr& overridePhysics=sdf::ElementPtr());

/// Like LoadWorldFromFile(), but does additional error checking and waiting for the
/// namespace to be loaded
gazebo::physics::WorldPtr LoadWorld(const std::string& worldfile,
                                    const std::string& name="",
                                    const sdf::ElementPtr& overridePhysics=sdf::ElementPtr());

/// Gets the ``<physics>`` element from the SDF given in \e filename. Must be under the root's ``<world>`` tag.
sdf::ElementPtr GetPhysicsFromSDF(const std::string& filename);

class Worldfile
{
  public: Worldfile(const std::string& filename_, const std::string& worldname_):
            filename(filename_),
            worldname(worldname_) {}

  public: Worldfile(const Worldfile& o):
            filename(o.filename),
            worldname(o.worldname) {}

  public: friend bool operator<(const Worldfile& w1, const Worldfile& w2)
          {
            return w1.filename < w2.filename || (w1.filename == w2.filename && w1.worldname < w2.worldname);
          }

  public: std::string filename;
  public: std::string worldname;
};

/// Convenience function to load several worlds at once. Loads the worlds in the order given in \e worldfiles
///  and returns the accordingly in the same order.
///
/// \param worldNames has to be of same size as \e worldfiles and contains names
///        of the respective worlds to override the name given in the world file.
///        If a names is an empty string, it will instead keep the name in the original world
///        file or use the default name.
/// \return worlds will contain the loaded worlds
std::vector<gazebo::physics::WorldPtr> LoadWorlds(const std::vector<Worldfile>& worldfiles);

/// Waits for the namespace \e worldNamespace to appear in the Gazebo list of namespaces.
/// Repeatedly waits for new incoming namespaces (waiting *maximum* \e sleepTime seconds at each attempt)
/// and checks whether the given \e worldNamespace is in the list of namespaces.
/// \param maxWaitTime waits for this maximum time (seconds)
bool WaitForNamespace(std::string worldNamespace, float maxWaitTime = 10, float sleepTime = 1);

}  // namespace collision_benchmark

#endif  // COLLISIONBENCHMARK_WORLDLOADER_
