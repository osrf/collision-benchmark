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
#include <collision_benchmark/GazeboMultipleWorldsServer.hh>
#include <collision_benchmark/GazeboWorldLoader.hh>
#include <collision_benchmark/GazeboTopicForwardingMirror.hh>
#include <collision_benchmark/GazeboControlServer.hh>

#include <gazebo/physics/WorldState.hh>
#include <gazebo/gazebo.hh>

using collision_benchmark::GazeboMultipleWorldsServer;

/////////////////////////////////////////////////////////////////////
bool GazeboMultipleWorldsServer::Start(int argc, const char** argv)
{
  if ((argc == 0) || !argv)
  {
    // can suppress this by creating fake empty comand line parameters
    // which live throughout the lifetime of this instance.
    THROW_EXCEPTION("At this point, for gazebo you need to pass "
                    <<" valid command line parameters");
  }
  gazebo::common::Console::SetQuiet(false);

  // Initialize gazebo.
  try
  {
    // unfortunately still needs non-const char
    // parameter, though it should not be manipulating it
    // so const casting shoudld be safe.
    gazebo::setupServer(argc, (char**)argv);
  }
  catch (...)
  {
    std::cerr<<"Could not setup server"<<std::endl;
    return false;
  }
  return true;
}

/////////////////////////////////////////////////////////////////////
void GazeboMultipleWorldsServer::Stop()
{
  std::cout << "Shutting down..." <<std::endl;
  gazebo::shutdown();
  std::cout << "Multi-world server ended." << std::endl;
}

/////////////////////////////////////////////////////////////////////
GazeboMultipleWorldsServer::WorldManagerPtr
GazeboMultipleWorldsServer::createWorldManager
    (const std::string& mirror_name,
     const bool allowMirrorControl)
{
  MirrorWorld::Ptr mirror;
  ControlServer<Super::ModelID>::Ptr controlServer;
  if (!mirror_name.empty())
  {
    // Before loading all other worlds, the mirror world must be loaded
    // (it has to be loaded first for gzclient to connect to this one
    // instead of the others).
    // To see why, refer to transport::Node::Init(), called
    // from gui::MainWindow constructor with the empty string)
    // To check for consistent use, check first if any other worlds
    // are on the namespace already, and throw an exception if this is
    // the case.
    std::string firstWorld = collision_benchmark::GetFirstNamespace();
    if (!firstWorld.empty())
    {
      THROW_EXCEPTION("A world has already been loaded, but the mirror "
                      << "world needs to be constructed first in order to "
                      << "allow gzclient to connect to it. Create the "
                      << "WorldManager before you load any other worlds!");
    }

    mirror.reset(new GazeboTopicForwardingMirror(mirror_name));
    if (allowMirrorControl)
      controlServer.reset(new GazeboControlServer(mirror_name));
  }
  WorldManagerPtr worldManager(new WorldManagerT(mirror, controlServer));
  return worldManager;
}
