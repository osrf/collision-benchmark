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
#ifndef COLLISION_BENCHMARK_GAZEBOMULTIPLEWORLDS_H
#define COLLISION_BENCHMARK_GAZEBOMULTIPLEWORLDS_H

#include <collision_benchmark/PhysicsWorld.hh>
#include <collision_benchmark/GazeboPhysicsWorld.hh>
#include <collision_benchmark/GazeboWorldState.hh>
#include <collision_benchmark/WorldManager.hh>

#include <collision_benchmark/GazeboMultipleWorldsServer.hh>

#include <gazebo/gazebo.hh>

#include <unistd.h>
#include <sys/wait.h>
#include <memory>

namespace collision_benchmark
{

/**
 * Helper class which loads up a MultpleWorldsServer and a gzclient
 * \author Jennifer Buehler
 * \date May 2017
 */
class GazeboMultipleWorlds
{

  public: typedef std::shared_ptr<GazeboMultipleWorlds> Ptr;
  public: typedef std::shared_ptr<const GazeboMultipleWorlds> ConstPtr;

  private: typedef collision_benchmark::MultipleWorldsServer
                        <GazeboPhysicsWorldTypes::WorldState,
                         GazeboPhysicsWorldTypes::ModelID,
                         GazeboPhysicsWorldTypes::ModelPartID,
                         GazeboPhysicsWorldTypes::Vector3,
                         GazeboPhysicsWorldTypes::Wrench>
                          GzMultipleWorldsServer;

  public: typedef collision_benchmark::WorldManager
                    <GazeboPhysicsWorldTypes::WorldState,
                     GazeboPhysicsWorldTypes::ModelID,
                     GazeboPhysicsWorldTypes::ModelPartID,
                     GazeboPhysicsWorldTypes::Vector3,
                     GazeboPhysicsWorldTypes::Wrench>
                        GzWorldManager;

  public: virtual ~GazeboMultipleWorlds();

  // Loads the client and initializes the server.
  // Must be called before Run().
  // A call to this function will fork the process, returning one parent
  // and one child. Though only the parent returns from this call.
  public: bool Load(const std::vector<std::string>& selectedEngines,
                    bool physicsEnabled = true,
                    bool loadMirror = true,
                    bool enforceContactCalc=false,
                    bool allowControlViaMirror = true);

  // \return the world manager.
  public: GzWorldManager::Ptr GetWorldManager();

  // Runs the multiple worlds server. Blocks until the client has been closed.
  // \param waitForStartSignal if true, waits for the user to press [Enter] or
  //    hit the play button in the client before starting the simulation
  // \param loopCallback optional: callback which will be called once per
  //    server update loop
  public: bool Run(bool waitForStartSignal = false,
                   const std::function<void(int)>& loopCallback
                      = std::function<void(int)>());

  // \brief Returns the name of the mirror.
  // This is the name of the world which the gzclient will use.
  // This is useful when Load() was called with parameter \e loadMirror
  // set to true, and the application needs to know the name of the
  // mirror being used.
  public: std::string GetMirrorName() const { return MirrorName; }

  protected: bool IsChild() const;
  protected: bool IsParent() const;

    // Initializes the multiple worlds server
  protected: bool Init(const bool loadMirror,
                       const bool allowControlViaMirror,
                       const bool enforceContactCalc);

  // returns true if gzclient is still running
  protected: bool IsClientRunning();

  protected: void KillClient();

  // the name of the mirror. This is the name of the world which the
  // gzclient will use.
  private: static const std::string MirrorName;

  // the server
  private: GzMultipleWorldsServer::Ptr server;

  // the child process ID for running gzclient
  private: pid_t gzclient_pid;

};  // class GazeboMultipleWorlds

} // namespace

#endif  // COLLISION_BENCHMARK_GAZEBOMULTIPLEWORLDS_H
