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
/*
 * Author: Jennifer Buehler
 */
#ifndef COLLISION_BENCHMARK_GAZEBOMULTIPLEWORLDS_H
#define COLLISION_BENCHMARK_GAZEBOMULTIPLEWORLDS_H

#include <collision_benchmark/PhysicsWorldInterfaces.hh>
#include <collision_benchmark/GazeboPhysicsWorld.hh>
#include <collision_benchmark/GazeboWorldState.hh>
#include <collision_benchmark/WorldManager.hh>

#include <collision_benchmark/GazeboMultipleWorldsServer.hh>

#include <gazebo/gazebo.hh>

#include <unistd.h>
#include <sys/wait.h>
#include <memory>
#include <vector>
#include <string>

namespace collision_benchmark
{
/**
 * \brief Convenience class which loads up a MultpleWorldsServer and a gzclient
 * at the same time. Optionally, the non-interactive mode can be used to
 * not load up the gzclient and only load the server.
 *
 * Will also by default load the collision benchmark GUI
 * (collision_benchmark::ClientGui),
 * and provides the option to load additional GUI interfaces.

 * This class uses fork() to start gzclient in the child process.
 *
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

  public: GazeboMultipleWorlds();
  public: virtual ~GazeboMultipleWorlds();

  // Starts the client and initializes the server.
  // Must be called before Run() and before Load().
  // A call to this function will fork the process with fork().
  // \param[in] useInteractiveMode flag whether the gzclient is to be loaded to
  //      allow interactive mode. If false, gzclient is not loaded.
  // \param[in] additionalGuis additional guis to load in gzclient
  public: bool Init(const bool loadMirror = true,
                    const bool enforceContactCalc = false,
                    const bool allowControlViaMirror = true,
                    const bool useInteractiveMode = true,
                    const std::vector<std::string>& additionalGuis = {});

  // \brief Loads an empty world with each of the given engines
  // and sets the physics enabled flag.
  // Before this is called, Init() has to be called.
  public: bool LoadEngines(const std::vector<std::string>& selectedEngines,
                           bool physicsEnabled);

  // \return the world manager.
  public: GzWorldManager::Ptr GetWorldManager();

  // \return the server
  public: GzMultipleWorldsServer::Ptr GetServer() { return server; }

  public: bool InteractiveMode() { return interactiveMode; }

  // \brief Runs the multiple worlds server.
  // Can be run in blocking or non-blocking mode.
  //
  // \param blocking if true, blocks until the client has been closed.
  //    If false, the caller is responsible for repeatedly calling
  //    ``GetWorldManager()->Update()``, and after the simulation is finished,
  //    calling ShutdownServer().
  //    The function HasStarted() can be useful to know when the user has
  //    triggered the start signal, in case \e waitForStartSignal is true.
  //    The function IsClientRunning() can be useful to know whether the
  //    gzclient has been closed by the user.
  // \param waitForStartSignal if true, waits for the user to press [Enter] or
  //    hit the play button in the client before starting the simulation
  // \param loopCallback optional: callback which will be called once per
  //    server update loop. Only will be called if \e blocking is true.
  public: bool Run(bool waitForStartSignal = false,
                   bool blocking = true,
                   const std::function<void(int)>& loopCallback
                      = std::function<void(int)>());

  // \brief stops the client and server
  public: void Stop();

  // \brief flag whether the simulation has been started.
  // This will only be true after the user hit "play" or [Enter] after
  // Run() is being called (if Run() was called with \e waitForStartSignal
  // set to false, then this will be true just before the simulation starts
  // running).
  public: bool HasStarted() const;

  // returns true if gzclient is still running
  public: bool IsClientRunning() const;
  // returns !isClientRunning()
  private: bool IsClientClosed() const;

  // \brief Shuts down the server.
  // Must be called after Run() has been used with
  // \e blocking set to false. Otherwise doesn't need to be called.
  public: void ShutdownServer();

  // \brief Returns the name of the mirror.
  // This is the name of the world which the gzclient will use.
  // This is useful when Init() was called with parameter \e loadMirror
  // set to true, and the application needs to know the name of the
  // mirror being used.
  public: std::string GetMirrorName() const { return MirrorName; }

  protected: bool IsChild() const;
  protected: bool IsParent() const;

    // Initializes the multiple worlds server
  protected: bool InitServer(const bool loadMirror,
                             const bool allowControlViaMirror,
                             const bool enforceContactCalc);


  protected: void KillClient();

  // the name of the mirror. This is the name of the world which the
  // gzclient will use.
  private: static const std::string MirrorName;

  // the server
  private: GzMultipleWorldsServer::Ptr server;

  // the child process ID for running gzclient
  private: pid_t progPID;

  // \brief flag whether the simulation has been started.
  private: std::atomic<bool> started;

  // \brief flag whether the gzclient is to be loaded to allow interactive mode
  private: bool interactiveMode;
};  // class GazeboMultipleWorlds
}  // namespace
#endif  // COLLISION_BENCHMARK_GAZEBOMULTIPLEWORLDS_H
