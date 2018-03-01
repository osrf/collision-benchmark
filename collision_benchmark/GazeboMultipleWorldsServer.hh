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
#ifndef COLLISION_BENCHMARK_GAZEBOMULTIPLEWORLDSSERVER_H
#define COLLISION_BENCHMARK_GAZEBOMULTIPLEWORLDSSERVER_H

#include <collision_benchmark/MultipleWorldsServer.hh>
#include <collision_benchmark/GazeboPhysicsWorld.hh>
#include <collision_benchmark/GazeboWorldLoader.hh>

#include <gazebo/physics/WorldState.hh>
#include <ignition/math/Vector3.hh>
#include <atomic>
#include <string>

namespace collision_benchmark
{
/**
 * \brief Implementation of MultipleWorldServer for Gazebo
 *
 * \author Jennifer Buehler
 * \date March 2017
 */
class GazeboMultipleWorldsServer
  : public MultipleWorldsServer<GazeboPhysicsWorldTypes::WorldState,
                              GazeboPhysicsWorldTypes::ModelID,
                              GazeboPhysicsWorldTypes::ModelPartID,
                              GazeboPhysicsWorldTypes::Vector3,
                              GazeboPhysicsWorldTypes::Wrench>
{
  private: typedef MultipleWorldsServer<GazeboPhysicsWorldTypes::WorldState,
                              GazeboPhysicsWorldTypes::ModelID,
                              GazeboPhysicsWorldTypes::ModelPartID,
                              GazeboPhysicsWorldTypes::Vector3,
                              GazeboPhysicsWorldTypes::Wrench> Super;

  /**
   * \brief Calls superclass constructor with same parameters
   */
  public: GazeboMultipleWorldsServer
              (const WorldLoader_M &worldLoaders,
               const WorldLoader::ConstPtr &universalLoader =
                 WorldLoader::ConstPtr(new GazeboWorldLoader())):
          Super(worldLoaders, universalLoader),
          running(false) {}
  public: virtual ~GazeboMultipleWorldsServer() { Stop(); }
  public: virtual bool Start(int argc, const char** argv);
  public: virtual void Stop();
  public: virtual bool IsRunning() const;
  protected: virtual WorldManagerPtr
             createWorldManager(const std::string &mirror_name = "",
                                const bool allowMirrorControl = false);
  private: std::atomic<bool> running;
};  // class GazeboMultipleWorldsServer


}  // namespace

#endif  // COLLISION_BENCHMARK_GAZEBOMULTIPLEWORLDSSERVER_H
