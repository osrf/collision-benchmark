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
#ifndef COLLISION_BENCHMARK_GAZEBOWORLDLOADER_H
#define COLLISION_BENCHMARK_GAZEBOWORLDLOADER_H

#include <collision_benchmark/PhysicsWorld.hh>

namespace collision_benchmark
{

/**
 * \brief Factory class to help load worlds using a specific physics engine.
 *
 * \author Jennifer Buehler
 * \date March 2017
 */
class WorldLoader
{
  public: typedef std::shared_ptr<WorldLoader> Ptr;
  public: typedef std::shared_ptr<const WorldLoader> ConstPtr;

  // \param _engine name of the engine which will be used for the created
  //    worlds. If empty, this loader is to be considered universal and will
  //    load worlds with the physics engine specified in the file/SDF.
  public: WorldLoader(const std::string& _engine):
          engine(_engine) {}

  // \sa PhysicsWorldBaseInterface::LoadFromSDF
  public: virtual PhysicsWorldBaseInterface::Ptr
          LoadFromSDF(const sdf::ElementPtr& sdf,
                      const std::string& worldname="") const = 0;

  // \sa PhysicsWorldBaseInterface::LoadFromFile
  public: virtual PhysicsWorldBaseInterface::Ptr
          LoadFromFile(const std::string& filename,
                       const std::string& worldname="") const = 0;

  // \sa PhysicsWorldBaseInterface::LoadFromString
  public: virtual PhysicsWorldBaseInterface::Ptr
          LoadFromString(const std::string& str,
                         const std::string& worldname="") const = 0;

  public: std::string EngineName() const { return engine; }
  private: std::string engine;
};

}  // namespace

#endif  // COLLISION_BENCHMARK_GAZEBOWORLDLOADER_H
