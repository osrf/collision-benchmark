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
/* Desc: World adaptor mirroring another physics::World
 * Author: Jennifer Buehler
 * Date: December 2016
 */
#ifndef COLLISION_BENCHMARK_MIRRORWORLD_H
#define COLLISION_BENCHMARK_MIRRORWORLD_H

#include <collision_benchmark/PhysicsWorld.hh>
#include <mutex>
#include <string>

namespace collision_benchmark
{

/**
 * \brief World which can be set to mirror another physics::World.
 *
 * The world which mirrors another world is called the ''mirror world''.
 * Because it is a mirror, manipulation of the original world should not
 * be possible via this interface.
 *
 * This is just a basic interface which can have a variety of implementations.
 *
 * \author Jennifer Buehler
 * \date December 2016
 */
//template<class WorldState_>
class MirrorWorld
{
//  public: typedef WorldState_ WorldState;
//  public: typedef MirrorWorld<WorldState> Self;
  public: typedef MirrorWorld Self;
  public: typedef std::shared_ptr<Self> Ptr;
  public: typedef std::shared_ptr<const Self> ConstPtr;

  // the original world type
  //public: typedef PhysicsWorldStateInterface<WorldState> OriginalWorld;
  public: typedef PhysicsWorldBaseInterface OriginalWorld;
  public: typedef std::shared_ptr<OriginalWorld> OriginalWorldPtr;

  public:  ~MirrorWorld(){}

  /// Sets the original world to be mirrored by this MirrorWorld
  public:  void SetOriginalWorld(const OriginalWorldPtr& _originalWorld)
           {
             std::lock_guard<std::recursive_mutex> lock(originalWorldMutex);
             NotifyOriginalWorldChange(_originalWorld);
             originalWorld=_originalWorld;
           }

  /// Returns the original world which is mirrored by this class
  public:  OriginalWorldPtr GetOriginalWorld() const
           {
             std::lock_guard<std::recursive_mutex> lock(originalWorldMutex);
             return originalWorld;
           }

  /// Synchronizes the world with the original
  public:  virtual void Sync()=0;

  // Will be called when the original world is about to be changed in
  // SetOriginalWorld. Can be used by subclasses.
  protected: virtual void NotifyOriginalWorldChange
                (const OriginalWorldPtr &_newWorld) {}
  private:  OriginalWorldPtr originalWorld;
  private:  mutable std::recursive_mutex originalWorldMutex;
};

}  // namespace collision_benchmark
#endif
