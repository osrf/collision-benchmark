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
/*
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
 * \brief World which can be set to mirror a PhysicsWorld.
 *
 * The world which mirrors another world is called the *mirror world*.
 * Because it is only a mirror to the other *original* world,
 * manipulation of the original world should not be possible via this interface.
 * It is only possible to view the original world in the mirror.
 *
 * The mirror can be useful for scenarios such as visualization of a world,
 * in which the mirror world is the one used for displaying the original world;
 * it can be switched to display a different world.
 *
 * This is just a basic interface which can have a variety of implementations.
 *
 * \author Jennifer Buehler
 * \date December 2016
 */
class MirrorWorld
{
  public: typedef MirrorWorld Self;
  public: typedef std::shared_ptr<Self> Ptr;
  public: typedef std::shared_ptr<const Self> ConstPtr;

  // the original world type
  public: typedef PhysicsWorldBaseInterface OriginalWorld;
  public: typedef std::shared_ptr<OriginalWorld> OriginalWorldPtr;

  public:  virtual ~MirrorWorld() {}

  /// Sets the original world to be mirrored by this MirrorWorld
  public:  void SetOriginalWorld(const OriginalWorldPtr &_originalWorld)
           {
             NotifyOriginalWorldChange(_originalWorld);
             // DEVELOPER MEMO for Gazebo implementation:
             // transport::Node::Subscribe() locks Node::incomingMutex.
             // Before a callback is called with Node::HandleMessage,
             // incomingMutex is locked as well. Now if the implementation
             // of NotifyOriginalWorldChange calls GetOriginalWorld() and
             // locks the originalWorldMutex WHILE an already incoming
             // message from another thread also calls GetOriginalWorld()
             // while Node::incomingMutex being locked
             // (from Node::ProcessIncoming), we have a deadlock.
             // Therefore, as we don't know what excatly is done in
             // the implementation of NotifyOriginalWorldChange, we can't
             // have a lock in place while calling it, even if it is recursvie.
             // Hence lock it only around the actual access to the local
             // variable.
             std::lock_guard<std::mutex> lock(originalWorldMutex);
             originalWorld = _originalWorld;
           }

  /// Returns the original world which is mirrored by this class
  public:  OriginalWorldPtr GetOriginalWorld() const
           {
             std::lock_guard<std::mutex> lock(originalWorldMutex);
             return originalWorld;
           }

  /// Synchronizes the world with the original
  public:  virtual void Sync() = 0;

  /// \return the name of the mirror world (not the original world).
  ///     Can be used by subclasses in case the mirror worlds are
  ///     named - otherwise returns the default name 'MirrorWorld'.
  public: virtual std::string GetName() const { return "MirrorWorld"; }

  // Will be called when the original world is about to be changed in
  // SetOriginalWorld. Can be used by subclasses.
  protected: virtual void NotifyOriginalWorldChange
                (const OriginalWorldPtr &_newWorld) {}
  private:  OriginalWorldPtr originalWorld;
  private:  mutable std::mutex originalWorldMutex;
};

}  // namespace collision_benchmark
#endif
