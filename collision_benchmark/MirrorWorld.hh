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
 * \brief Maintains a world of its own, and can be set to mirror another physics::World.
 *
 * The world which mirrors another world is called the ''mirror world''.
 *
 * This is just a basic interface which can have a variety of implementations.
 *
 * \author Jennifer Buehler
 * \date December 2016
 */
template<class WorldState>
class MirrorWorld
{
    public: typedef std::shared_ptr<MirrorWorld> Ptr;
    public: typedef std::shared_ptr<const MirrorWorld> ConstPtr;

    // the original world type
    public: typedef PhysicsWorldBase<WorldState> OriginalWorld;
    public: typedef std::shared_ptr<OriginalWorld> OriginalWorldPtr;

    public:  ~MirrorWorld(){}

    /// Sets the original world to be mirrored by this MirrorWorld
    public:  void SetOriginalWorld(const OriginalWorldPtr& _originalWorld)
             {
               std::lock_guard<std::mutex> lock(originalWorldMutex);
               originalWorld=_originalWorld;
             }

    /// Returns the original world which is mirrored by this class
    public:  OriginalWorldPtr GetOriginalWorld() const
             {
               std::lock_guard<std::mutex> lock(originalWorldMutex);
               return originalWorld;
             }

    /// Synchronizes the world with the original
    public:  virtual void Sync()=0;

    /// Clears all models from the current mirror world.
    public: virtual void ClearModels()=0;

    /// Updates the mirror world. Typically should be called right after Sync().
    public: virtual void Update(int iter=1)=0;

    protected:  OriginalWorldPtr originalWorld;
    protected:  mutable std::mutex originalWorldMutex;
};

}  // namespace collision_benchmark
#endif
