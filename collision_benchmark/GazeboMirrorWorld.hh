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
 * Date: May 2016
 */
#ifndef COLLISION_BENCHMARK_GAZEBOMIRRORWORLD_H
#define COLLISION_BENCHMARK_GAZEBOMIRRORWORLD_H

#include <gazebo/gazebo.hh>
#include <gazebo/physics/World.hh>
#include <collision_benchmark/PhysicsWorld.hh>
#include <string>
#include <iostream>

namespace collision_benchmark
{

/**
 * \brief Maintains a world of its own, and can be set to mirror another physics::World.
 * The world contains a *copy* of the other worlds state (models, lights, etc)
 * but maintains its own collision engine, which is disabled.
 *
 * The main purpose of this class is to create a main world which can be viewed with gzclient,
 * and which can be switched to mirror a number of different worlds which are running concurrently.
 *
 * Current drawbacks which still need to be handled:
 * - The contact points published by the mirror world are still the ones which the mirror world itself,
 *   not the mirrored world. This may be a desirable feature to keep, but it should be possible to mirror
 *   the original world's contact points as well.
 * - Manipulating a shape by a request message will manipulate the shape of the mirror world, not the
 *   world being mirrored. This will have to be handled by somehow redirecting World::ProcessMessages
 *
 * Other things I still don't like or am unsure how to handle:
 * - not sure how to handle times/iterations in the states when transferring them in-between worlds
 * - Exception handling has to be worked over
 * - The SDF of the insertions returned by WorldState::operator- have to be manually fixed to contain the <sdf> tag.
 *   Otherwise it can't be used in WorldState::SetState(). Current solution is not so pretty.
 * - WorldState::operator- does not return the current state of newly inserted objects. When switching
 *   between worlds, an update of the state has to be enforced right after the application of the new
 *   state which added the models/lights. Otherwise, no pose message may be published if the state of
 *   the model hasn't changed within its own world. This causes a "flickering" of the model state
 *   (briefly sowhn at its original pose in the SDF) in the viewer, despite all changes happening in
 *   a paused world. It needs to be checked whether this can be improved.
 * - waiting for namespaces can still be improved, sometimes it may fail (see also PrivateHelpers)
 *
 * \author Jennifer Buehler
 * \date May 2016
 */
class GazeboMirrorWorld
{
    public: typedef std::shared_ptr<GazeboMirrorWorld> Ptr;
    public: typedef std::shared_ptr<const GazeboMirrorWorld> ConstPtr;

    // the original world supporting the gazebo::physics::WorldState to synchronize to
    public: typedef PhysicsWorldBase<gazebo::physics::WorldState> OriginalWorld;
    public: typedef std::shared_ptr<OriginalWorld> OriginalWorldPtr;

    /// Constructor.
    /// \param mirrorWorld the main mirror world (the one which will reflect the original).
    ///     The physics engine will be disabled.
    public:  GazeboMirrorWorld(gazebo::physics::WorldPtr& mirrorWorld);

    // prohibit copy constructor
    private: GazeboMirrorWorld(const GazeboMirrorWorld& o){}
    public:  ~GazeboMirrorWorld();

    /// Returns the mirror world
    public:  gazebo::physics::WorldPtr GetMirrorWorld() const;

    /// Sets the original world to be mirrored by this GazeboMirrorWorld
    public:  void SetOriginalWorld(const OriginalWorldPtr& originalWorld);

    /// Returns the original world which is mirrored by this class
    public:  OriginalWorldPtr GetOriginalWorld() const;

    /// Synchronizes the world with the original
    public:  void Sync();

    /// Clears all models from the current mirror world.
    public: virtual void ClearModels();

    /// Synchronizes the mirror world to the original by calling Sync(),
    /// and subsequently updates the mirror world.
    public: virtual void Update(int iter=1);

    protected:  gazebo::physics::WorldPtr mirrorWorld;
    protected:  OriginalWorldPtr originalWorld;
};

}  // namespace collision_benchmark
#endif
