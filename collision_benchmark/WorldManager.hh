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
#ifndef COLLISION_BENCHMARK_WORLDMANAGER_H
#define COLLISION_BENCHMARK_WORLDMANAGER_H

#include <collision_benchmark/PhysicsWorld.hh>
#include <collision_benchmark/MirrorWorld.hh>
#include <collision_benchmark/ControlServer.hh>
#include <collision_benchmark/BasicTypes.hh>
#include <collision_benchmark/TypeHelper.hh>

#include <gazebo/gazebo.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <string>
#include <iostream>
#include <mutex>

namespace collision_benchmark
{

/**
 * \brief Simple convenience class which maintains a number of worlds
 * along with an optional MirrorWorld and accepts messages to control
 * switching the mirror world.
 *
 * Communication with the client works via gazebo::Any messages.
 * An INT32 of -1 is received for "Prev", an integer of 1 for "Next",
 * a 0 for no change and simply triggering the sending of the name of
 * the currently mirrored world.
 * Each time a message is received, the current world name is sent back in
 * a gazebo::Any message with type STRING.
 *
 * \param _WorldState describes the state of a world.
 *         XXX Note: will probably be removed?
 * \param _ModelID the identifier for a specific model
 * \param _ModelPartID the identifier for a part of a model
 *
 * \author Jennifer Buehler
 * \date December 2016
 */
template<class _WorldState, class _ModelID, class _ModelPartID>
class WorldManager
{
  public: typedef _WorldState WorldState;
  public: typedef _ModelID ModelID;
  public: typedef _ModelPartID ModelPartID;
  private: typedef WorldManager<WorldState, ModelID, ModelPartID> Self;

  public: typedef std::shared_ptr<WorldManager> Ptr;
  public: typedef std::shared_ptr<const WorldManager> ConstPtr;

  // the world interface supporting the WorldState
  public: typedef PhysicsWorldStateInterface<WorldState>
              PhysicsWorldStateInterfaceT;
  public: typedef typename PhysicsWorldStateInterfaceT::Ptr
              PhysicsWorldStateInterfacePtr;

  //public: typedef typename MirrorWorld<WorldState>::Ptr MirrorWorldPtr;
  public: typedef typename MirrorWorld::Ptr MirrorWorldPtr;
  public: typedef typename ControlServer<ModelID>::Ptr ControlServerPtr;

  /// Constructor.
  /// \param _mirrorWorld the main mirror world (the one which will reflect
  ///   the original). Does not need to be set to mirror any particular world
  ///     yet, will automatically be set to mirror the first world added with
  ///     AddPhysicsWorld().
  /// \param _controlServer server which receives control commands
  ///     for the world(s). If NULL, worlds cannot be controlled.
  public:  WorldManager(const MirrorWorldPtr &_mirrorWorld = MirrorWorldPtr(),
                        const ControlServerPtr &_controlServer
                            = ControlServerPtr()):
             mirroredWorldIdx(-1),
             controlServer(_controlServer)
           {
             this->SetMirrorWorld(_mirrorWorld);
             if (this->controlServer)
             {
               this->controlServer->RegisterPauseCallback
                 (std::bind(&Self::NotifyPause, this, std::placeholders::_1));
               this->controlServer->RegisterUpdateCallback
                 (std::bind(&Self::NotifyUpdate, this, std::placeholders::_1));
               this->controlServer->RegisterSetModelStateCallback
                 (std::bind(&Self::NotifyModelStateChange, this,
                            std::placeholders::_1,
                            std::placeholders::_2));
               this->controlServer->RegisterSdfModelLoadCallback
                 (std::bind(&Self::NotifySdfModelLoad, this,
                            std::placeholders::_1,
                            std::placeholders::_2,
                            std::placeholders::_3));

               this->controlServer->RegisterSelectWorldService
                 (std::bind(&Self::ChangeMirrorWorld,
                            this, std::placeholders::_1));
             }
           }

  public:  ~WorldManager() {}


  /// Sets the mirror world
  /// \param _mirrorWorld the main mirror world (the one which will reflect
  ///       the original). Does not need to be set to any world yet, will
  ///       automatically be set to mirror the first world added with
  ///       AddPhysicsWorld.
  public: void SetMirrorWorld(const MirrorWorldPtr& _mirrorWorld)
          {
            if (!_mirrorWorld)
            {
              if (this->mirrorWorld)
              {
                this->mirrorWorld.reset();
//                this->ctrlClientSubscriber.reset();
//                this->ctrlClientPublisher.reset();
              }
              this->mirroredWorldIdx=-1;
              return;
            }
            this->mirrorWorld=_mirrorWorld;
            {
              std::lock_guard<std::recursive_mutex> lock(this->worldsMutex);
              if (!this->worlds.empty())
              {
                this->mirrorWorld->SetOriginalWorld(worlds.front());
                this->mirroredWorldIdx=0;
              }
            }
          }

  // returns the original world which is currently mirrored by the mirror world
  public:  PhysicsWorldBaseInterface::Ptr GetMirroredWorld()
           {
             assert(this->mirrorWorld);
             return this->mirrorWorld->GetOriginalWorld();
           }

  /// Adds this world and returns the index this world can be accessed at
  /// \return positive int or zero on success (index this world
  ///         can be accessed at)
  public:  void AddPhysicsWorld(const PhysicsWorldBaseInterface::Ptr& _world)
           {
             std::lock_guard<std::recursive_mutex> lock(this->worldsMutex);
             if (this->worlds.empty() && this->mirrorWorld)
             {
               this->mirrorWorld->SetOriginalWorld(_world);
               this->mirroredWorldIdx=0;
             }
             this->worlds.push_back(_world);
           }

  public:  void SetMirroredWorld(const int _index)
           {
             std::cout<<"Getting world at idx "<<_index<<std::endl;
             PhysicsWorldBaseInterface::Ptr world=GetPhysicsWorld(_index);
             if (!world)
             {
               gzerr<<"Cannot get world in WorldManager::SetMirroredWorld()\n";
               return;
             }
             this->mirrorWorld->SetOriginalWorld(world);
             this->mirroredWorldIdx=_index;
           }

  /// Returns the original world which is mirrored by this class
  public:  size_t GetNumWorlds() const
           {
             std::lock_guard<std::recursive_mutex> lock(this->worldsMutex);
             return this->worlds.size();
           }


  /// Returns the original world which is mirrored by this class
  public:  PhysicsWorldBaseInterface::Ptr
           GetPhysicsWorld(unsigned int _index) const
           {
             std::lock_guard<std::recursive_mutex> lock(this->worldsMutex);
             GZ_ASSERT(_index >=0 && _index < this->worlds.size(),
                       "Index out of range");
             if (_index >= this->worlds.size())
             {
               return PhysicsWorldBaseInterface::Ptr();
             }
             return this->worlds.at(_index);
           }

/*  public: std::vector<PhysicsWorldBaseInterface::Ptr> GetPhysicsWorlds() const
          {
            std::lock_guard<std::recursive_mutex> lock(this->worldsMutex);
            return this->worlds;
          }*/

  // Convenience method which casts the world \e w to a
  // PhysicsWorldStateInterface with the given state
  public: template<class WorldState_>
          static typename PhysicsWorldStateInterface<WorldState_>::Ptr
            ToWorldWithState(const PhysicsWorldBaseInterface::Ptr& w)
          {
            return std::dynamic_pointer_cast
                   <PhysicsWorldStateInterface<WorldState_>>(w);
          }

  // Convenience method which casts the world \e w to a
  // PhysicsWorldStateInterface with the given state
  public: template<class ModelID_, class ModelPartID_>
          static typename
            PhysicsWorldModelInterface<ModelID_, ModelPartID_>::Ptr
            ToWorldWithModel(const PhysicsWorldBaseInterface::Ptr& w)
          {
            return std::dynamic_pointer_cast
                   <PhysicsWorldModelInterface<ModelID_, ModelPartID_>>(w);
          }


  public: void SetPaused(bool flag)
          {
            std::lock_guard<std::recursive_mutex> lock(this->worldsMutex);
            for (std::vector<PhysicsWorldBaseInterface::Ptr>::iterator
                 it=this->worlds.begin();
                 it != this->worlds.end(); ++it)
            {
              PhysicsWorldBaseInterface::Ptr w=*it;
              w->SetPaused(flag);
            }
          }

  /// \brief Set the dynamics engine to enabled or disabled.
  /// If disabled, the objects won't react to physics
  /// laws, but objects can be maintained in the world and
  /// collision states / contact points between them checked.
  public: virtual void SetDynamicsEnabled(const bool flag)
          {
            std::lock_guard<std::recursive_mutex> lock(this->worldsMutex);
            for (std::vector<PhysicsWorldBaseInterface::Ptr>::iterator
                 it = this->worlds.begin();
                 it != this->worlds.end(); ++it)
            {
              PhysicsWorldBaseInterface::Ptr w=*it;
              w->SetDynamicsEnabled(flag);
            }
          }

  /// Calls PhysicsWorld::Update(iter,force) on all worlds and subsequently
  /// calls MirrorWorld::Sync() and MirrorWorld::Update().
  public: void Update(int iter=1, bool force=false)
          {
            // we cannot just lock the worldMutex with a lock here, because
            // calling Update() may trigger the call of callbacks in this
            // class, called by the ControlServer. ControlServer implementations
            // may trigger the call of the callbacks from a different thread,
            // therefore there will be a deadlock for accessing the worlds in
            // the callback functions of this class. Only block the worlds
            // vector while absolutey necessary.
            // std::cout<<"__________UPDATE__________"<<std::endl;
            this->worldsMutex.lock();
            int numWorlds=this->worlds.size();
            this->worldsMutex.unlock();
            for (int i=0; i< numWorlds; ++i)
            {
              PhysicsWorldBaseInterface::Ptr world;
              {
                // get the i'th world
                std::lock_guard<std::recursive_mutex> lock(this->worldsMutex);
                // update vector size in case more worlds were
                // added asynchronously
                numWorlds=this->worlds.size();
                // break loop if size of worlds has decreased
                if (i >= numWorlds) break;
                world=worlds[i];
              }
              world->Update(iter, force);
            }
            if (this->mirrorWorld)
            {
              this->mirrorWorld->Sync();
            }
            // std::cout<<"__________UPDATE END__________"<<std::endl;
          }

  private: void NotifyPause(const bool _flag)
           {
             // std::cout<<"PAUSE FLAG "<<_flag<<std::endl;
             SetPaused(_flag);
           }

  private: void NotifyUpdate(const int _numSteps)
           {
             // std::cout<<"UPDATE "<<_numSteps<<std::endl;
             Update(_numSteps, true);
           }

  private: void NotifyModelStateChange(const ModelID  &_id,
                                    const BasicState &_state)
           {
              // std::cout<<"STATE CHANGE "<<_id<<": "<<_state<<std::endl;
              std::lock_guard<std::recursive_mutex> lock(this->worldsMutex);
              for (std::vector<PhysicsWorldBaseInterface::Ptr>::iterator
                   it = this->worlds.begin();
                   it != this->worlds.end(); ++it)
              {
                typename PhysicsWorldModelInterface<ModelID, ModelPartID>::Ptr
                  w = ToWorldWithModel<ModelID, ModelPartID>(*it);
                if (!w)
                {
                  THROW_EXCEPTION("Only support worlds which have the "
                                  << "interface PhysicsWorldModelInterface<"
                                  << GetTypeName<ModelID>()
                                  << ", "<<GetTypeName<ModelPartID>()<<">");
                }
                if (!w->SetBasicModelState(_id,_state))
                {
                  std::cerr<<"Could not set basic model state because model "<<
                    _id<<" does not exist."<<std::endl;
                }
              }
           }


  private: void NotifySdfModelLoad(const std::string& _sdf,
                                   const bool _isString,
                                   const BasicState &_state)
           {
              std::lock_guard<std::recursive_mutex> lock(this->worldsMutex);
              for (std::vector<PhysicsWorldBaseInterface::Ptr>::iterator
                   it = this->worlds.begin();
                   it != this->worlds.end(); ++it)
              {
                typename PhysicsWorldModelInterface<ModelID, ModelPartID>::Ptr
                  w =ToWorldWithModel<ModelID, ModelPartID>(*it);
                if (!w)
                {
                  THROW_EXCEPTION("Only support worlds which have the "
                                  << "interface PhysicsWorldModelInterface<"
                                  << GetTypeName<ModelID>()
                                  << ", "<<GetTypeName<ModelPartID>()<<">");
                }

                if (_isString)
                {
                  w->AddModelFromString(_sdf);
                }
                else
                {
                  w->AddModelFromFile(_sdf);
                }
              }
           }

  private: std::string ChangeMirrorWorld(const int ctrl)
           {
              std::lock_guard<std::recursive_mutex> lock(this->worldsMutex);
              int oldMirrorIdx = mirroredWorldIdx;
              if (ctrl < 0)
              {
                // Switch to previous world
                std::cout<<"WorldManager: Switching to prev world"<<std::endl;
                if (mirroredWorldIdx > 0) --mirroredWorldIdx;
                else mirroredWorldIdx=worlds.size()-1; // go back to last world
              }
              else if (ctrl > 0)
              {
                // Switch to next world
                std::cout<<"WorldManager: Switching to next world"<<std::endl;
                if (mirroredWorldIdx < (worlds.size()-1)) ++mirroredWorldIdx;
                else mirroredWorldIdx=0; // go back to first world
              }

              if (mirroredWorldIdx == oldMirrorIdx)
                  return mirrorWorld->GetOriginalWorld()->GetName();

              // update mirrored world
              this->SetMirroredWorld(mirroredWorldIdx);

              std::cout << "WorldManager: New world is "
                        << mirrorWorld->GetOriginalWorld()->GetName()
                        << std::endl;

              // return the name of the new world
              return mirrorWorld->GetOriginalWorld()->GetName();
            }


  // all the worlds
  private: std::vector<PhysicsWorldBaseInterface::Ptr> worlds;
  // mutex protecting the worlds vector (not the worlds itself!)
  private: mutable std::recursive_mutex worldsMutex;

  private: MirrorWorldPtr mirrorWorld;
  private: int mirroredWorldIdx;

  private: ControlServerPtr controlServer;

};

}  // namespace collision_benchmark
#endif
