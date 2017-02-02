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

#include <gazebo/gazebo.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <string>
#include <iostream>

namespace collision_benchmark
{

/**
 * Simple convenience class which maintains a number of worlds along with an optional
 * MirrorWorld and accepts messages to control switching the mirror world.
 *
 * Communication with the client works via gazebo::Any messages.
 * An INT32 of -1 is received for "Prev", an integer of 1 for "Next",
 * a 0 for no change and simply triggering the sending of the name of the currently
 * mirrored world.
 * Each time a message is received, the current world name is sent back in a gazebo::Any message
 * with type STRING.
 *
 * \author Jennifer Buehler
 * \date December 2016
 */
template<class WorldState>
class WorldManager
{
  private: typedef WorldManager<WorldState> Self;

  public: typedef std::shared_ptr<WorldManager> Ptr;
  public: typedef std::shared_ptr<const WorldManager> ConstPtr;

  // the original world supporting the gazebo::physics::WorldState to synchronize to
  //public: typedef PhysicsWorldStateInterface<gazebo::physics::WorldState> PhysicsWorldStateInterfaceT;

  //public: typedef typename MirrorWorld<WorldState>::Ptr MirrorWorldPtr;
  public: typedef typename MirrorWorld::Ptr MirrorWorldPtr;

  /// Constructor.
  /// \param _mirrorWorld the main mirror world (the one which will reflect the original). Does not
  ///    need to be set to any world yet, will automatically be set to mirror the first world added with
  ///    AddPhysicsWorld.
  public:  WorldManager(const MirrorWorldPtr& _mirrorWorld=MirrorWorldPtr()):
             mirroredWorldIdx(-1)
           {
             SetMirrorWorld(_mirrorWorld);
           }

  public:  ~WorldManager() {}


  /// Sets the mirror world
  /// \param _mirrorWorld the main mirror world (the one which will reflect the original). Does not
  ///    need to be set to any world yet, will automatically be set to mirror the first world added with
  ///    AddPhysicsWorld.
  public: void SetMirrorWorld(const MirrorWorldPtr& _mirrorWorld)
          {
            if (!_mirrorWorld)
            {
              if (mirrorWorld)
              {
                mirrorWorld.reset();
                ctrlClientSubscriber.reset();
                ctrlClientPublisher.reset();
              }
              mirroredWorldIdx=-1;
              return;
            }
            mirrorWorld=_mirrorWorld;
            if (!worlds.empty())
            {
              mirrorWorld->SetOriginalWorld(worlds.front());
              mirroredWorldIdx=0;
            }
            if (!node)
            {
              node.reset(new gazebo::transport::Node());
              node->Init();
            }
            if (!ctrlClientSubscriber) ctrlClientSubscriber = node->Subscribe("mirror_world/set_world", &Self::crtlClientCallback, this);
            if (!ctrlClientPublisher) ctrlClientPublisher  = node->Advertise<gazebo::msgs::Any>("mirror_world/get_world");
          }

  // returns the original world which is currently mirrored by the mirror world
  public:  PhysicsWorldBaseInterface::Ptr GetMirroredWorld()
           {
             assert(mirrorWorld);
             return mirrorWorld->GetOriginalWorld();
           }

  /// Adds this world and returns the index this world can be accessed at
  /// \return positive int or zero on success (index this world can be accessed at)
  public:  void AddPhysicsWorld(const PhysicsWorldBaseInterface::Ptr& _world)
           {
             if (worlds.empty() && mirrorWorld)
             {
               mirrorWorld->SetOriginalWorld(_world);
               mirroredWorldIdx=0;
             }
             worlds.push_back(_world);
           }

  public:  void SetMirroredWorld(const int _index)
           {
             PhysicsWorldBaseInterface::Ptr world=GetPhysicsWorld(_index);
             if (!world)
             {
               gzerr<<"Cannot get world in WorldManager::SetMirroredWorld()\n";
               return;
             }
             mirrorWorld->SetOriginalWorld(world);
             mirroredWorldIdx=_index;
           }


  /// Returns the original world which is mirrored by this class
  public:  PhysicsWorldBaseInterface::Ptr GetPhysicsWorld(unsigned int _index) const
           {
             GZ_ASSERT(_index < worlds.size(), "Index out of range");
             if (_index >= worlds.size()) return PhysicsWorldBaseInterface::Ptr();
             return worlds.at(_index);
           }

  public: std::vector<PhysicsWorldBaseInterface::Ptr> GetPhysicsWorlds() const
          {
            return worlds;
          }

  // Convenience method which casts the world \e w to a PhysicsWorldStateInterface with the given state
  public: template<class WorldState_>
          static typename PhysicsWorldStateInterface<WorldState_>::Ptr ToWorldWithState(PhysicsWorldBaseInterface::Ptr& w)
          {
            return std::dynamic_pointer_cast<PhysicsWorldStateInterface<WorldState_>>(w);
          }

  public: void SetPaused(bool flag)
          {
            for (std::vector<PhysicsWorldBaseInterface::Ptr>::iterator it=worlds.begin(); it!= worlds.end(); ++it)
            {
              PhysicsWorldBaseInterface::Ptr w=*it;
              w->SetPaused(flag);
            }
          }

  /// Set the dynamics engine to enabled or disabled. If disabled, the objects won't react to physics
  /// laws, but objects can be maintained in the world and collision states / contact points between them checked.
  public: virtual void SetDynamicsEnabled(const bool flag)
          {
            for (std::vector<PhysicsWorldBaseInterface::Ptr>::iterator it=worlds.begin(); it!= worlds.end(); ++it)
            {
              PhysicsWorldBaseInterface::Ptr w=*it;
              w->SetDynamicsEnabled(flag);
            }
          }

  /// Calls Update(iter) on all worlds and subsequently calls MirrorWorld::Sync() and MirrorWorld::Update().
  public: void Update(int iter=1)
          {
            for (std::vector<PhysicsWorldBaseInterface::Ptr>::iterator it=worlds.begin(); it!= worlds.end(); ++it)
            {
              PhysicsWorldBaseInterface::Ptr w=*it;
              w->Update(iter);
            }
            if (mirrorWorld)
            {
              mirrorWorld->Sync();
            }
          }

  /// Callback for control client subscriber \e ctrlClientSubscriber.
  /// Will send back the current world name with \e ctrlClientPublisher.
  private:  void crtlClientCallback(ConstAnyPtr &_msg)
            {
              // std::cout << "Received: "<<_msg->DebugString();
              // change the world:
              if (_msg->type() != gazebo::msgs::Any::INT32)
              {
                gzerr<<"Received Control message of invalid type, expecting INT32.\n"<<_msg->DebugString();
                return;
              }
              int ctrl=_msg->int_value();
              if (ctrl < 0)
              {
                // Switch to previous world
                std::cout<<"Switching to previous world"<<std::endl;
                if (mirroredWorldIdx > 0) --mirroredWorldIdx;
                //  else mirroredWorldIdx=worlds.size()-1; // go back to last world
              }
              else if (ctrl > 0)
              {
                // Switch to next world
                std::cout<<"Switching to next world"<<std::endl;
                if (mirroredWorldIdx < (worlds.size()-1)) ++mirroredWorldIdx;
                // else mirroredWorldIdx=0; // go back to first world
              }

              if (ctrl != 0)
              {
                // update mirrored world
                SetMirroredWorld(mirroredWorldIdx);
              }

              // send back the name of the new world:
              SendWorldName(mirrorWorld->GetOriginalWorld()->GetName());
            }

  /// sends the name of a word via \e ctrlClientPublisher
  private: void SendWorldName(const std::string& name)
           {
              gazebo::msgs::Any m;
              m.set_type(gazebo::msgs::Any::STRING);
              m.set_string_value(name);
              ctrlClientPublisher->Publish(m);
           }

  private: gazebo::transport::SubscriberPtr ctrlClientSubscriber;
  private: gazebo::transport::PublisherPtr ctrlClientPublisher;
  private: gazebo::transport::NodePtr node;

  private: std::vector<PhysicsWorldBaseInterface::Ptr> worlds;
  private: MirrorWorldPtr mirrorWorld;
  private: int mirroredWorldIdx;

};

}  // namespace collision_benchmark
#endif
