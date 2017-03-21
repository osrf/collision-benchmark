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
 * \param _ModelID the identifier for a specific model
 * \param _ModelPartID the identifier for a part of a model
 * \param _Vector3 Math 3D vector implementation
 * \param _Wrench Math wrench implementation
 *
 * \author Jennifer Buehler
 * \date December 2016
 */
template<class _WorldState, class _ModelID,
         class _ModelPartID, class _Vector3, class _Wrench>
class WorldManager
{
  public: typedef _WorldState WorldState;
  public: typedef _ModelID ModelID;
  public: typedef _ModelPartID ModelPartID;
  public: typedef _Vector3 Vector3;
  public: typedef _Wrench Wrench;

  private: typedef WorldManager<WorldState, ModelID,
                                ModelPartID, Vector3, Wrench> Self;

  public: typedef std::shared_ptr<WorldManager> Ptr;
  public: typedef std::shared_ptr<const WorldManager> ConstPtr;

  // the world interface supporting the WorldState
  public: typedef PhysicsWorldStateInterface<WorldState>
            PhysicsWorldStateInterfaceT;
  public: typedef typename PhysicsWorldStateInterfaceT::Ptr
            PhysicsWorldStateInterfacePtr;

  public: typedef PhysicsWorldModelInterface<ModelID, ModelPartID,
            Vector3> PhysicsWorldModelInterfaceT;
  public: typedef typename PhysicsWorldModelInterfaceT::ModelLoadResult
            ModelLoadResult;
  public: typedef typename PhysicsWorldModelInterfaceT::Ptr
            PhysicsWorldModelInterfacePtr;

  public: typedef PhysicsWorldContactInterface<ModelID, ModelPartID,
            Vector3, Wrench> PhysicsWorldContactInterfaceT;
  public: typedef typename PhysicsWorldContactInterfaceT::Ptr
            PhysicsWorldContactInterfacePtr;

  public: typedef PhysicsWorld<WorldState, ModelID, ModelPartID,
            Vector3, Wrench> PhysicsWorldT;
  public: typedef typename PhysicsWorldT::Ptr
            PhysicsWorldPtr;


  public: typedef typename MirrorWorld::Ptr MirrorWorldPtr;
  public: typedef typename MirrorWorld::ConstPtr MirrorWorldConstPtr;
  public: typedef typename ControlServer<ModelID>::Ptr ControlServerPtr;

  /// Constructor.
  /// \param _mirrorWorld the main mirror world (the one which will reflect
  ///   the original). Does not need to be set to mirror any particular world
  ///     yet, will automatically be set to mirror the first world added with
  ///     AddPhysicsWorld().
  /// \param _controlServer server which receives control commands
  ///     for the world(s). If NULL, worlds cannot be controlled.
  /// \param _activeControl if true, the control server \e _controlServer will
  ///        allow all functions incl. the onew which manipulate the world,
  ///        itself, such as adding models, changing model poses, changing
  ///        gravity etc. If false, only basic controls for passively viewing
  ///        the world are allowed.
  public: WorldManager(const MirrorWorldPtr &_mirrorWorld = MirrorWorldPtr(),
                       const ControlServerPtr &_controlServer
                           = ControlServerPtr(),
                       const bool _activeControl = true):
            mirroredWorldIdx(-1),
            controlServer(_controlServer)
  {
    this->SetMirrorWorld(_mirrorWorld);
    if (this->controlServer)
    {
      if (_activeControl)
      {
        this->controlServer->RegisterPauseCallback
          (std::bind(&Self::NotifyPause, this, std::placeholders::_1));

        this->controlServer->RegisterUpdateCallback
          (std::bind(&Self::NotifyUpdate, this,
                     std::placeholders::_1));

        this->controlServer->RegisterSetModelStateCallback
          (std::bind(&Self::NotifyModelStateChange, this,
                     std::placeholders::_1,
                     std::placeholders::_2));

        this->controlServer->RegisterSdfModelLoadCallback
          (std::bind(&Self::NotifySdfModelLoad, this,
                     std::placeholders::_1,
                     std::placeholders::_2,
                     std::placeholders::_3));

        this->controlServer->RegisterDynamicsEnableCallback
          (std::bind(&Self::SetDynamicsEnabled, this,
                     std::placeholders::_1));

        // not supported yet but
        /* this->controlServer->RegisterGravityCallback
          (std::bind(&Self::NotifyGravity, this,
                     std::placeholders::_1,
                     std::placeholders::_2,
                     std::placeholders::_3));*/
      }
      this->controlServer->RegisterSelectWorldService
        (std::bind(&Self::ChangeMirrorWorld,
                   this, std::placeholders::_1));
    }
  }

  public: ~WorldManager() {}


  /// Sets the mirror world. This world can be set to mirror any of the worlds,
  /// for example for visualization of the currently selected world.
  ///
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

  // MirrorWorldPtr GetMirrorWorld() { return this->mirrorWorld; }
  MirrorWorldConstPtr GetMirrorWorld() const { return this->mirrorWorld; }

  // returns the original world which is currently mirrored by the mirror world
  public: PhysicsWorldBaseInterface::Ptr GetMirroredWorld()
  {
    assert(this->mirrorWorld);
    return this->mirrorWorld->GetOriginalWorld();
  }

  /// Adds this world and returns the index this world can be accessed at
  /// \return positive int or zero on success (index this world
  ///         can be accessed at). Negative if a world with this name
  ///         already exists.
  public: int AddPhysicsWorld(const PhysicsWorldBaseInterface::Ptr& _world)
  {
    std::lock_guard<std::recursive_mutex> lock(this->worldsMutex);
    if (GetWorld(_world->GetName()))
    {
      std::cerr << "World with this name already exists! " << std::endl;
      return -1;
    }
    if (this->worlds.empty() && this->mirrorWorld)
    {
      this->mirrorWorld->SetOriginalWorld(_world);
      this->mirroredWorldIdx=0;
    }
    this->worlds.push_back(_world);
    return this->worlds.size()-1;
  }

  public: bool SetMirroredWorld(const int _index)
  {

    if (_index < 0 ||  _index >= this->worlds.size())
      return false;

    // std::cout<<"Getting world at idx "<<_index<<std::endl;
    PhysicsWorldBaseInterface::Ptr world=GetWorld(_index);
    if (!world)
    {
      gzerr<<"Cannot get world in WorldManager::SetMirroredWorld()\n";
      return false;
    }
    this->mirrorWorld->SetOriginalWorld(world);
    this->mirroredWorldIdx=_index;
    return true;
  }

  /// Returns the original world which is mirrored by this class
  public: size_t GetNumWorlds() const
  {
    std::lock_guard<std::recursive_mutex> lock(this->worldsMutex);
    return this->worlds.size();
  }


  /// Returns the original world which is mirrored by this class
  public: PhysicsWorldBaseInterface::Ptr GetWorld(unsigned int _index) const
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

  public: PhysicsWorldBaseInterface::Ptr GetWorld(const std::string& name) const
  {
     std::lock_guard<std::recursive_mutex> lock(this->worldsMutex);
     for (std::vector<PhysicsWorldBaseInterface::Ptr>::const_iterator
          it = this->worlds.begin();
          it != this->worlds.end(); ++it)
     {
       PhysicsWorldBaseInterface::Ptr w = *it;
       assert(w);
       if (w->GetName() == name) return w;
     }
     return PhysicsWorldBaseInterface::Ptr();
  }

  /// Returns all worlds.
  /// Note that accessing the returned worlds asynchronously may lead to
  /// thread safety issues. It is recommended to access the returned
  /// worlds only in-between calls of Update().
  public: std::vector<PhysicsWorldBaseInterface::Ptr> GetWorlds() const
  {
    std::lock_guard<std::recursive_mutex> lock(this->worldsMutex);
    std::vector<PhysicsWorldBaseInterface::Ptr> ret(this->worlds.begin(),
                                                    this->worlds.end());
    return ret;
  }

  /// Returns all worlds which could be casted to PhysicsWorldModelInterfaceT.
  /// Note that accessing the returned worlds asynchronously may lead to
  /// thread safety issues. It is recommended to access the returned
  /// worlds only in-between calls of Update().
  public: std::vector<PhysicsWorldModelInterfacePtr>
          GetModelPhysicsWorlds() const
  {
     std::vector<PhysicsWorldModelInterfacePtr> ret;
     std::lock_guard<std::recursive_mutex> lock(this->worldsMutex);
     int i = 0;
     for (std::vector<PhysicsWorldBaseInterface::Ptr>::const_iterator
          it = this->worlds.begin();
          it != this->worlds.end(); ++it, ++i)
     {
       PhysicsWorldModelInterfacePtr w = ToWorldWithModel(*it);
       if (!w)
       {
         std::cerr<<"Cannot cast world " << i << " to "
                  << "interface PhysicsWorldModelInterface<"
                  << GetTypeName<ModelID>()
                  << ", "<<GetTypeName<ModelPartID>()
                  << ", "<<GetTypeName<Vector3>()<<">" << std::endl;
       }
       ret.push_back(w);
     }
     return ret;
  }

  /// Returns all worlds which could be casted to PhysicsWorldContactInterfaceT.
  /// Note that accessing the returned worlds asynchronously may lead to
  /// thread safety issues. It is recommended to access the returned
  /// worlds only in-between calls of Update().
  public: std::vector<PhysicsWorldContactInterfacePtr>
          GetContactPhysicsWorlds() const
  {
     std::vector<PhysicsWorldContactInterfacePtr> ret;
     std::lock_guard<std::recursive_mutex> lock(this->worldsMutex);
     int i = 0;
     for (std::vector<PhysicsWorldBaseInterface::Ptr>::const_iterator
          it = this->worlds.begin();
          it != this->worlds.end(); ++it, ++i)
     {
       PhysicsWorldContactInterfacePtr
         w = ToWorldWithContact(*it);
       if (!w)
       {
         std::cerr<<"Cannot cast world " << i << " to "
                  << "interface PhysicsWorldContactInterface<"
                  << ", "<<GetTypeName<ModelID>()
                  << ", "<<GetTypeName<ModelPartID>()
                  << ", "<<GetTypeName<Vector3>()
                  << ", "<<GetTypeName<Wrench>()<<">" << std::endl;
       }
       ret.push_back(w);
     }
     return ret;
  }

  /// Returns all worlds which could be casted to PhysicsWorldT.
  /// Note that accessing the returned worlds asynchronously may lead to
  /// thread safety issues. It is recommended to access the returned
  /// worlds only in-between calls of Update().
  public: std::vector<PhysicsWorldPtr> GetPhysicsWorlds() const
  {
     std::vector<PhysicsWorldPtr> ret;
     std::lock_guard<std::recursive_mutex> lock(this->worldsMutex);
     int i = 0;
     for (std::vector<PhysicsWorldBaseInterface::Ptr>::const_iterator
          it = this->worlds.begin();
          it != this->worlds.end(); ++it, ++i)
     {
       PhysicsWorldPtr w = ToPhysicsWorld(*it);
       if (!w)
       {
         std::cerr<<"Cannot cast world " << i << " to "
                  << "interface PhysicsWorld<"
                  << GetTypeName<WorldState>()
                  << ", "<<GetTypeName<ModelID>()
                  << ", "<<GetTypeName<ModelPartID>()
                  << ", "<<GetTypeName<Vector3>()
                  << ", "<<GetTypeName<Wrench>()<<">" << std::endl;
       }
       ret.push_back(w);
     }
     return ret;
  }

  /// Calls PhysicsWorldModelInterface::AddModelFromFile
  /// on all worlds.
  /// \return the return value for each world. Typically,
  ///   the model name will be the same in all
  ///   worlds, as it is defined in the file or given in \e modelname.
  public: std::vector<ModelLoadResult>
                  AddModelFromFile(const std::string& filename,
                                   const std::string& modelname="")
  {
    return CallOnAllWorldsWithModel
      <ModelLoadResult, const std::string&, const std::string&>
        (&Self::AddModelFromFileCB, filename, modelname);
  }


  /// Calls PhysicsWorldModelInterface::AddModelFromString
  /// on all worlds.
  /// \return the return value for each world. Typically,
  ///   the model name will be the same in all
  ///   worlds, as it is defined in \e str or given in \e modelname.
  public: std::vector<ModelLoadResult>
                  AddModelFromString(const std::string& str,
                                     const std::string& modelname="")
  {
    return CallOnAllWorldsWithModel
      <ModelLoadResult, const std::string&, const std::string&>
        (&Self::AddModelFromStringCB, str, modelname);
  }


  /// Calls PhysicsWorldModelInterface::AddModelFromSDF
  /// on all worlds.
  /// \return the return value for each world. Typically,
  ///   the model name will be the same in all
  ///   worlds, as it is defined in \e sdf or given in \e modelname.
  public: std::vector<ModelLoadResult>
                  AddModelFromSDF(const sdf::ElementPtr& sdf,
                                  const std::string& modelname="")
  {
    return CallOnAllWorldsWithModel
      <ModelLoadResult, const sdf::ElementPtr&, const std::string&>
        (&Self::AddModelFromSdfCB, sdf, modelname);
  }


  /// Calls PhysicsWorldModelInterface::AddModelFromShape
  /// on all worlds which return true for SupportShapes().
  /// \return the return value for each world. Typically,
  ///   the model name will be the same in all
  ///   worlds, as it is defined in \e modelname.
  public: std::vector<ModelLoadResult>
          AddModelFromShape(const std::string& modelname,
                            const Shape::Ptr& shape,
                            const Shape::Ptr& collShape = Shape::Ptr())
  {
    return CallOnAllWorldsWithModel
      <ModelLoadResult, const std::string&,
       const Shape::Ptr&, const Shape::Ptr&>
        (&Self::AddModelFromShapeCB, modelname, shape, collShape);
  }

  /// Calls PhysicsWorldModelInterface::SetBasicModelState on
  /// all worlds. Assumes that all worlds use the same model name.
  /// \return number of worlds in which the state was successfully set.
  public: int SetBasicModelState(const ModelID& id,
                                 const BasicState& state)
  {
    std::vector<bool> ret = CallOnAllWorldsWithModel
      <bool, const ModelID&, const BasicState&>
        (&Self::SetBasicModelStateCB, id, state);
    int cnt = 0;
    for (std::vector<bool>::iterator it = ret.begin(); it != ret.end(); ++it)
    {
      if (*it) ++cnt;
    }
    return cnt;
  }


  // Convenience method which casts the world \e w to a
  // PhysicsWorldStateInterface with the given state
  public: static PhysicsWorldStateInterfacePtr
           ToWorldWithState(const PhysicsWorldBaseInterface::Ptr& w)
  {
   return std::dynamic_pointer_cast<PhysicsWorldStateInterfaceT>(w);
  }

  // Convenience method which casts the world \e w to a
  // PhysicsWorldModelInterface
  public: static PhysicsWorldModelInterfacePtr
          ToWorldWithModel(const PhysicsWorldBaseInterface::Ptr& w)
  {
   return std::dynamic_pointer_cast <PhysicsWorldModelInterfaceT>(w);
  }

  // Convenience method which casts the world \e w to a
  // PhysicsWorldContactInterface
  public: static PhysicsWorldContactInterfacePtr
          ToWorldWithContact(const PhysicsWorldBaseInterface::Ptr& w)
  {
   return std::dynamic_pointer_cast<PhysicsWorldContactInterfaceT>(w);
  }

   // Convenience method which casts the world \e w to a PhysicsWorld
  public: static PhysicsWorldPtr
          ToPhysicsWorld(const PhysicsWorldBaseInterface::Ptr& w)
  {
   return std::dynamic_pointer_cast<PhysicsWorldT>(w);
  }

  /*  public: template<class WorldState_, class ModelID_, class ModelPartID_,
                   class Vector3_, class Wrench_>
          static typename PhysicsWorld<WorldState, ModelID_, ModelPartID_,
                                       Vector3_, Wrench_>::Ptr
          ToPhysicsWorld(const PhysicsWorldBaseInterface::Ptr& w)
  {
   return std::dynamic_pointer_cast
          <PhysicsWorld<WorldState_, ModelID_, ModelPartID_,
                         Vector3_, Wrench_>>(w);
  }*/

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
  public: void SetDynamicsEnabled(const bool flag)
  {
   std::cout << "WorldManager received request to set dynamics "
             << "enable to " << flag << std::endl;
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

  public: ControlServerPtr GetControlServer()
  {
   return controlServer;
  }

  // Saves all worlds to files. The filenames will be written
  // into the \e directory and the filename will be composed as
  // follows: \e prefix + PhysicsWorldBaseInterface::GetName() + "." + \e ext.
  // \param[in] directory the directory to write in. If empty, the local
  //    directory is used.
  // \return number of failures
  public: int SaveAllWorlds(const std::string& directory = "",
                             const std::string& prefix = "",
                             const std::string& ext = "world")
  {
    int fail = 0;
    std::lock_guard<std::recursive_mutex> lock(this->worldsMutex);
    for (std::vector<PhysicsWorldBaseInterface::Ptr>::iterator
         it = this->worlds.begin();
         it != this->worlds.end(); ++it)
    {
      PhysicsWorldBaseInterface::Ptr w=*it;
      std::string filename;
      if (!directory.empty()) filename += directory + "/";
      filename += prefix + w->GetName() + "." + ext;
      std::cout << "Writing to file " << filename << std::endl;
      if (!w->SaveToFile(filename))
      {
        std::cerr << "ERROR: Could not save world " << w->GetName() <<
                     " to file " << filename << std::endl;
        ++fail;
      }
    }
    return fail;
  }

  private: void NotifyPause(const bool _flag)
  {
    std::cout << "WorldManager Received PAUSE command: "
              << _flag << std::endl;
    SetPaused(_flag);
  }

  private: void NotifyUpdate(const int _numSteps)
  {
    std::cout << "WorldManager Received UPDATE command with "
              << _numSteps << " steps. " << std::endl;
    Update(_numSteps, true);
  }

  private: void NotifyModelStateChange(const ModelID  &_id,
                                   const BasicState &_state)
  {
     std::cout << "WorldManager received STATE CHANGE command "
               << "for model " << _id << ": " << _state << std::endl;
     // std::vector<bool> retVals =
       CallOnAllWorldsWithModel <bool, const ModelID&, const BasicState&>
        (&Self::SetBasicModelStateCB, _id, _state);
  }


  private: void NotifySdfModelLoad(const std::string& _sdf,
                                  const bool _isString,
                                  const BasicState &_state)
  {
     std::cout << "WorldManager received SDF MODEL command"
               << std::endl;
     std::lock_guard<std::recursive_mutex> lock(this->worldsMutex);
     for (std::vector<PhysicsWorldBaseInterface::Ptr>::iterator
          it = this->worlds.begin();
          it != this->worlds.end(); ++it)
     {
       PhysicsWorldModelInterfacePtr
         w =ToWorldWithModel(*it);
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

  // changing gravity is not supported yet, but if it is,
  // it can be implemented here at some point
  /*public: void SetGravity(const float x, const float y, const float z)
   {
     std::lock_guard<std::recursive_mutex> lock(this->worldsMutex);
     for (std::vector<PhysicsWorldBaseInterface::Ptr>::iterator
          it = this->worlds.begin();
          it != this->worlds.end(); ++it)
     {
         PhysicsWorldBaseInterface::Ptr w=*it;
         ...
     }
   }*/

  /**
   * Changes the mirror world to either the previous one or the last one.
   * Returns the name of the world currently set.
   * If no world is being mirrored by the mirror world, or there are
   * no worlds at all, the empty
   * string is returned and an error printed.
   */
  private: std::string ChangeMirrorWorld(const int ctrl)
  {
     std::lock_guard<std::recursive_mutex> lock(this->worldsMutex);
       if (worlds.empty())
       {
         std::cerr<<"There are no worlds to be mirrored." << std::endl;
         return "";
       }

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
     {
       if (!mirrorWorld->GetOriginalWorld())
       {
         std::cerr<<"Mirror world has no original world set, "
                  <<"cannot return name." << std::endl;
         return "";
       }
       return mirrorWorld->GetOriginalWorld()->GetName();
     }

     // update mirrored world
     if (this->SetMirroredWorld(mirroredWorldIdx))
     {
       std::cout << "WorldManager: New world is "
                 << mirrorWorld->GetOriginalWorld()->GetName()
                 << std::endl;
     }

     if (!mirrorWorld->GetOriginalWorld())
     {
       std::cerr<<"Mirror world has no original world set, "
                <<"cannot return name." << std::endl;
       return "";
     }

     // return the name of the new world
     return mirrorWorld->GetOriginalWorld()->GetName();
   }


  // Helper callback to call AddModelFromFile on the world
  private: static ModelLoadResult
                  AddModelFromFileCB(PhysicsWorldModelInterfaceT& w,
                                     const std::string& filename,
                                     const std::string& modelname="")
  {
    return w.AddModelFromFile(filename, modelname);
  }


  // Helper callback to call AddModelFromString on the world
  private: static ModelLoadResult
                  AddModelFromStringCB(PhysicsWorldModelInterfaceT& w,
                                       const std::string& str,
                                       const std::string& modelname="")
  {
    return w.AddModelFromString(str, modelname);
  }


  // Helper callback to call AddModelFromSDF on the world
  private: static ModelLoadResult
                  AddModelFromSdfCB(PhysicsWorldModelInterfaceT& w,
                                    const sdf::ElementPtr& sdf,
                                    const std::string& modelname="")
  {
    return w.AddModelFromSDF(sdf, modelname);
  }



  // Helper callback to call AddModelFromShape on the world
  private: static ModelLoadResult
                  AddModelFromShapeCB(PhysicsWorldModelInterfaceT& w,
                                      const std::string& modelname,
                                      const Shape::Ptr& shape,
                                      const Shape::Ptr& collShape)
  {
    return w.AddModelFromShape(modelname, shape, collShape);
  }

  // Helper callback to call SetBasicModelState on the world
  private: static bool SetBasicModelStateCB
              (PhysicsWorldModelInterfaceT& w,
               const ModelID&id,
               const BasicState& state)
  {
    return w.SetBasicModelState(id, state);
  }

  // Helper function which calls a callback function on each of the worlds
  // after casting it to PhysicsWorldModelInterfaceT. Accumulates all return
  // values in a vector and returns it.
  private: template<typename RetVal, typename ... Params>
  std::vector<RetVal> CallOnAllWorldsWithModel
      (RetVal(*callback)(PhysicsWorldModelInterfaceT&, Params...),
       Params... params)
  {
     std::vector<RetVal> ret;
     std::lock_guard<std::recursive_mutex> lock(this->worldsMutex);
     for (std::vector<PhysicsWorldBaseInterface::Ptr>::iterator
          it = this->worlds.begin();
          it != this->worlds.end(); ++it)
     {
       PhysicsWorldModelInterfacePtr w = ToWorldWithModel(*it);
       if (!w)
       {
         THROW_EXCEPTION("Only support worlds which have the "
                         << "interface PhysicsWorldModelInterface<"
                         << GetTypeName<ModelID>()
                         << ", "<<GetTypeName<ModelPartID>()<<">");
       }
       RetVal r = callback(*w, std::forward<Params>(params)...);
       ret.push_back(r);
     }
     return ret;
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
