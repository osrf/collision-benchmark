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
#ifndef COLLISION_BENCHMARK_MULTIPLEWORLDSSERVER_H
#define COLLISION_BENCHMARK_MULTIPLEWORLDSSERVER_H

#include <collision_benchmark/MirrorWorld.hh>

#include <collision_benchmark/PhysicsWorld.hh>
#include <collision_benchmark/WorldManager.hh>
#include <collision_benchmark/WorldLoader.hh>
#include <collision_benchmark/ControlServer.hh>

#include <memory>

namespace collision_benchmark
{

/**
 * \brief Server which can be used to run a world with multiple physics engines.
 *
 * This results in several worlds running in parallel.
 *
 * \author Jennifer Buehler
 * \date March 2017
 */
template<class _WorldState, class _ModelID,
        class _ModelPartID, class _Vector3, class _Wrench>
class MultipleWorldsServer
{
  public: typedef
          MultipleWorldsServer<_WorldState, _ModelID,
                               _ModelPartID, _Vector3, _Wrench> Self;
  public: typedef std::shared_ptr<Self> Ptr;
  public: typedef std::shared_ptr<const Self> ConstPtr;

  public: typedef _WorldState WorldState;
  public: typedef _ModelID ModelID;
  public: typedef _ModelPartID ModelPartID;
  public: typedef _Vector3 Vector3;
  public: typedef _Wrench Wrench;

  public: typedef WorldManager<WorldState, ModelID,
                               ModelPartID, Vector3, Wrench> WorldManagerT;
  public: typedef typename WorldManagerT::Ptr WorldManagerPtr;

  // for each physics engine identified by name,
  // the world loader assigned to it.
  public: typedef std::map<std::string, WorldLoader::ConstPtr> WorldLoader_M;

  // \param _worldLoaders world loaders for all physics engines.
  public: MultipleWorldsServer(const WorldLoader_M& _worldLoaders):
          worldLoaders(_worldLoaders) {}
  public: virtual ~MultipleWorldsServer() {}

  // Start the server. Starting of the server may accept
  // command line parameters depending on the implementation.
  // \return success of starting the server
  public: virtual bool Start(int argc=0, const char** argv=NULL) = 0;
  public: virtual void Stop() = 0;


  // Loads the world file with the different engines.
  // \param worldfile the filename the filename
  // \param engines the physics engines (identified by name) to use.
  // \param mirror_name the name of the mirror world, or empty to disable
  //        creating a mirror world.
  // \param allowMirrorControl if true, the mirror world will be allowed to
  //        serve as control world to control all underlying worlds, while`
  //        it is mirroring one at a time.
  // \return number of engines which were successfully loaded
  public: int Load(const std::string& worldfile,
                   const std::vector<std::string>& engines,
                   const std::string& mirror_name = "mirror",
                   const bool allowMirrorControl = false)
  {
    worldManager = createWorldManager(mirror_name, allowMirrorControl);
    assert(worldManager);

    int i = 1;
    for (std::vector<std::string>::const_iterator
         it = engines.begin(); it != engines.end(); ++it, ++i)
    {
      std::string engine = *it;

      WorldLoader_M::iterator wlIt = worldLoaders.find(engine);
      if (wlIt == worldLoaders.end())
      {
        std::cerr << "No world loader provided for engine " << engine
                  << ", skipping it." << std::endl;
        continue;
      }
      WorldLoader::ConstPtr loader = wlIt->second;
      assert(loader);

      std::stringstream _worldname;
      _worldname << "world_" << i << "_" << engine;
      std::string worldname=_worldname.str();

      std::cout << "Loading with physics engine " << engine
                << " (named as '" << worldname << "')" << std::endl;

      PhysicsWorldBaseInterface::Ptr world =
        loader->LoadFromFile(worldfile, worldname);

      worldManager->AddPhysicsWorld(world);
    }
    return worldManager->GetNumWorlds();
  }

  WorldManagerPtr GetWorldManager() { return worldManager; }

  // creates the world manager.
  // \param mirror_name the name of the mirror world, or empty to disable
  //        creating a mirror world.
  // \param allowMirrorControl if true, the mirror world will be allowed to
  //        serve as control world to control all underlying worlds, while
  //        it is mirroring one at a time.
  protected: virtual WorldManagerPtr
             createWorldManager(const std::string& mirror_name = "",
                                const bool allowMirrorControl = false) = 0;

  // world loaders for all physics engines.
  protected: WorldLoader_M worldLoaders;

  protected: WorldManagerPtr worldManager;
};  // class MultpleWorldsServer

}  // namespace

#endif  // COLLISION_BENCHMARK_MULTIPLEWORLDSSERVER_H
