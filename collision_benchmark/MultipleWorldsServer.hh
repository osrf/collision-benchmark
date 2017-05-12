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
 * \brief Server which can be used to run one or more world with
 * multiple physics engines.
 *
 * Several worlds can be running in parallel.
 * This interface offers methods to make the use of multiple worlds
 * easier, including the maintenance of a WorldManager and providing methods
 * for loading, starting and stopping worlds.
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

  // \param _worldLoaders world loaders for all the physics engines.
  // \param _universalLoader a loader which will automatically choose the
  //    engine to load from the world file given upon loading. nullptr if
  //    no such loader specified.
  public: MultipleWorldsServer(const WorldLoader_M& _worldLoaders,
                               const WorldLoader::ConstPtr& _universalLoader =
                                     nullptr):
          worldLoaders(_worldLoaders),
          universalLoader(_universalLoader) {}
  public: virtual ~MultipleWorldsServer() { Fini(); }

  // Start the server. Starting of the server may accept
  // command line parameters depending on the implementation.
  // \return success of starting the server
  public: virtual bool Start(int argc=0, const char** argv=NULL) = 0;
  public: virtual void Stop() = 0;
  public: virtual bool isRunning() const = 0;

  // Initializes the server. Should be called before any Load()
  // functions and will create the world manager.
  // \param mirror_name the name of the mirror world, or empty to disable
  //        creating a mirror world.
  // \param allowMirrorControl if true, the mirror world will be allowed to
  //        serve as control world to control all underlying worlds, while
  //        it is mirroring one at a time.
  public: void Init(const std::string& mirror_name = "mirror",
                   const bool allowMirrorControl = false)
  {
    worldManager = createWorldManager(mirror_name, allowMirrorControl);
    assert(worldManager);
  }

  public: void Fini()
  {
    worldManager.reset();
  }

  // \brief Loads the world file with the different engines.
  // Generates a world name based on the prefix \e namePrefix.
  // This will create several worlds (one for each engine) which are
  // added to the WorldManager.
  // \param worldfile the filename the filename
  // \param engines the physics engines (identified by name) to use.
  // \param namePrefix The name of the world will be generated
  //  using this prefix to the name. This is required because multiple
  //  worlds loaded from the same world file cannot have the same name.
  // \return number of engines which were successfully loaded
  public: int Load(const std::string& worldfile,
                   const std::vector<std::string>& engines,
                   const std::string& namePrefix = "world")
  {
    assert(worldManager);
    assert(!namePrefix.empty());

    int i = 1;
    for (std::vector<std::string>::const_iterator
         it = engines.begin(); it != engines.end(); ++it, ++i)
    {
      std::string engine = *it;
      std::stringstream _worldname;
      _worldname << namePrefix << "_engine_" << i << "_" << engine;
      std::string worldname=_worldname.str();
      if (Load(worldfile, engine, worldname) < 0)
      {
        std::cerr << "Could not load world " << worldfile << " with engine "
                  << engine << ", skipping it." << std::endl;
        continue;
      }
    }
    return worldManager->GetNumWorlds();
  }

  // \brief Loads the world file with the given engine.
  // This will create one new world using this engine which will be
  // added to the WorldManager.
  // \param worldfile the filename the filename
  // \param engine the physics engine (identified by name) to use.
  // \param worldname name to use for the world. If empty, will use the
  //    name specified in the file.
  // \retval >= 0 on success, the index at which this world can be accessed in
  //    the world manager.
  // \retval -1 no world loader exists for this engine
  // \retval -2 world with this engine name cannot be loaded
  // \retval -3 world with this name already exists
  public: int Load(const std::string& worldfile,
                   const std::string& engine,
                   const std::string& worldname = "")
  {
    assert(worldManager);
    WorldLoader_M::iterator wlIt = worldLoaders.find(engine);
    if (wlIt == worldLoaders.end())
    {
      return -1;
    }
    WorldLoader::ConstPtr loader = wlIt->second;
    assert(loader);

    std::cout << "Loading with physics engine " << engine
              << " (named as '" << worldname << "')" << std::endl;

    PhysicsWorldBaseInterface::Ptr world =
      loader->LoadFromFile(worldfile, worldname);

    if (!world) return -2;

    int ret = worldManager->AddPhysicsWorld(world);
    if (ret < 0) return -3;
    return ret;
  }

  // \brief Loads the world file and determines the engine to use from the file.
  // This will create one new worlds using the engine determined
  // from the file. The world will be added to the WorldManager.
  // \param worldfile the filename the filename
  // \param worldname name to use for the world. If empty, will use the
  //    name specified in the file.
  // \retval >0 success, and index this world can be accessed at in the
  //    world manager.
  // \retval -1 f there is no world loader which can determine the
  //    engine from the file
  // \retval -2 if the loader failed to the world
  // \retval -3 world with this name already exists
  public: int AutoLoad(const std::string& worldfile,
                   const std::string& worldname = "")
  {
    assert(worldManager);
    if (!universalLoader) return -1;
    std::cout << "Auto-loading world (named as '"
              << worldname << "')" << std::endl;

    PhysicsWorldBaseInterface::Ptr world =
      universalLoader->LoadFromFile(worldfile, worldname);

    if (!world) return -2;

    int ret = worldManager->AddPhysicsWorld(world);
    if (ret < 0) return -3;
    return ret;
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

  // world loaders for all the physics engines.
  protected: WorldLoader_M worldLoaders;

  // A loader which will automatically choose the
  // engine to load from the world file given upon loading.
  protected: WorldLoader::ConstPtr universalLoader;

  protected: WorldManagerPtr worldManager;
};  // class MultpleWorldsServer

}  // namespace

#endif  // COLLISION_BENCHMARK_MULTIPLEWORLDSSERVER_H
