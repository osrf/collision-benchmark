#include <collision_benchmark/GazeboMultipleWorldsServer.hh>
#include <collision_benchmark/GazeboWorldLoader.hh>
#include <collision_benchmark/GazeboHelpers.hh>

#include <gtest/gtest.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

using collision_benchmark::MultipleWorldsServer;
using collision_benchmark::GazeboMultipleWorldsServer;
using collision_benchmark::WorldLoader;
using collision_benchmark::WorldManager;
using collision_benchmark::GazeboWorldLoader;
using collision_benchmark::GazeboPhysicsWorldTypes;

typedef MultipleWorldsServer<GazeboPhysicsWorldTypes::WorldState,
                             GazeboPhysicsWorldTypes::ModelID,
                             GazeboPhysicsWorldTypes::ModelPartID,
                             GazeboPhysicsWorldTypes::Vector3>
                                GzMultipleWorldsServer;

typedef WorldManager<GazeboPhysicsWorldTypes::WorldState,
                     GazeboPhysicsWorldTypes::ModelID,
                     GazeboPhysicsWorldTypes::ModelPartID,
                     GazeboPhysicsWorldTypes::Vector3>
          GzWorldManager;


class MultipleWorldsTestFramework : public ::testing::Test {
  protected:

  MultipleWorldsTestFramework()
  :fakeProgramName("MultipleWorldsTestFramework")
  {
  }
  virtual ~MultipleWorldsTestFramework()
  {
  }

  virtual void SetUp()
  {
    bool enforceContactCalc=true;
    std::set<std::string> engines =
      collision_benchmark::GetSupportedPhysicsEngines();
    GzMultipleWorldsServer::WorldLoader_M loaders;
    for (std::set<std::string>::const_iterator
         it = engines.begin(); it != engines.end(); ++it)
    {
      std::string engine = *it;
      try
      {
        loaders[engine] =
          WorldLoader::ConstPtr(new GazeboWorldLoader(engine,
                                                      enforceContactCalc));
      }
      catch (collision_benchmark::Exception& e)
      {
        std::cerr << "Could not add support for engine "
                  <<engine << ": " << e.what() << std::endl;
        continue;
      }
    }

    if (loaders.empty())
    {
      std::cerr << "Could not get support for any engine." << std::endl;
      return;
    }

    server.reset(new GazeboMultipleWorldsServer(loaders));
    server->Start(1, &fakeProgramName);
  }

  virtual void TearDown()
  {
    if (server) server->Stop();
  }

  public: GzMultipleWorldsServer::Ptr GetServer() { return server; }

  private:
  const char * fakeProgramName;
  GzMultipleWorldsServer::Ptr server;
};
