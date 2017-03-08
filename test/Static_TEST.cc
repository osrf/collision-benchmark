#include <collision_benchmark/WorldManager.hh>
#include <gazebo/gazebo.hh>

#include "MultipleWorldsTestFramework.hh"


class StaticTest : public MultipleWorldsTestFramework {};

// Loads a world with different physics engines (only the ones implemented
// in Gazebo) and does the static test in which two models are generated at
// various poses and all engines have to agree on the
// collision state (boolean collision).
TEST_F(StaticTest, TwoSpheres)
{
  std::vector<std::string> selectedEngines;
  selectedEngines.push_back("bullet");
  selectedEngines.push_back("ode");

  /*std::set<std::string> engines =
    collision_benchmark::GetSupportedPhysicsEngines();
  // run test on all engines
  selectedEngines.insert(selectedEngines.end(), engines.begin(), engines.end());*/

  // test world
  std::string worldfile = "worlds/rubble.world";

  GzMultipleWorldsServer::Ptr mServer = GetServer();
  ASSERT_NE(mServer.get(), nullptr) << "Could not create and start server";

  bool loadMirror = true;
  std::string mirrorName = "";
  if (loadMirror) mirrorName = "mirror";
  // with the tests, the mirror can be used to watch the test,
  // but not to manipulate the worlds.
  bool allowControlViaMirror = false;
  int numWorlds = mServer->Load(worldfile, selectedEngines,
                               mirrorName, allowControlViaMirror);

  GzWorldManager::Ptr worldManager = mServer->GetWorldManager();
  assert(worldManager);

/*  GzWorldManager::ControlServerPtr controlServer =
    worldManager->GetControlServer();

  if (controlServer)
  {
    controlServer->RegisterPauseCallback(std::bind(pauseCallback,
                                                   std::placeholders::_1));
  }*/

  worldManager->SetDynamicsEnabled(true);
  worldManager->SetPaused(true);

  std::cout << "Now start gzclient if you would like "
            << "to view the test. "<<std::endl;
  std::cout << "Press [Enter] to continue without gzclient or hit "
            << "the play button in gzclient."<<std::endl;
  getchar();

  worldManager->SetPaused(false);

  std::cout << "Now starting to update worlds."<<std::endl;
  while(true)
  {
    int numSteps=1;
    worldManager->Update(numSteps);
  }
}


int main(int argc, char**argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
