#include <test/StaticTestFramework.hh>
#include <collision_benchmark/PrimitiveShape.hh>
#include <collision_benchmark/SimpleTriMeshShape.hh>
#include <collision_benchmark/BasicTypes.hh>
#include <gazebo/gazebo.hh>

#include "MultipleWorldsTestFramework.hh"

using collision_benchmark::Shape;
using collision_benchmark::PrimitiveShape;
using collision_benchmark::SimpleTriMeshShape;
using collision_benchmark::BasicState;
using collision_benchmark::Vector3;
using collision_benchmark::Quaternion;


void StaticTestFramework::TwoShapes(const Shape::Ptr& shape1,
                                    const std::string& modelName1,
                                    const Shape::Ptr& shape2,
                                    const std::string& modelName2,
                                    const std::vector<std::string>& engines)
{
  // world to load
  std::string worldfile = "worlds/empty.world";

  GzMultipleWorldsServer::Ptr mServer = GetServer();
  ASSERT_NE(mServer.get(), nullptr) << "Could not create and start server";

  bool loadMirror = true;
  std::string mirrorName = "";
  if (loadMirror) mirrorName = "mirror";
  // with the tests, the mirror can be used to watch the test,
  // but not to manipulate the worlds.
  bool allowControlViaMirror = false;
  int numWorlds = mServer->Load(worldfile, engines,
                               mirrorName, allowControlViaMirror);

  GzWorldManager::Ptr worldManager = mServer->GetWorldManager();
  assert(worldManager);

  ASSERT_EQ(numWorlds, worldManager->GetNumWorlds())
    << "Inconsistency: Server returned different number of worlds";

/*  GzWorldManager::ControlServerPtr controlServer =
    worldManager->GetControlServer();

  if (controlServer)
  {
    controlServer->RegisterPauseCallback(std::bind(pauseCallback,
                                                   std::placeholders::_1));
  }*/

  // Load model 1
  typedef GzWorldManager::ModelLoadResult ModelLoadResult;
  std::vector<ModelLoadResult> res1
    = worldManager->AddModelFromShape(modelName1, shape1, shape1);
  ASSERT_EQ(res1.size(), numWorlds)
    << "Model must have been loaded in all worlds";
  for (std::vector<ModelLoadResult>::iterator it = res1.begin();
       it != res1.end(); ++it)
  {
    const ModelLoadResult& mlRes=*it;
    ASSERT_EQ(mlRes.opResult, collision_benchmark::SUCCESS)
      << "Could not load model";
    ASSERT_EQ(mlRes.modelID, modelName1)
      << "Model names should be equal";
  }

  // Load model 2
  std::vector<ModelLoadResult> res2
    = worldManager->AddModelFromShape(modelName2, shape2, shape2);
  ASSERT_EQ(res2.size(), numWorlds)
    << "Model must have been loaded in all worlds";
  for (std::vector<ModelLoadResult>::iterator it = res2.begin();
       it != res2.end(); ++it)
  {
    const ModelLoadResult& mlRes=*it;
    ASSERT_EQ(mlRes.opResult, collision_benchmark::SUCCESS)
      << "Could not load model";
    ASSERT_EQ(mlRes.modelID, modelName2)
      << "Model names should be equal";
  }

  worldManager->SetDynamicsEnabled(false);
  worldManager->SetPaused(true);

  std::cout << "Now start gzclient if you would like "
            << "to view the test. "<<std::endl;
  std::cout << "Press [Enter] to continue without gzclient or hit "
            << "the play button in gzclient."<<std::endl;
  getchar();

  worldManager->SetPaused(false);

  // set models to their initial pose
  BasicState bstate1, bstate2;
  bstate1.SetPosition(Vector3(1,0,0));
  bstate2.SetPosition(Vector3(-1,0,0));
  int cnt1 = worldManager->SetBasicModelState(modelName1, bstate1);
  int cnt2 = worldManager->SetBasicModelState(modelName2, bstate2);
  ASSERT_EQ(cnt1, numWorlds) << "All worlds should have been updated";
  ASSERT_EQ(cnt2, numWorlds) << "All worlds should have been updated";

  // start the update loop
  std::cout << "Now starting to update worlds."<<std::endl;
  int msSleep = 1000;  // delay for running the test
  while(true)
  {
    std::cout<<"UPDATE"<<std::endl;
    int numSteps=1;
    worldManager->Update(numSteps);
    if (msSleep > 0) gazebo::common::Time::MSleep(msSleep);

    bstate1.position.x-=0.1;
    bstate2.position.x+=0.1;
    cnt1 = worldManager->SetBasicModelState(modelName1, bstate1);
    cnt2 = worldManager->SetBasicModelState(modelName2, bstate2);
    ASSERT_EQ(cnt1, numWorlds) << "All worlds should have been updated";
    ASSERT_EQ(cnt2, numWorlds) << "All worlds should have been updated";
  }
}
