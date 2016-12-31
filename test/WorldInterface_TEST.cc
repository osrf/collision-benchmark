#include <collision_benchmark/GazeboWorldLoader.hh>
#include <collision_benchmark/PhysicsWorld.hh>
#include <collision_benchmark/GazeboPhysicsWorld.hh>
#include <collision_benchmark/GazeboStateCompare.hh>
#include <collision_benchmark/GazeboMirrorWorld.hh>
#include <collision_benchmark/GazeboHelpers.hh>
#include <collision_benchmark/WorldManager.hh>
#include <collision_benchmark/boost_std_conversion.hh>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include "BasicTestFramework.hh"

using collision_benchmark::PhysicsWorldBase;
using collision_benchmark::PhysicsWorld;
using collision_benchmark::GazeboPhysicsWorld;
using collision_benchmark::GazeboMirrorWorld;
using collision_benchmark::GazeboStateCompare;
using collision_benchmark::WorldManager;


class WorldInterfaceTest : public BasicTestFramework {};

TEST_F(WorldInterfaceTest, TransferWorldState)
{
  typedef collision_benchmark::PhysicsWorldBase<gazebo::physics::WorldState> GzPhysicsWorldBase;

  GazeboPhysicsWorld::Ptr gzWorld1(new GazeboPhysicsWorld());
  ASSERT_EQ(gzWorld1->LoadFromFile("worlds/empty.world"),GzPhysicsWorldBase::SUCCESS) << " Could not load empty world";
  gzWorld1->GetWorld()->SetPhysicsEnabled(false);

  GazeboPhysicsWorld::Ptr gzWorld2(new GazeboPhysicsWorld());
  ASSERT_EQ(gzWorld2->LoadFromFile("worlds/rubble.world"),GzPhysicsWorldBase::SUCCESS) << " Could not load rubble world";

  GzPhysicsWorldBase::Ptr world1(gzWorld1);
  GzPhysicsWorldBase::Ptr world2(gzWorld2);

  int numIters=1000;
  std::cout<<"Doing "<<numIters<<" iterations on the rubble world and set the second world to the same state"<<std::endl;
  for (int i=0; i<numIters; ++i)
  {
    gazebo::physics::WorldState target=world2->GetWorldState(); // get the rubble world
    ASSERT_EQ(world1->SetWorldState(target, false), GzPhysicsWorldBase::SUCCESS) << " Could not set world state";
    gazebo::physics::WorldState newState = world1->GetWorldState();
    GazeboStateCompare::Tolerances t=GazeboStateCompare::Tolerances::CreateDefault(1e-03);
    t.CheckDynamics=false; // don't check dynamics because we disable physics engine in gzWorld1
    ASSERT_EQ(GazeboStateCompare::Equal(newState, target, t), true)
      <<"Target state was not set as supposed to!! ";
/*      <<std::endl<<std::endl<<newState<<std::endl<<std::endl<<target
      <<std::endl<<std::endl
      <<"Diff: "<<std::endl
      <<target - world1->GetWorldState();*/
    gzWorld1->Update(1);
    gzWorld2->Update(1);
  }
}

TEST_F(WorldInterfaceTest, WorldManager)
{
  std::map<std::string,std::string> physicsEngines = collision_benchmark::getPhysicsSettingsSdfForAllEngines();
  std::string worldfile = "../test_worlds/cube.world";

  std::cout << "Loading world " << worldfile << " with "<<physicsEngines.size()<<" engines."<<std::endl;

  typedef gazebo::physics::WorldState GzWorldState;
  typedef WorldManager<GzWorldState> GzWorldManager;
  typedef GzWorldManager::PhysicsWorldBasePtr PhysicsWorldBasePtr;

  // create one world per physics engine and load it with the cube world, and add it to the world manager
  GzWorldManager worldManager;
  int i=1;
  for (std::map<std::string,std::string>::iterator it = physicsEngines.begin(); it!=physicsEngines.end(); ++it, ++i)
  {
    std::string engine = it->first;
    std::string physicsSDF = it->second;

    std::stringstream _worldname;
    _worldname << "world_" << i << "_" << engine;
    std::string worldname=_worldname.str();

    std::cout << "Loading with physics engine " << engine << " (named as '" << worldname << "')" << std::endl;

    std::cout<<"Loading physics from "<<physicsSDF<<std::endl;
    sdf::ElementPtr physics = collision_benchmark::GetPhysicsFromSDF(physicsSDF);
    ASSERT_NE(physics.get(), nullptr) << "Could not get phyiscs engine from " << physicsSDF << std::endl;

    // std::cout<<"Physics: "<<physics->ToString("")<<std::endl;
    std::cout<<"Loading world from "<<worldfile<<std::endl;
    gazebo::physics::WorldPtr gzworld = collision_benchmark::LoadWorldFromFile(worldfile, worldname, physics);

    ASSERT_NE(gzworld.get(), nullptr) <<"Error loading world "<<worldfile<<std::endl;

    // Create the GazeboPhysicsWorld object
    GazeboPhysicsWorld::Ptr gzPhysicsWorld(new GazeboPhysicsWorld());
    gzPhysicsWorld->SetWorld(collision_benchmark::to_std_ptr<gazebo::physics::World>(gzworld));
    worldManager.AddPhysicsWorld(gzPhysicsWorld);
  }

  std::vector<PhysicsWorldBasePtr> allWorlds = worldManager.GetPhysicsWorlds();
  ASSERT_EQ(allWorlds.size(), physicsEngines.size()) << "As many worlds as physics engines should have been loaded";

  // All worlds should have a cube named "box", and only two models (ground and cube).
  for (std::vector<PhysicsWorldBasePtr>::iterator wit=allWorlds.begin(); wit!=allWorlds.end(); ++wit)
  {
    PhysicsWorldBasePtr world=*wit;
    GzWorldState state = world->GetWorldState();
    ASSERT_EQ(state.GetModelStates().size(), 2) <<"World "<<world->GetName()<<" should have only two models.";
    ASSERT_NE(state.HasModelState("box"), false) << "World "<<world->GetName()<<" has no model named 'box'";
  }
}


TEST_F(WorldInterfaceTest, ModelLoading)
{
  std::string worldfile = "../test_worlds/cube.world";

  std::cout << "Loading world " << worldfile << std::endl;

  typedef gazebo::physics::WorldState GzWorldState;
  typedef WorldManager<GzWorldState> GzWorldManager;
  typedef GzWorldManager::PhysicsWorldBasePtr PhysicsWorldBasePtr;
  typedef PhysicsWorld<collision_benchmark::GazeboPhysicsWorldTypes> GzPhysicsWorld;

  // create one world per physics engine and load it with the cube world, and add it to the world manager
  GzPhysicsWorld::Ptr world (new GazeboPhysicsWorld());
  world->LoadFromFile(worldfile);

  ASSERT_NE(world.get(), nullptr) <<"Error loading world "<<worldfile<<std::endl;

  // by now, the world should only have two models: the cube and the ground
  GzWorldState state = world->GetWorldState();
  ASSERT_EQ(state.GetModelStates().size(), 2) <<"World "<<world->GetName()<<" should have only two models.";

  // Now load an extra sphere to the worlds to test the PhysicsWorld::AddModel* functions
  std::string forcedModelName = "test-sphere";
  GzPhysicsWorld::ModelLoadResult res = world->AddModelFromFile("../test_worlds/sphere.sdf",forcedModelName);
  ASSERT_EQ(res.opResult, GzPhysicsWorld::SUCCESS) << " Could not add extra model to world";
  ASSERT_EQ(res.modelID, forcedModelName) << " Model name must have been forced to "<<forcedModelName<<" but is "<<res.modelID;
  state = world->GetWorldState();
  ASSERT_EQ(state.GetModelStates().size(), 3) <<"World "<<world->GetName()<<" should have 3 models.";

  // Add another model via the String load function
  std::string sdfStr("\
    <model name='box'>\
      <pose>0 0 2.0 0 0 0</pose>\
      <link name='link'>\
        <collision name='cube_collision'>\
          <geometry>\
            <box>\
              <size>0.5 0.5 0.5</size>\
            </box>\
          </geometry>\
        </collision>\
        <visual name='cube_visual'>\
          <geometry>\
            <box>\
              <size>0.5 0.5 0.5</size>\
            </box>\
          </geometry>\
        </visual>\
      </link>\
    </model>");
  std::string forcedModelName2 = "test-cube";
  GzPhysicsWorld::ModelLoadResult res2 = world->AddModelFromString(sdfStr,forcedModelName2);
  ASSERT_EQ(res2.opResult, GzPhysicsWorld::SUCCESS) << " Could not add extra model to world";
  ASSERT_EQ(res2.modelID, forcedModelName2) << " Model name must have been forced to "<<forcedModelName2<<" but is "<<res2.modelID;
  state = world->GetWorldState();
  ASSERT_EQ(state.GetModelStates().size(), 4) <<"World "<<world->GetName()<<" should have 4 models.";

  /*std::cout<<"Now you can view it with gzclient. Press any key to start the world."<<std::endl;
  getchar();
  while(true)
  {
    int numSteps=1;
    world->Update(numSteps);
  }*/
}


int main(int argc, char**argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
