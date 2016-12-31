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

  for (int i=0; i<2000; ++i)
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

  typedef WorldManager<gazebo::physics::WorldState> GzWorldManager;

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


}



int main(int argc, char**argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
