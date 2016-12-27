#include <gtest/gtest.h>

#include <collision_benchmark/WorldLoader.hh>
#include <collision_benchmark/PhysicsWorld.hh>
#include <collision_benchmark/GazeboPhysicsWorld.hh>
#include <collision_benchmark/GazeboStateCompare.hh>
#include <collision_benchmark/GazeboMirrorWorld.hh>
#include <collision_benchmark/GazeboHelpers.hh>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <gazebo/test/helper_physics_generator.hh>

using collision_benchmark::PhysicsWorldBase;
using collision_benchmark::PhysicsWorld;
using collision_benchmark::GazeboPhysicsWorld;
using collision_benchmark::GazeboMirrorWorld;
using collision_benchmark::GazeboStateCompare;

class MultipleWorldsTest : public ::testing::Test {
  protected:

  MultipleWorldsTest()
  :fakeProgramName("MultipleWorldsTest")
  {
  }
  virtual ~MultipleWorldsTest()
  {
  }

  virtual void SetUp()
  {
    // irrelevant to pass fake argv, so make an exception
    // and pass away constness, so that fakeProgramName can be
    // initialized easily in constructor.
    gazebo::setupServer(1, (char**)&fakeProgramName);
  }

  virtual void TearDown()
  {
    gazebo::shutdown();
  }

  private:
  const char * fakeProgramName;
};

/// load all worlds from file, in the order given in \e filenames
std::vector<gazebo::physics::WorldPtr> LoadWorlds(const std::vector<std::string>& filenames)
{
  // list of worlds to be loaded
  std::vector<collision_benchmark::Worldfile> worldsToLoad;
  int i=0;
  for (std::vector<std::string>::const_iterator it=filenames.begin(); it!=filenames.end(); ++it, ++i)
  {
    std::string worldfile = *it;
    std::stringstream worldname;
    worldname << "world_" << i - 1;
    worldsToLoad.push_back(collision_benchmark::Worldfile(worldfile,worldname.str()));
  }
  return collision_benchmark::LoadWorlds(worldsToLoad);
}


TEST_F(MultipleWorldsTest, UsesDifferentEngines)
{
  std::vector<std::string> filenames;
  std::set<std::string> engines=collision_benchmark::GetSupportedPhysicsEngines();
  // insert engines alphanumerically into filenames so that filenames has same
  // order as engines
  if (engines.count("bullet"))
    filenames.push_back("../test_worlds/empty_bullet.world");
  if (engines.count("dart"))
    filenames.push_back("../test_worlds/empty_dart.world");
  if (engines.count("ode"))
    filenames.push_back("../test_worlds/empty_ode.world");
  if (engines.count("simbody"))
    // XXX TODO add the empty_simbody.world file
    filenames.push_back("../test_worlds/empty_simbody.world");

  std::vector<gazebo::physics::WorldPtr> worlds=LoadWorlds(filenames);

  ASSERT_EQ(worlds.size(), filenames.size()) << filenames.size() << "  Worlds must have been loaded";

  std::set<std::string>::iterator eit=engines.begin();
  for (std::vector<gazebo::physics::WorldPtr>::iterator it=worlds.begin(); it!=worlds.end(); ++it, ++eit)
  {
    ASSERT_NE(it->get(),nullptr) << " World NULL pointer returned";
    ASSERT_NE((*it)->Physics().get(), nullptr) << " World PhysicsEngine cannot be NULL";
    ASSERT_EQ((*it)->Physics()->GetType(), *eit) << "Engine must be '"<<*eit
      <<"', is "<<(*it)->Physics()->GetType();
  }
}

// Same test as UseDifferentEngines, but loads from one world file and overrides the physics engine
TEST_F(MultipleWorldsTest, UsesDifferentEnginesOverride)
{
  std::vector<std::string> physics_filenames;
  std::set<std::string> engines=collision_benchmark::GetSupportedPhysicsEngines();
  // insert engines alphanumerically into filenames so that filenames has same
  // order as engines
  if (engines.count("bullet"))
    physics_filenames.push_back("../physics_settings/bullet_default.sdf");
  if (engines.count("dart"))
    physics_filenames.push_back("../physics_settings/dart_default.sdf");
  if (engines.count("ode"))
    physics_filenames.push_back("../physics_settings/ode_default.sdf");
  if (engines.count("simbody"))
    // XXX TODO add the empty_simbody.world file
    physics_filenames.push_back("../physics_settings/simbody_default.world");

  std::vector<gazebo::physics::WorldPtr> worlds;
  for (std::vector<std::string>::const_iterator it=physics_filenames.begin(); it!=physics_filenames.end(); ++it)
  {
    std::string physicsfile = *it;
    std::string worldfile = "worlds/rubble.world";
    std::cout<<"Loading physics from "<<physicsfile<<std::endl;
    sdf::ElementPtr physics = collision_benchmark::GetPhysicsFromSDF(physicsfile);
    ASSERT_NE(physics.get(), nullptr) << "Could not get phyiscs engine from "<<physicsfile;
    // std::cout<<"Physics: "<<physics->ToString("")<<std::endl;
    std::cout<<"Loading world from "<<worldfile<<std::endl;
    gazebo::physics::WorldPtr gzworld = collision_benchmark::LoadWorldFromFile(worldfile, "", physics);
    ASSERT_NE(gzworld.get(), nullptr) <<"Could not load world "<<worldfile;
    worlds.push_back(gzworld);
  }

  ASSERT_EQ(worlds.size(), physics_filenames.size()) << physics_filenames.size() << "  Worlds must have been loaded";

  std::set<std::string>::iterator eit=engines.begin();
  for (std::vector<gazebo::physics::WorldPtr>::iterator it=worlds.begin(); it!=worlds.end(); ++it, ++eit)
  {
    std::cout<<"Engine used: "<<(*it)->Physics()->GetType()<<std::endl;
    ASSERT_NE(it->get(),nullptr) << " World NULL pointer returned";
    ASSERT_NE((*it)->Physics().get(), nullptr) << " World PhysicsEngine cannot be NULL";
    ASSERT_EQ((*it)->Physics()->GetType(), *eit) << "Engine must be '"<<*eit
      <<"', is "<<(*it)->Physics()->GetType();
  }
}

TEST_F(MultipleWorldsTest, TransferWorldState)
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

int main(int argc, char**argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
