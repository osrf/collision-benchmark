#include <collision_benchmark/GazeboWorldLoader.hh>
#include <collision_benchmark/PhysicsWorld.hh>
#include <collision_benchmark/GazeboPhysicsWorld.hh>
#include <collision_benchmark/GazeboStateCompare.hh>
#include <collision_benchmark/GazeboHelpers.hh>
#include <collision_benchmark/WorldManager.hh>
#include <collision_benchmark/boost_std_conversion.hh>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include "BasicTestFramework.hh"

using collision_benchmark::PhysicsWorld;
using collision_benchmark::GazeboPhysicsWorld;
using collision_benchmark::GazeboStateCompare;
using collision_benchmark::WorldManager;


class MultipleWorldsTest : public BasicTestFramework {};


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
    // XXX TODO add the simbody file
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


int main(int argc, char**argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
