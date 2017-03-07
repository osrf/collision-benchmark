#include <collision_benchmark/GazeboWorldLoader.hh>
#include <collision_benchmark/WorldManager.hh>
#include <collision_benchmark/PhysicsWorld.hh>
#include <collision_benchmark/GazeboPhysicsWorld.hh>
#include <collision_benchmark/GazeboHelpers.hh>
#include <collision_benchmark/boost_std_conversion.hh>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include "BasicTestFramework.hh"

using collision_benchmark::PhysicsWorld;
using collision_benchmark::GazeboPhysicsWorld;
using collision_benchmark::WorldManager;

// Gets the preset files required for the engines in \e engines.
// \param engines the engines to use [bullet, dart, ode, simbody].
// \return engine filenames are inserted alphanumerically so that
//    the returned vector has the same order as \e engines.
std::vector<std::string>
GetPhysicsPresetFiles(const std::set<std::string>& engines)
{
  std::vector<std::string> physics_filenames;
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
  return physics_filenames;
}

class StaticTest : public BasicTestFramework {};

// Loads a world with different physics engines (only the ones implemented
// in Gazebo) and does the static test in which two models are generated at
// various poses and all engines have to agree on the
// collision state (boolean collision).
TEST_F(StaticTest, TwoSpheres)
{
  std::set<std::string> engines =
    collision_benchmark::GetSupportedPhysicsEngines();
  std::vector<std::string> physics_filenames = GetPhysicsPresetFiles(engines);

  // test world
  std::string worldfile = "worlds/empty.world";

  std::vector<gazebo::physics::WorldPtr> worlds;
  for (std::vector<std::string>::const_iterator
       it=physics_filenames.begin(); it!=physics_filenames.end(); ++it)
  {
    std::string physicsfile = *it;
    std::cout<<"Loading physics from "<<physicsfile<<std::endl;
    sdf::ElementPtr physics =
      collision_benchmark::GetPhysicsFromSDF(physicsfile);
    ASSERT_NE(physics.get(), nullptr) << "Could not get phyiscs engine from "
                                      << physicsfile;
    // std::cout<<"Physics: "<<physics->ToString("")<<std::endl;
    std::cout<<"Loading world from "<<worldfile<<std::endl;
    gazebo::physics::WorldPtr gzworld =
      collision_benchmark::LoadWorldFromFile(worldfile, "", physics);
    ASSERT_NE(gzworld.get(), nullptr) <<"Could not load world "<<worldfile;
    worlds.push_back(gzworld);
  }

  ASSERT_EQ(worlds.size(), physics_filenames.size())
            << physics_filenames.size() << "  Worlds must have been loaded";

  std::set<std::string>::iterator eit=engines.begin();
  for (std::vector<gazebo::physics::WorldPtr>::iterator
       it=worlds.begin(); it!=worlds.end(); ++it, ++eit)
  {
    std::cout<<"Engine used: "<<(*it)->Physics()->GetType()<<std::endl;
    ASSERT_NE(it->get(),nullptr) << " World NULL pointer returned";
    ASSERT_NE((*it)->Physics().get(), nullptr)
      << " World PhysicsEngine cannot be NULL";
    ASSERT_EQ((*it)->Physics()->GetType(), *eit) << "Engine must be '"<<*eit
      <<"', is "<<(*it)->Physics()->GetType();
  }
}


int main(int argc, char**argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
