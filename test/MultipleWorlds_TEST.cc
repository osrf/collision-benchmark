#include <gtest/gtest.h>

#include <collision_benchmark/WorldLoader.hh>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

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
  std::vector<std::string> engines;
  if (gazebo::physics::PhysicsFactory::IsRegistered("ode"))
  {
    filenames.push_back("../test_worlds/empty_ode.world");
    engines.push_back("ode");
  }
  if (gazebo::physics::PhysicsFactory::IsRegistered("bullet"))
  {
    filenames.push_back("../test_worlds/empty_bullet.world");
    engines.push_back("bullet");
  }
  if (gazebo::physics::PhysicsFactory::IsRegistered("dart"))
  {
    filenames.push_back("../test_worlds/empty_dart.world");
    engines.push_back("dart");
  }

  std::vector<gazebo::physics::WorldPtr> worlds=LoadWorlds(filenames);

  std::cout<<worlds.size()<<" worlds loaded."<<std::endl;

  if (worlds.size() != filenames.size() ) std::cout<<"Worlds not loaded"<<std::endl;

  ASSERT_EQ(worlds.size(), filenames.size()) << filenames.size() << "  Worlds must have been loaded";

  int i=0;
  for (std::vector<gazebo::physics::WorldPtr>::iterator it=worlds.begin(); it!=worlds.end(); ++it, ++i)
  {
    ASSERT_NE(it->get(),nullptr) << " World NULL pointer returned";
    ASSERT_NE((*it)->GetPhysicsEngine().get(), nullptr) << " World PhysicsEngine cannot be NULL";
    ASSERT_EQ((*it)->GetPhysicsEngine()->GetType(), engines[i]) << "Engine must be '"<<engines[i]
      <<"', is "<<(*it)->GetPhysicsEngine()->GetType();
  }
}

TEST_F(MultipleWorldsTest, OtherTest)
{
  int test = 2;
  ASSERT_EQ(test,2) << "Test must be 2";
}


int main(int argc, char**argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
