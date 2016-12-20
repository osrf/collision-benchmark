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
  filenames.push_back("../test_worlds/empty_ode.world");
  filenames.push_back("../test_worlds/empty_bullet.world");
  filenames.push_back("../test_worlds/empty_dart.world");

  std::vector<gazebo::physics::WorldPtr> worlds=LoadWorlds(filenames);

  if (worlds.size() !=3 ) std::cout<<"Worlds not loaded"<<std::endl;

  ASSERT_EQ(worlds.size(), 3) << " 3 Worlds must have been loaded";

  for (std::vector<gazebo::physics::WorldPtr>::iterator it=worlds.begin(); it!=worlds.end(); ++it)
  {
    ASSERT_NE(it->get(),nullptr) << " World NULL pointer returned";
    ASSERT_NE((*it)->Physics().get(), nullptr) << " World PhysicsEngine cannot be NULL";
  }

  int i=0;
  ASSERT_EQ(worlds[i++]->Physics()->GetType(), "ode") << "Engine must be ODE";
  ASSERT_EQ(worlds[i++]->Physics()->GetType(), "bullet") << "Engine must be Bullet";
  ASSERT_EQ(worlds[i++]->Physics()->GetType(), "dart") << "Engine must be DART";
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
