#include <gtest/gtest.h>
#include <gazebo/gazebo.hh>

class BasicTestFramework : public ::testing::Test {
  protected:

  BasicTestFramework()
  :fakeProgramName("BasicTestFramework")
  {
  }
  virtual ~BasicTestFramework()
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
