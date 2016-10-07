#include <gtest/gtest.h>

class MultipleWorldsTest : public ::testing::Test {
  protected:

  MultipleWorldsTest()
  {
  }
  virtual ~MultipleWorldsTest()
  {
  }

  virtual void SetUp()
  {
  }

  virtual void TearDown()
  {
  }
};

TEST_F(MultipleWorldsTest, UsesDifferentEngines)
{
  int test = 1;
  ASSERT_EQ(test,1) << "Test must be 1";
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
