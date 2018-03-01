/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#include <collision_benchmark/WorldManager.hh>
#include <collision_benchmark/PrimitiveShape.hh>
#include "test/MultipleWorldsTestFramework.hh"

#include <gazebo/gazebo.hh>

using collision_benchmark::Shape;
using collision_benchmark::PrimitiveShape;

/**
 * \brief subclass to create a new test group
 */
class MyTest:
  public MultipleWorldsTestFramework
{
  public:
};

//////////////////////////////////////////////////////////////////////////////
// Example for physics engine testing.
// Creates two shapes and adds them to the worlds.
TEST_F(MyTest, ExampleTest)
{
  // get all supported physics engines
  std::set<std::string> engines =
    collision_benchmark::GetSupportedPhysicsEngines();
  // make sure there are at least two supported engines
  ASSERT_GE(engines.size(), 2)
    << "Need at least two physics engines";

  // run test on all engines
  std::vector<std::string> selectedEngines;
  selectedEngines.insert(selectedEngines.end(), engines.begin(), engines.end());

  // Create shape 1
  std::string modelName1 = "model1";
  Shape::Ptr shape1(PrimitiveShape::CreateBox(2, 2, 2));
  // Create shape 2
  std::string modelName2 = "model2";
  Shape::Ptr shape2(PrimitiveShape::CreateCylinder(1, 3));

  // Running the test in interactive mode will also bring up gzclient
  bool interactive = true;
  InitMultipleEngines(selectedEngines, interactive);

  // Get the world manager
  ASSERT_NE(GetServer(), nullptr) << "Could not create and start server";
  GzWorldManager::Ptr worldManager = GetServer()->GetWorldManager();
  ASSERT_NE(worldManager.get(), nullptr) << "No valid world manager created";

  // load the two shapes into the world
  LoadShape(shape1, modelName1);
  LoadShape(shape2, modelName2);

  // ...
  // Do your tests here
  // ...

  // if you run the test in interactive mode, gzclient will not have
  // been loaded by the time this test finishes, so we will end the
  // test only once the client is closed.
  if (interactive)
  {
    // GetMultipleWorlds() returns the GazeboMultipleWorlds instance
    // which runs as the server. This class supports a client in addition
    // to a server, just like gazebo starts both gzserver and gzclient.
    while (GetMultipleWorlds()->IsClientRunning())
    {
      gazebo::common::Time::MSleep(100);
    }
  }
}

int main(int argc, char**argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
