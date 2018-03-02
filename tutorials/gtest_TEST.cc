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
#include <collision_benchmark/BasicTypes.hh>

#include "test/MultipleWorldsTestFramework.hh"

#include <gazebo/gazebo.hh>

using collision_benchmark::Shape;
using collision_benchmark::PrimitiveShape;
using collision_benchmark::BasicState;

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

  // Get the world manager.
  // GzWorldManager is defined in MultipleWorldsTestFramework and is an
  // instantiation of WorldManager with all gazebo types.
  ASSERT_NE(GetServer(), nullptr) << "Could not create and start server";
  GzWorldManager::Ptr worldManager = GetServer()->GetWorldManager();
  ASSERT_NE(worldManager.get(), nullptr) << "No valid world manager created";

  // load the two shapes into the world
  LoadShape(shape1, modelName1);
  LoadShape(shape2, modelName2);

  // Example test: we will move around the first model around using
  // WorldManager to change its pose in ALL worlds.
  // We will then ensure that it really has the same pose in all worlds
  // (we disable physics so the poses are expected to be exactly the same)

  // disable physics
  worldManager->SetDynamicsEnabled(false);

  // in a number of iterations we will move model 1 in all worlds.
  const int numIters = 1000;
  // model 1 will move in circles. This is the current angle on the circle.
  float angle = 0;
  // model 1 will turn in cirlces as many times
  const int numCircles = 2;
  // amoutn to increase the angle
  const float angleInc = 2*M_PI*numCircles/numIters;
  // current state of model 1
  BasicState modelState1;

  // Loop through all world updates in which model 1 turns numCircle circle(s)
  for (int i=0; i < numIters; ++i, angle+=angleInc)
  {
    // update the model pose in all worlds
    modelState1.SetPosition(cos(angle), sin(angle), 0);
    ASSERT_EQ(worldManager->SetBasicModelState(modelName1, modelState1),
              worldManager->GetNumWorlds())
        << "Could not set model pose to required pose";

    // Convenience typedef for PhysicsWorldModelInterface.
    typedef GzWorldManager::PhysicsWorldModelInterfaceT
      PhysicsWorldModelInterfaceT;

    // get a vector with all worlds from WorldManager
    // We will use the PhysicsWorldModelInterface here because we only need
    // to access model-specific types, but to have the full interface we
    // could also use the GzWorldManager::PhysicsWorldInterfaceT interface
    // instead and call worldManager->GetPhysicsWorlds().
    std::vector<PhysicsWorldModelInterfaceT::Ptr>
      allWorlds = worldManager->GetModelPhysicsWorlds();

    // check that the models have the same position in all worlds.
    for (std::vector<PhysicsWorldModelInterfaceT::Ptr>::const_iterator
         it = allWorlds.begin(); it != allWorlds.end(); ++it)
    {
      // convenience pointer to the current world
      const PhysicsWorldModelInterfaceT::ConstPtr world = *it;

      // get the state of the model
      BasicState state1;
      ASSERT_TRUE(world->GetBasicModelState(modelName1, state1))
        << "Could not get basic model state for " << modelName1;

      // check that the state is the same as modelState1 which we used above
      // to set the state in all worlds
      EXPECT_DOUBLE_EQ(modelState1.position.x, state1.position.x)
        << "x position is expected to be equal";
      EXPECT_DOUBLE_EQ(modelState1.position.y, state1.position.y)
        << "y position is expected to be equal";
      EXPECT_DOUBLE_EQ(modelState1.position.z, state1.position.z)
        << "z position is expected to be equal";
    }

    // Update the world one step
    worldManager->Update(1);
  }

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
