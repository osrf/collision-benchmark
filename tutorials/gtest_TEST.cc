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
// AABBTestWorldsAgreement with one cylinder primitive and one box primitive
TEST_F(MyTest, ExampleTest)
{
  std::vector<std::string> selectedEngines;
  std::set<std::string> engines =
    collision_benchmark::GetSupportedPhysicsEngines();
  // run test on all engines
  selectedEngines.insert(selectedEngines.end(), engines.begin(), engines.end());

  ASSERT_GE(selectedEngines.size(), 2)
    << "Need at least two physics engines";

  // Model 1
  std::string modelName1 = "model1";
  Shape::Ptr shape1(PrimitiveShape::CreateBox(2, 2, 2));
  // Model 2
  std::string modelName2 = "model2";
  Shape::Ptr shape2(PrimitiveShape::CreateCylinder(1, 3));

  // To run the test in interactive mode,
  bool interactive = true;
  InitMultipleEngines(selectedEngines, interactive);
  LoadShape(shape1, modelName1);
  LoadShape(shape2, modelName2);
}

int main(int argc, char**argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
