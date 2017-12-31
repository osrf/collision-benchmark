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
#include <collision_benchmark/SimpleTriMeshShape.hh>
#include <collision_benchmark/BasicTypes.hh>
#include <collision_benchmark/GazeboModelLoader.hh>

#include <collision_benchmark/MeshShapeGeneratorVtk.hh>

#include <gazebo/gazebo.hh>
#include <gazebo/test/helper_physics_generator.hh>

#include "ContactsFlickerTestFramework.hh"

using collision_benchmark::Shape;
using collision_benchmark::PrimitiveShape;
using collision_benchmark::SimpleTriMeshShape;
using collision_benchmark::GazeboModelLoader;

// tolerance for values close to zero: All contacts as close to zero will be
// considered "just touching" and disagreement of engines won't be triggered.
const double zeroDepthTol = 5e-02;

// Default value to run tests interactively (if false, automated)
bool defaultInteractive = false;

// Default output path (empty string prevents writing to file)
std::string defaultOutputPath = "";

/**
 * \brief subclass to create a new test group
 */
class ContactsFlickerTest:
  public ContactsFlickerTestFramework
{
  public:
};

/**
 * \brief subclass to create a new test group
 */
class ContactsFlickerTestWithParam:
  public ContactsFlickerTestFramework,
  public testing::WithParamInterface<const char*>
{
  public:
};


//////////////////////////////////////////////////////////////////////////////
TEST_P(ContactsFlickerTestWithParam, BoxTriangleTest)
{
  std::string engine = GetParam();
  static const bool interactive = defaultInteractive;

  // XXX HACK: For now, only test with ODE
  if (engine != "ode") return;

  std::string boxSDF = GazeboModelLoader::GetModelSdfFilename("box");
  std::string triangleSDF = GazeboModelLoader::GetModelSdfFilename("triangle");

  ASSERT_NE(triangleSDF, "") << "Triangle model not found";
  ASSERT_NE(boxSDF, "") << "Box model not found";

  std::string modelName1 = "box";
  std::string modelName2 = "triangle";

  InitOneEngine(engine, 1, interactive, {"libcollision_benchmark_step_gui.so"});
  LoadModel(boxSDF, modelName1);
  LoadModel(triangleSDF, modelName2);
  FlickerTest(modelName1, modelName2,
              interactive, defaultOutputPath, "BoxCylinderTest");
}

// cannot test simbody because there are still issues with meshes and
// lack of bounding box support
INSTANTIATE_TEST_CASE_P(PhysicsEngines, ContactsFlickerTestWithParam,
                        ::testing::Values("ode", "bullet", "dart"));
                        // PHYSICS_ENGINE_VALUES);

int main(int argc, char**argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  for (int i = 1; i < argc; ++i)
  {
    if (strcmp(argv[i], "--interactive") == 0)
    {
      defaultInteractive = true;
    }
    else if (strcmp(argv[i], "--output") == 0)
    {
      if (i+1 >= argc)
      {
        std::cerr << "--output requires specification of a path" << std::endl;
        continue;
      }
      ++i;
      defaultOutputPath = argv[i];
      std::cout << "Writing files to " << defaultOutputPath << std::endl;
    }
    else
    {
      std::cerr << "Unrecognized command line parameter: "
                << argv[i] << std::endl;
    }
  }
  return RUN_ALL_TESTS();
}
