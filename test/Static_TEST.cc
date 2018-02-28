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
#include <collision_benchmark/MeshShapeGeneratorVtk.hh>

#include <gazebo/gazebo.hh>
#include <gazebo/test/helper_physics_generator.hh>

#include "StaticTestFramework.hh"

using collision_benchmark::Shape;
using collision_benchmark::PrimitiveShape;
using collision_benchmark::SimpleTriMeshShape;

// tolerance for values close to zero: All contacts as close to zero will be
// considered "just touching" and disagreement of engines won't be triggered.
const double zeroDepthTol = 5e-02;

// minimum agreement of engines that has to be reached, value in range [0..1].
const double minAgree = 0.999;


// Default tolerance for comparison of bounding box sizes. The min
// and max coordinates (per x, y, z) are allowed to vary by this much.
const double bbTol = 1e-01;

// Default value to run tests interactively (if false, automated)
bool defaultInteractive = false;

// Default output path (empty string prevents writing to file)
std::string defaultOutputPath = "";

  /**
   * \brief subclass to create a new test group
   */
  class StaticTest:
    public StaticTestFramework
  {
    public:
  };

  /**
   * \brief subclass to create a new test group
   */
  class StaticTestWithParam:
    public StaticTestFramework,
    public testing::WithParamInterface<const char*>
  {
    public:
  };

//////////////////////////////////////////////////////////////////////////////
// Helper to create a simple shape out of two triangles
Shape::Ptr GetSimpleTestTriangle(const std::string &modelName)
{
  std::string modelName1 = "model1";
  // create simple mesh for testing
  SimpleTriMeshShape::MeshDataPtr meshData(new SimpleTriMeshShape::MeshDataT());
  typedef SimpleTriMeshShape::Vertex Vertex;
  typedef SimpleTriMeshShape::Face Face;
  std::vector<Vertex>& vertices = meshData->GetVertices();
  std::vector<Face>& triangles = meshData->GetFaces();
  vertices.push_back(Vertex(-1, 0, 0));
  vertices.push_back(Vertex(0, 0, -1));
  vertices.push_back(Vertex(1, 0, 0));
  vertices.push_back(Vertex(0, 1, 0));
  triangles.push_back(Face(0, 1, 2));
  triangles.push_back(Face(0, 2, 3));
  Shape::Ptr shape(new SimpleTriMeshShape(meshData, modelName1));
  return shape;
}


//////////////////////////////////////////////////////////////////////////////
// AABBTestWorldsAgreement with one cylinder primitive and one box primitive
TEST_F(StaticTest, BoxCylinderTest)
{
  std::vector<std::string> selectedEngines;
#ifdef BULLET_SUPPORT
  selectedEngines.push_back("bullet");
#endif
  selectedEngines.push_back("ode");
#ifdef DART_SUPPORT
  selectedEngines.push_back("dart");
#endif

  /*std::set<std::string> engines =
    collision_benchmark::GetSupportedPhysicsEngines();
  // run test on all engines
  selectedEngines.insert(selectedEngines.end(), engines.begin(), engines.end());*/

  // Model 1
  std::string modelName1 = "model1";
  Shape::Ptr shape1(PrimitiveShape::CreateBox(2, 2, 2));
  // Model 2
  std::string modelName2 = "model2";
  Shape::Ptr shape2(PrimitiveShape::CreateCylinder(1, 3));

  InitMultipleEngines(selectedEngines, defaultInteractive);
  LoadShape(shape1, modelName1);
  LoadShape(shape2, modelName2);
  static const bool interactive = defaultInteractive;
  static const float cellSizeFactor = 0.1;
  AABBTestWorldsAgreement(modelName1, modelName2, cellSizeFactor, minAgree,
           bbTol, zeroDepthTol, interactive,
           defaultOutputPath, "BoxCylinderTest");
}

//////////////////////////////////////////////////////////////////////////////
// AABBTestWorldsAgreement with one cylinder primitive and a simple
// triangle (GetSimpleTestTriangle)
TEST_F(StaticTest, CylinderAndTwoTriangles)
{
  std::vector<std::string> selectedEngines;
#ifdef BULLET_SUPPORT
  selectedEngines.push_back("bullet");
#endif
  selectedEngines.push_back("ode");
#ifdef DART_SUPPORT
  selectedEngines.push_back("dart");
#endif
  // selectedEngines.push_back("simbody");

  /*std::set<std::string> engines =
    collision_benchmark::GetSupportedPhysicsEngines();
  // run test on all engines
  selectedEngines.insert(selectedEngines.end(),
                         engines.begin(), engines.end());*/

  // Model 1
  std::string modelName1 = "model1";
  Shape::Ptr shape1 = GetSimpleTestTriangle(modelName1);

  // Model 2
  std::string modelName2 = "model2";
  Shape::Ptr shape2(PrimitiveShape::CreateCylinder(1, 3));

  InitMultipleEngines(selectedEngines, defaultInteractive);
  LoadShape(shape1, modelName1);
  LoadShape(shape2, modelName2);
  static const bool interactive = defaultInteractive;
  static const float cellSizeFactor = 0.1;
  AABBTestWorldsAgreement(modelName1, modelName2, cellSizeFactor, minAgree,
                          bbTol, zeroDepthTol, interactive,
                          defaultOutputPath, "CylinderAndTwoTriangles");
}

//////////////////////////////////////////////////////////////////////////////
// AABBTestWorldsAgreement with one sphere primitive and one sphere mesh
TEST_F(StaticTest, SpherePrimMesh)
{
  std::vector<std::string> selectedEngines;
#ifdef BULLET_SUPPORT
  selectedEngines.push_back("bullet");
#endif
  selectedEngines.push_back("ode");
#ifdef DART_SUPPORT
  selectedEngines.push_back("dart");
#endif
  // selectedEngines.push_back("simbody");

  /*std::set<std::string> engines =
    collision_benchmark::GetSupportedPhysicsEngines();
  // run test on all engines
  selectedEngines.insert(selectedEngines.end(),
                         engines.begin(), engines.end());*/

  typedef SimpleTriMeshShape::MeshDataT::VertexPrecision Precision;
  collision_benchmark::MeshShapeGenerator<Precision>::Ptr generator
      (new collision_benchmark::MeshShapeGeneratorVtk<Precision>());

  double radius = 2;

  // sphere as mesh
  std::string meshName = "SphereMesh";
  SimpleTriMeshShape::MeshDataT::Ptr sphereMeshData =
    generator->MakeSphere(radius, 10, 10);
  Shape::Ptr sphereMesh(new SimpleTriMeshShape(sphereMeshData, meshName));

  // sphere as a primitive
  std::string primName = "SpherePrimitive";
  Shape::Ptr spherePrimitive(PrimitiveShape::CreateSphere(radius));

  // load up the worlds
  InitMultipleEngines(selectedEngines, defaultInteractive);
  LoadShape(sphereMesh, meshName);
  LoadShape(spherePrimitive, primName);
  static const bool interactive = defaultInteractive;
  static const float cellSizeFactor = 0.1;
  AABBTestWorldsAgreement(meshName, primName, cellSizeFactor, minAgree,
                          bbTol, zeroDepthTol, interactive,
                          defaultOutputPath, "SpherePrimMesh");
}

//////////////////////////////////////////////////////////////////////////////
// AABBTestWorldsAgreement with only one physics engine, testing a world
// with two primitive spheres and another world with two mesh representations
// of the same sphere
TEST_P(StaticTestWithParam, SphereEquivalentsTest)
{
  typedef SimpleTriMeshShape::MeshDataT::VertexPrecision Precision;
  collision_benchmark::MeshShapeGenerator<Precision>::Ptr generator
      (new collision_benchmark::MeshShapeGeneratorVtk<Precision>());

  double radius = 2;

  // sphere as mesh
  std::string modelName2 = "SphereMesh";
  SimpleTriMeshShape::MeshDataT::Ptr sphereMeshData =
    generator->MakeSphere(radius, 10, 10);
  // sphereMeshData->Perturb(-0.2, 0.2,
  //                        SimpleTriMeshShape::MeshDataT::Vertex(0, 0, 0),
  //                        SimpleTriMeshShape::MeshDataT::Vertex(0, 1, 0));
  Shape::Ptr sphereMesh(new SimpleTriMeshShape(sphereMeshData, modelName2));

  // sphere as a primitive
  std::string modelName1 = "SpherePrimitive";
  Shape::Ptr spherePrimitive(PrimitiveShape::CreateSphere(radius));

  // load up the worlds
  InitOneEngine(GetParam(), 2, defaultInteractive);

  // as a first shape, load the primitive into both worlds
  LoadShape(spherePrimitive, modelName1);
  // as the second shape, load the primitive into the
  // first world, and the mesh into the second
  LoadShape(spherePrimitive, modelName2, 0);
  LoadShape(sphereMesh, modelName2, 1);
  static const bool interactive = defaultInteractive;
  static const float cellSizeFactor = 0.1;
  const double _bbTol = 0.15;
  AABBTestWorldsAgreement(modelName1, modelName2, cellSizeFactor, minAgree,
                          _bbTol, zeroDepthTol, interactive,
                          defaultOutputPath, "SphereEquivalentTest");
}

// cannot test simbody because there are still issues with meshes and
// lack of bounding box support
INSTANTIATE_TEST_CASE_P(PhysicsEngines, StaticTestWithParam,
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
