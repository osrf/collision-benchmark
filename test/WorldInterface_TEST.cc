#include <collision_benchmark/GazeboWorldLoader.hh>
#include <collision_benchmark/PhysicsWorld.hh>
#include <collision_benchmark/GazeboPhysicsWorld.hh>
#include <collision_benchmark/GazeboStateCompare.hh>
#include <collision_benchmark/GazeboHelpers.hh>
#include <collision_benchmark/WorldManager.hh>
#include <collision_benchmark/PrimitiveShape.hh>
#include <collision_benchmark/SimpleTriMeshShape.hh>
#include <collision_benchmark/boost_std_conversion.hh>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include "BasicTestFramework.hh"

using collision_benchmark::PhysicsWorldBaseInterface;
using collision_benchmark::PhysicsWorldStateInterface;
using collision_benchmark::PhysicsWorld;
using collision_benchmark::GazeboPhysicsWorld;
using collision_benchmark::GazeboPhysicsWorldTypes;
using collision_benchmark::GazeboStateCompare;
using collision_benchmark::WorldManager;
using collision_benchmark::Shape;
using collision_benchmark::PrimitiveShape;
using collision_benchmark::MeshData;
using collision_benchmark::SimpleTriMeshShape;

typedef gazebo::physics::WorldState GzWorldState;
typedef WorldManager<GazeboPhysicsWorldTypes::WorldState,
                     GazeboPhysicsWorldTypes::ModelID,
                     GazeboPhysicsWorldTypes::ModelPartID,
                     GazeboPhysicsWorldTypes::Vector3,
                     GazeboPhysicsWorldTypes::Wrench> GzWorldManager;

typedef PhysicsWorldStateInterface<GzWorldState> GzPhysicsWorldStateInterface;

typedef collision_benchmark::PhysicsWorldStateInterface
      <gazebo::physics::WorldState> GzPhysicsWorldStateInterface;

typedef PhysicsWorld<GazeboPhysicsWorldTypes::WorldState,
                     GazeboPhysicsWorldTypes::ModelID,
                     GazeboPhysicsWorldTypes::ModelPartID,
                     GazeboPhysicsWorldTypes::Vector3,
                     GazeboPhysicsWorldTypes::Wrench> GzPhysicsWorld;


//////////////////////////////////////////////////////
class WorldInterfaceTest : public BasicTestFramework {};


//////////////////////////////////////////////////////
TEST_F(WorldInterfaceTest, TransferWorldState)
{
  GazeboPhysicsWorld::Ptr gzWorld1(new GazeboPhysicsWorld(false));
  ASSERT_EQ(gzWorld1->LoadFromFile("worlds/empty.world", "blank"),
            collision_benchmark::SUCCESS) << " Could not load empty world";
  gzWorld1->SetDynamicsEnabled(false);

  GazeboPhysicsWorld::Ptr gzWorld2(new GazeboPhysicsWorld(false));
  ASSERT_EQ(gzWorld2->LoadFromFile("worlds/rubble.world", "rubble"),
            collision_benchmark::SUCCESS) << " Could not load rubble world";

  GzPhysicsWorldStateInterface::Ptr world1(gzWorld1);
  GzPhysicsWorldStateInterface::Ptr world2(gzWorld2);

  int numIters=3000;
  std::cout << "Doing "<<numIters<<" iterations on the rubble world and "
            << "set the second world to the same state"<<std::endl;
  for (int i=0; i<numIters; ++i)
  {
    gazebo::physics::WorldState target = world2->GetWorldState();
    collision_benchmark::OpResult setStateRet
      = world1->SetWorldState(target, false);
    ASSERT_EQ(setStateRet, collision_benchmark::SUCCESS)
      << " Could not set world state";
    gazebo::physics::WorldState newState = world1->GetWorldState();
    GazeboStateCompare::Tolerances t =
      GazeboStateCompare::Tolerances::CreateDefault(1e-03);

    // don't check dynamics because we disable physics engine in gzWorld1
    t.CheckDynamics=false;

    ASSERT_EQ(GazeboStateCompare::Equal(newState, target, t), true)
      <<"Target state was not set as supposed to!! ";
/*      <<std::endl<<std::endl<<newState<<std::endl<<std::endl<<target
      <<std::endl<<std::endl
      <<"Diff: "<<std::endl
      <<target - world1->GetWorldState();*/
    gzWorld1->Update(1);
    gzWorld2->Update(1);
  }
}

TEST_F(WorldInterfaceTest, WorldManager)
{
  std::map<std::string,std::string> physicsEngines
    = collision_benchmark::getPhysicsSettingsSdfForAllEngines();
  std::string worldfile = "../test_worlds/cube.world";

  std::cout << "Loading world " << worldfile << " with "
            << physicsEngines.size() << " engines." << std::endl;

  // create one world per physics engine and load it with the cube world,
  // and add it to the world manager
  GzWorldManager worldManager;
  int i=1;
  for (std::map<std::string,std::string>::iterator it = physicsEngines.begin();
       it!=physicsEngines.end(); ++it, ++i)
  {
    std::string engine = it->first;
    std::string physicsSDF = it->second;

    std::stringstream _worldname;
    _worldname << "world_" << i << "_" << engine;
    std::string worldname=_worldname.str();

    std::cout << "Loading with physics engine " << engine
              << " (named as '" << worldname << "')" << std::endl;

    std::cout<<"Loading physics from " << physicsSDF << std::endl;
    sdf::ElementPtr physics = collision_benchmark::GetPhysicsFromSDF(physicsSDF);
    ASSERT_NE(physics.get(), nullptr)
      << "Could not get phyiscs engine from " << physicsSDF << std::endl;

    // std::cout<<"Physics: "<<physics->ToString("")<<std::endl;
    std::cout<<"Loading world from "<<worldfile<<std::endl;
    gazebo::physics::WorldPtr gzworld =
      collision_benchmark::LoadWorldFromFile(worldfile, worldname, physics);

    ASSERT_NE(gzworld.get(), nullptr)
      << "Error loading world " << worldfile << std::endl;

    // Create the GazeboPhysicsWorld object
    GazeboPhysicsWorld::Ptr gzPhysicsWorld(new GazeboPhysicsWorld(false));
    gzPhysicsWorld->SetWorld
      (collision_benchmark::to_std_ptr<gazebo::physics::World>(gzworld));
    worldManager.AddPhysicsWorld(gzPhysicsWorld);
  }

  ASSERT_EQ(worldManager.GetNumWorlds(), physicsEngines.size())
    << "As many worlds as physics engines should have been loaded";

  // All worlds should have a cube named "box",
  // and only two models (ground and cube).
  for (int i=0; i<worldManager.GetNumWorlds(); ++i)
  {
    PhysicsWorldBaseInterface::Ptr world = worldManager.GetWorld(i);
    GzPhysicsWorldStateInterface::Ptr sWorld =
      GzWorldManager::ToWorldWithState(world);
    ASSERT_NE(sWorld, nullptr) <<"World should have been of Gazebo type";
    GzWorldState state = sWorld->GetWorldState();
    ASSERT_EQ(state.GetModelStates().size(), 2)
      << "World " << world->GetName() << " should have only two models.";
    ASSERT_NE(state.HasModelState("box"), false)
      << "World "<<world->GetName()<<" has no model named 'box'";
  }
}


/**
 * Tests the model loading methods of the GazeboPhysicsWorld
 */
TEST_F(WorldInterfaceTest, GazeboModelLoading)
{
  std::string worldfile = "../test_worlds/cube.world";

  std::cout << "Loading world " << worldfile << std::endl;


  // create a world with a cube
  GzPhysicsWorld::Ptr world (new GazeboPhysicsWorld(false));
  ASSERT_EQ(world->LoadFromFile(worldfile),collision_benchmark::SUCCESS)
    << " Could not load world";

  ASSERT_NE(world.get(), nullptr)
    << "Error loading world " << worldfile << std::endl;

  // by now, the world should only have two models: the cube and the ground
  GzWorldState state = world->GetWorldState();
  ASSERT_EQ(state.GetModelStates().size(), 2)
    << "World " << world->GetName() << " should have only two models.";

  // Now load an extra sphere to the worlds to test the
  // PhysicsWorld::AddModel* functions
  std::string forcedModelName = "test-sphere";
  GzPhysicsWorld::ModelLoadResult res =
    world->AddModelFromFile("../test_worlds/sphere.sdf",forcedModelName);
  ASSERT_EQ(res.opResult, collision_benchmark::SUCCESS)
    << " Could not add extra model to world";
  ASSERT_EQ(res.modelID, forcedModelName)
    << " Model name must have been forced to " << forcedModelName
    << " but is " << res.modelID;
  state = world->GetWorldState();
  ASSERT_EQ(state.GetModelStates().size(), 3)
    << "World " <<world->GetName() << " should have 3 models.";

  // Add another model via the String load function
  std::string sdfStr("\
    <model name='box'>\
      <pose>0 0 2.0 0 0 0</pose>\
      <link name='link'>\
        <collision name='cube_collision'>\
          <geometry>\
            <box>\
              <size>0.5 0.5 0.5</size>\
            </box>\
          </geometry>\
        </collision>\
        <visual name='cube_visual'>\
          <geometry>\
            <box>\
              <size>0.5 0.5 0.5</size>\
            </box>\
          </geometry>\
        </visual>\
      </link>\
    </model>");
  std::string forcedModelName2 = "test-cube";
  GzPhysicsWorld::ModelLoadResult res2 =
    world->AddModelFromString(sdfStr,forcedModelName2);
  ASSERT_EQ(res2.opResult, collision_benchmark::SUCCESS)
    << " Could not add extra model to world";
  ASSERT_EQ(res2.modelID, forcedModelName2)
    << " Model name must have been forced to " << forcedModelName2
    << " but is " << res2.modelID;
  state = world->GetWorldState();
  ASSERT_EQ(state.GetModelStates().size(), 4)
    << "World " << world->GetName() << " should have 4 models.";

  // Add another model via the Shape load function
  std::string forcedModelName3 = "test-shape";
  //Shape::Ptr shape(PrimitiveShape::CreateBox(2,2,2));
  //Shape::Ptr shape(PrimitiveShape::CreateSphere(2));
  //Shape::Ptr shape(PrimitiveShape::CreateCylinder(1,3));
  //Shape::Ptr shape(PrimitiveShape::CreatePlane
  //                  (Shape::Vector3(1,0,0),Shape::Vector2(10,10)));

  // create simple mesh
  SimpleTriMeshShape::MeshDataPtr meshData(new SimpleTriMeshShape::MeshDataT());
  typedef SimpleTriMeshShape::Vertex Vertex;
  typedef SimpleTriMeshShape::Face Face;
  std::vector<Vertex>& vertices=meshData->GetVertices();
  std::vector<Face>& triangles=meshData->GetFaces();
  vertices.push_back(Vertex(-1,0,0));
  vertices.push_back(Vertex(0,0,-1));
  vertices.push_back(Vertex(1,0,0));
  vertices.push_back(Vertex(0,1,0));
  triangles.push_back(Face(0,1,2));
  triangles.push_back(Face(0,2,3));
  Shape::Ptr shape(new SimpleTriMeshShape(meshData, "test_mesh"));

  shape->SetPose(Shape::Pose3(2,2,2,0,0,0));
  std::cout<<"Adding model from shape.."<<std::endl;
  GzPhysicsWorld::ModelLoadResult res3 =
    world->AddModelFromShape(forcedModelName3, shape, shape);
  ASSERT_EQ(res3.opResult, collision_benchmark::SUCCESS)
    << " Could not add extra model to world";
  ASSERT_EQ(res3.modelID, forcedModelName3)
    << " Model name must have been forced to " << forcedModelName3
    << " but is " << res3.modelID;
  state = world->GetWorldState();
  ASSERT_EQ(state.GetModelStates().size(), 5)
    << "World "<<world->GetName()<<" should have 5 models.";
}

/**
 * Tests the saving of the world. In particular, we want to test the
 * saving of worlds which have meshes which were generated with a shape.
 */
TEST_F(WorldInterfaceTest, GazeboWorldSaving)
{
  std::string worldfile = "worlds/empty.world";
  std::cout << "Loading world " << worldfile << std::endl;

  // create the world
  GzPhysicsWorld::Ptr world (new GazeboPhysicsWorld(false));
  ASSERT_EQ(world->LoadFromFile(worldfile),collision_benchmark::SUCCESS)
    << " Could not load world";
  ASSERT_NE(world.get(), nullptr)
    << "Error loading world " << worldfile << std::endl;

  // Add a model via the Shape load function
  std::string shapeName = "test-simple-shape";

  // create simple mesh
  SimpleTriMeshShape::MeshDataPtr meshData(new SimpleTriMeshShape::MeshDataT());
  typedef SimpleTriMeshShape::Vertex Vertex;
  typedef SimpleTriMeshShape::Face Face;
  std::vector<Vertex>& vertices=meshData->GetVertices();
  std::vector<Face>& triangles=meshData->GetFaces();
  vertices.push_back(Vertex(-1,0,0));
  vertices.push_back(Vertex(0,0,-1));
  vertices.push_back(Vertex(1,0,0));
  vertices.push_back(Vertex(0,1,0));
  triangles.push_back(Face(0,1,2));
  triangles.push_back(Face(0,2,3));
  Shape::Ptr shape(new SimpleTriMeshShape(meshData, "test_mesh"));

  shape->SetPose(Shape::Pose3(2,2,2,0,0,0));
  std::cout<<"Adding model from shape.."<<std::endl;
  GzPhysicsWorld::ModelLoadResult res
    = world->AddModelFromShape(shapeName, shape, shape);
  ASSERT_EQ(res.opResult, collision_benchmark::SUCCESS)
    << "Could not add test shape to world";

  std::string filename = "/tmp/test_file.world";
  ASSERT_EQ(world->SaveToFile(filename), true)
    << "Could not save world to file";

  std::ifstream inFile(filename);
  ASSERT_EQ(inFile.is_open(), true)
    << "Unable to open file[" << filename << "]\n";
}

/**
 * Tests the GetContactInfo() methods of the GazeboPhysicsWorld
 */
TEST_F(WorldInterfaceTest, GazeboContacts)
{
  std::string worldfile = "worlds/empty.world";

  std::cout << "Loading world " << worldfile << std::endl;

  // create one world per physics engine and load it with the cube world,
  // and add it to the world manager
  bool enforceContactComp=true;
  GzPhysicsWorld::Ptr world (new GazeboPhysicsWorld(enforceContactComp));
  ASSERT_EQ(world->LoadFromFile(worldfile),collision_benchmark::SUCCESS)
    << " Could not load empty world";

  // Add another cube which should fall on the ground in only very few iterations
  std::string sdfStr("\
    <model name='box'>\
      <pose>0 0 0.5 0 0 0</pose>\
      <link name='link'>\
        <collision name='cube_collision'>\
          <geometry>\
            <box>\
              <size>0.5 0.5 0.5</size>\
            </box>\
          </geometry>\
        </collision>\
        <visual name='cube_visual'>\
          <geometry>\
            <box>\
              <size>0.5 0.5 0.5</size>\
            </box>\
          </geometry>\
        </visual>\
      </link>\
    </model>");
  GzPhysicsWorld::ModelLoadResult res2 = world->AddModelFromString(sdfStr);
  ASSERT_EQ(res2.opResult, collision_benchmark::SUCCESS)
    << " Could not add extra model to world";
  // by now, the world should only have two models: the cube and the ground
  GzWorldState state = world->GetWorldState();
  ASSERT_EQ(state.GetModelStates().size(), 2)
    << "World " << world->GetName() << " should have only two models.";

  while(true)
  {
    world->Update(1);
    std::vector<GzPhysicsWorld::ContactInfoPtr> contacts1 =
      world->GetContactInfo();
    contacts1 = world->GetContactInfo();
    if (contacts1.size() > 0)
    {
      std::cout << "World has " << contacts1.size()
                << " pair of colliding models." << std::endl;
      for (auto c: contacts1) std::cout<<*c<<std::endl;
      break;
    }
  }
}


int main(int argc, char**argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
