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

#include <collision_benchmark/GazeboMultipleWorlds.hh>
#include <collision_benchmark/PrimitiveShape.hh>
#include <collision_benchmark/GazeboModelLoader.hh>
#include <collision_benchmark/GazeboWorldLoader.hh>
#include <collision_benchmark/BasicTypes.hh>

#include <boost/program_options.hpp>

#include <thread>
#include <unistd.h>
#include <sys/wait.h>

using collision_benchmark::GazeboMultipleWorlds;
using collision_benchmark::GazeboModelLoader;
using collision_benchmark::Shape;
using collision_benchmark::PrimitiveShape;
using collision_benchmark::GazeboMultipleWorlds;
using collision_benchmark::BasicState;

namespace po = boost::program_options;

typedef GazeboMultipleWorlds::GzWorldManager
          ::PhysicsWorldModelInterfaceT::Vector3 Vector3;

// Helper fuction which returns the AABB of the model from the first
// world in \e worldManager.
// Presumes that the model exists in all worlds and the AABB would be
// the same (or very, very similar) in all worlds.
// See also collision_benchmark::GetConsistentAABB() (in test/TestUtils.hh)
// which checks that all AABBs are the same in both worlds. Automated tests
// have ensured that this is the case if the model has been loaded in
// all collision engine worlds simultaneously and the world hasn't been updated
// since the model was added.
//
// \retval 0 success
// \retval -1 the model does not exist in the first world.
// \retval -2 no worlds in world manager
// \retval -3 could not get AABB from model
int GetAABB(const std::string& modelName,
            const GazeboMultipleWorlds::GzWorldManager::Ptr& worldManager,
            Vector3& min, Vector3& max, bool& inLocalFrame)
{
  std::vector<GazeboMultipleWorlds::GzWorldManager
              ::PhysicsWorldModelInterfacePtr>
    worlds = worldManager->GetModelPhysicsWorlds();

  if (worlds.empty()) return -2;

  GazeboMultipleWorlds::GzWorldManager::PhysicsWorldModelInterfacePtr w =
    worlds.front();
  if (!w->GetAABB(modelName, min, max, inLocalFrame))
  {
      std::cerr << "Model " << modelName << ": AABB could not be retrieved"
                << std::endl;
      return -3;
  }
  return 0;
}


// Helper function which gets state of the model in the first world of the
// \e worldManager. Presumes that the model exists in all worlds and the state
// would be the same (or very, very similar) in all worlds.
// \retval 0 success
// \retval -1 the model does not exist in the first world.
// \retval -2 no worlds in world manager
// \retval -3 could not get state of model
int GetBasicModelState(const std::string& modelName,
               const GazeboMultipleWorlds::GzWorldManager::Ptr& worldManager,
               BasicState& state)
{
  std::vector<GazeboMultipleWorlds::GzWorldManager
              ::PhysicsWorldModelInterfacePtr>
    worlds = worldManager->GetModelPhysicsWorlds();

  if (worlds.empty()) return -2;

  GazeboMultipleWorlds::GzWorldManager::PhysicsWorldModelInterfacePtr w =
    worlds.front();
  if (!w->GetBasicModelState(modelName, state))
  {
      std::cerr << "Model " << modelName << ": state could not be retrieved"
                << std::endl;
      return -3;
  }
  return 0;
}

// for the thread handling the collision bar
std::atomic<bool> running;

// handles the collision bar
void collisionBar(const std::string& mirrorName)
{
  // make a model publisher which will publish the bar to gzclient
  gazebo::transport::NodePtr node =
    gazebo::transport::NodePtr(new gazebo::transport::Node());
  node->Init();
  gazebo::transport::PublisherPtr modelPub =
    node->Advertise<gazebo::msgs::Model>("~/model/info");

  while (!modelPub->HasConnections())
  {
    std::cout << "Waiting gzclient subscriber to models." << std::endl;
    gazebo::common::Time::MSleep(500);
  }

  gazebo::transport::PublisherPtr posePub =
    node->Advertise<gazebo::msgs::PosesStamped>("~/pose/info");
  while (!posePub->HasConnections())
  {
    std::cout << "Waiting gzclient subscriber to pose." << std::endl;
    gazebo::common::Time::MSleep(500);
  }

  gzerr << "TEMPORARY: Wait for all other connections to be there (eg. ModelListWidget)" << std::endl;
  gazebo::common::Time::MSleep(2000);

  int visualID = 100;

  gazebo::msgs::Model modelMsg;
  modelMsg.set_name("collision_bar");
  modelMsg.set_id(visualID);
  modelMsg.set_is_static(true);
  ignition::math::Pose3d barPose;
  gazebo::msgs::Set(modelMsg.mutable_pose(), barPose);

  // create visual
  collision_benchmark::Shape::Ptr shape
    (collision_benchmark::PrimitiveShape::CreateCylinder(1,6));
  sdf::ElementPtr shapeGeom=shape->GetShapeSDF();
  sdf::ElementPtr visualSDF(new sdf::Element());
  visualSDF->SetName("visual");
  visualSDF->AddAttribute("name", "string", "visual", true, "visual name");
  visualSDF->InsertElement(shapeGeom);
  gazebo::msgs::Visual visualMsg = gazebo::msgs::VisualFromSDF(visualSDF);
  visualMsg.set_name("collision_bar_visual");
  ignition::math::Pose3d visualRelPose;
  gazebo::msgs::Set(visualMsg.mutable_pose(), visualRelPose);
  visualMsg.set_type(gazebo::msgs::Visual::VISUAL);
  visualMsg.set_is_static(true);
  // This is a bit of a HACK at this point:
  // Parent name must be scene name (the one gzclient subscribes to,
  // which is the mirror world), otherwise the visual won't be added
  // properly. Alternatively, it can be a random name, but instaed the
  // parent ID must be set to a known ID. 0 is used for the global scene
  // so the can be done.
  // See rendering::Scene::ProcessVisualMsg()
#if 0  // first solution: use mirror name as parent
  visualMsg.set_parent_name(mirrorName);
#else  // second solution: use invalid parent name but set ID to 0
  visualMsg.set_parent_name("invalid-name");
  // the visual parent ID will only be used if the parent name is not mirror.
  visualMsg.set_parent_id(0);
#endif

  // Visual must have ID assigned in order to be added under this ID
  // (otherwise it will be assigned a random ID),
  // so we can access it under this ID again.
  // See rendering::Scene::ProcessVisualMsg()
  visualMsg.set_id(visualID);

  // add the visual and publish the model message
  modelMsg.add_visual()->CopyFrom(visualMsg);
  modelPub->Publish(modelMsg);

  while (running)
  {
    ignition::math::Vector3d p(0,0,0);
    ignition::math::Quaterniond q(1,0,0,0);
    ignition::math::Pose3d origPose(p,q);
    gazebo::msgs::PosesStamped poseMsg;
    gazebo::msgs::Set(poseMsg.mutable_time(), 0);
    gazebo::msgs::Pose * singlePoseMsg = poseMsg.add_pose();

    singlePoseMsg->set_name("collision_bar");
    singlePoseMsg->set_id(visualID);

    // singlePoseMsg->set_name("unit_sphere_0");
    // singlePoseMsg->set_id(31);

    gazebo::msgs::Set(singlePoseMsg, origPose);
    posePub->Publish(poseMsg);
    gazebo::common::Time::MSleep(100);
  }
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  std::vector<std::string> selectedEngines;
  std::vector<std::string> unitShapes;
  std::vector<std::string> sdfModels;

  // Read command line parameters
  // ----------------------

  // description for engine options as stream so line doesn't go over 80 chars.
  std::stringstream descEngines;
  descEngines <<  "Specify one or several physics engines. " <<
      "Can contain [ode, bullet, dart, simbody]. Default is [ode].";

  // Declare the supported options.
  po::options_description desc("Allowed options");
  desc.add_options()
    ("help,h", "Produce help message")
    ("shape,s",
      po::value<std::vector<std::string>>(&unitShapes)->multitoken(),
      "Unit shape specification, can be any of [sphere, cylinder, cube]")
    ("model,m",
      po::value<std::vector<std::string>>(&sdfModels)->multitoken(),
      std::string(std::string("Model specification, can be either the ") +
      std::string("name of a model in the gazebo model paths, or a ") +
      std::string("path to a SDF file")).c_str())
    ;

  po::options_description desc_hidden("Positional options");
  desc_hidden.add_options()
    ("engines,e",
      po::value<std::vector<std::string>>(&selectedEngines)->multitoken(),
      descEngines.str().c_str())
    ;

  po::variables_map vm;
  po::positional_options_description p;
  // positional arguments default to "engines" argument
  p.add("engines", -1);

  po::options_description desc_composite;
  desc_composite.add(desc).add(desc_hidden);

  po::command_line_parser parser{argc, argv};
  parser.options(desc_composite).positional(p); // .allow_unregistered();
  po::parsed_options parsedOpt = parser.run();
  po::store(parsedOpt, vm);
  po::notify(vm);

  if (vm.count("help"))
  {
    std::cout << argv[0] <<" <list of engines> " << std::endl;
    std::cout << desc << std::endl;
    return 1;
  }

  if (vm.count("engines"))
  {
    std::cout << "Engines to load: " << std::endl;
    for (std::vector<std::string>::iterator it = selectedEngines.begin();
         it != selectedEngines.end(); ++it)
    {
      std::cout<<*it<<std::endl;
    }
  }
  else
  {
    std::cout << "No engines were specified, so using 'ode'" << std::endl;
    selectedEngines.push_back("ode");
  }

/*  if (vm.count("shape"))
  {
    std::cout << "Shapes specified " << vm.count("shape") << std::endl;
    for (std::vector<std::string>::iterator it = unitShapes.begin();
         it != unitShapes.end(); ++it)
    {
      std::cout<<*it<<std::endl;
    }
  }

  if (vm.count("model"))
  {
    std::cout << "Models specified " << vm.count("model") << std::endl;
    for (std::vector<std::string>::iterator it = sdfModels.begin();
         it != sdfModels.end(); ++it)
    {
      std::cout<<*it<<std::endl;
    }
  }
*/

  if (unitShapes.size() + sdfModels.size() != 2)
  {
    std::cerr << "Have to specify exactly two shapes or models" << std::endl;
    std::cerr << "Specified shapes: " << std::endl;
    for (std::vector<std::string>::iterator it = unitShapes.begin();
         it != unitShapes.end(); ++it)
      std::cout<<" " << *it<<std::endl;
    std::cerr << "Models: " << std::endl;
    for (std::vector<std::string>::iterator it = sdfModels.begin();
         it != sdfModels.end(); ++it)
      std::cout<<" " << *it<<std::endl;
    std::cout << "which makes a total of " << (unitShapes.size() +
                 sdfModels.size()) << " models. " << std::endl;
    return 1;
  }


  // Initialize server
  // ----------------------
  bool loadMirror = true;
  bool enforceContactCalc=false;
  bool allowControlViaMirror = true;
  GazeboMultipleWorlds gzMultiWorld;

  // physics should be disable as this test only is meant to
  // display the contacts.
  bool physicsEnabled = false;
  gzMultiWorld.Load(selectedEngines, physicsEnabled,
                    loadMirror, enforceContactCalc, allowControlViaMirror);

  // Load extra models to the world
  // ----------------------
  typedef GazeboMultipleWorlds::GzWorldManager GzWorldManager;
  GzWorldManager::Ptr worldManager
    = gzMultiWorld.GetWorldManager();
  assert(worldManager);

  std::vector<std::string> loadedModelNames;

  // load primitive shapes
  typedef GzWorldManager::ModelLoadResult ModelLoadResult;
  int i = 0;
  for (std::vector<std::string>::iterator it = unitShapes.begin();
       it != unitShapes.end(); ++it, ++i)
  {
    const std::string& shapeID = *it;
    Shape::Ptr shape;
    if (shapeID == "sphere")
    {
      float radius = 1;
      shape.reset(PrimitiveShape::CreateSphere(radius));
    }
    else if (shapeID == "cylinder")
    {
    }
    else if (shapeID == "cube")
    {
    }
    else
    {
      std::cerr << "Unknown shape type: " << shapeID << std::endl;
      return 1;
    }
    std::string modelName = "unit_" + shapeID + "_" + std::to_string(i);
    std::vector<ModelLoadResult> res
      = worldManager->AddModelFromShape(modelName, shape, shape);
    if (res.size() != worldManager->GetNumWorlds())
    {
      std::cerr << "Model must have been loaded in all worlds" << std::endl;
      return 1;
    }
    loadedModelNames.push_back(modelName);
  }

  // load models from SDF
  i = 0;
  for (std::vector<std::string>::iterator it = sdfModels.begin();
       it != sdfModels.end(); ++it, ++i)
  {
    const std::string& modelResource = *it;
    std::string modelSDF =
      GazeboModelLoader::GetModelSdfFilename(modelResource);
    // std::cout << "Loading model: " << modelSDF << std::endl;
    std::string modelName = "model" + std::to_string(i);
    std::cout << "Adding model name: " << modelName
              << " from resource " << modelSDF << std::endl;
    std::vector<ModelLoadResult> res =
      worldManager->AddModelFromFile(modelSDF, modelName);
    if (res.size() != worldManager->GetNumWorlds())
    {
      std::cerr << "Model must have been loaded in all worlds" << std::endl;
      return 1;
    }
    loadedModelNames.push_back(modelName);
  }

  if (loadedModelNames.size() != 2)
    throw std::runtime_error("Inconsistency: There have to be two models");

  // position models so they're not intersecting.
  // ----------------------

  // Get the AABB's of the two models
  Vector3 min1, min2, max1, max2;
  bool local1, local2;
  if ((GetAABB(loadedModelNames[0], worldManager, min1, max1, local1) != 0) ||
      (GetAABB(loadedModelNames[1], worldManager, min2, max2, local2) != 0))
  {
    std::cerr << "Could not get AABBs of models" << std::endl;
    return 1;
  }

  std::cout << "AABB 1: " << min1 << " -- " << max1 << std::endl;
  std::cout << "AABB 2: " << min2 << " -- " << max2 << std::endl;

  // separate them along the desired global coodrinate frame axis.
  // Leave model 1 where it is and move model 2 away from it.
  Vector3 axis(0,0,1); // can be unit x, y or z axis
  double dist = min2.Dot(axis) - max1.Dot(axis);
  double desiredDistance = 0;
  double moveDistance = desiredDistance - dist;
  std::cout << "Move model 2 along axis " << axis*moveDistance << std::endl;

  BasicState modelState2;
  if (GetBasicModelState(loadedModelNames[1], worldManager, modelState2) != 0)
  {
    std::cerr << "Could not get BasicModelState." << std::endl;
    return 1;
  }

  if (!modelState2.PosEnabled())
  {
    std::cerr << "Model is expected to have a position" << std::endl;
    return 1;
  }

  axis *= moveDistance;
  std::cout << "State of model 2: " << modelState2 << std::endl;
  collision_benchmark::Vector3 modelPos = modelState2.position;
  modelPos.x += axis.X();
  modelPos.y += axis.Y();
  modelPos.z += axis.Z();
  modelState2.SetPosition(modelPos);
  worldManager->SetBasicModelState(loadedModelNames[1], modelState2);

  // start the thread to handle the collision bar
  running = true;
  std::thread t(collisionBar, gzMultiWorld.GetMirrorName());

  // Run the world(s)
  // ----------------------
  bool waitForStart = true;
  gzMultiWorld.Run(waitForStart);

  // end the thread to handle the collision bar
  running = false;
  //std::cout << "Joining thread" << std::endl;
  t.join();

  std::cout << "Bye, bye." << std::endl;
  return 0;
}
