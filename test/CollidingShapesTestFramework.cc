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
#include "CollidingShapesTestFramework.hh"
#include "CollidingShapesParams.hh"
#include <collision_benchmark/PrimitiveShape.hh>
#include <collision_benchmark/GazeboModelLoader.hh>
#include <collision_benchmark/GazeboWorldLoader.hh>
#include <collision_benchmark/BasicTypes.hh>
#include <collision_benchmark/MathHelpers.hh>

#include <gazebo/common/Timer.hh>

#include <thread>
#include <unistd.h>
#include <sys/wait.h>

using collision_benchmark::test::CollidingShapesTestFramework;
using collision_benchmark::test::CollidingShapesParams;

using collision_benchmark::GazeboMultipleWorlds;
using collision_benchmark::GazeboModelLoader;
using collision_benchmark::Shape;
using collision_benchmark::PrimitiveShape;
using collision_benchmark::GazeboMultipleWorlds;
using collision_benchmark::BasicState;

/////////////////////////////////////////////////////////////////////////////
CollidingShapesTestFramework::CollidingShapesTestFramework():
  collisionAxis(0,1,0),
  triggeredAutoCollide(false),
  shapesOnAxisPos(CollidingShapesParams::MaxSliderVal)
{
}

/////////////////////////////////////////////////////////////////////////////
bool CollidingShapesTestFramework::Run
    (const std::vector<std::string>& physicsEngines,
     const std::vector<std::string>& unitShapes,
     const std::vector<std::string>& sdfModels)
{
  static const double axEp = 1e-06;
  if (!collision_benchmark::EqualVectors(collisionAxis, Vector3(1,0,0), axEp) &&
      !collision_benchmark::EqualVectors(collisionAxis, Vector3(0,1,0), axEp) &&
      !collision_benchmark::EqualVectors(collisionAxis, Vector3(0,0,1), axEp))
  {
    std::cerr << "At this point, the only collision axes supported are x,y "
              << "or z axis. Is " << collisionAxis << std::endl;
    return false;
  }

  if (unitShapes.size() + sdfModels.size() != 2)
  {
    std::cerr << "Have to specify exactly two shapes or models" << std::endl;
    std::cerr << "Specified shapes: " << std::endl;
    for (std::vector<std::string>::const_iterator it = unitShapes.begin();
         it != unitShapes.end(); ++it)
      std::cout<<" " << *it<<std::endl;
    std::cerr << "Models: " << std::endl;
    for (std::vector<std::string>::const_iterator it = sdfModels.begin();
         it != sdfModels.end(); ++it)
      std::cout<<" " << *it<<std::endl;
    std::cout << "which makes a total of " << (unitShapes.size() +
                 sdfModels.size()) << " models. " << std::endl;
    return false;
  }

  // Initialize server
  ///////////////////////////////

  // we need to load the mirror world in order to use gzclient.
  bool loadMirror = true;
  // important to be true so contacts are calculated for all worlds,
  // not only the displayed one!
  bool enforceContactCalc=true;
  // control via mirror has to be allowed so we can move around models
  // relative to collision bar.
  bool allowControlViaMirror = true;
  // physics should be disable as this test only is meant to
  // display the contacts.
  bool physicsEnabled = false;
  gzMultiWorld.reset(new GazeboMultipleWorlds());
  std::vector<std::string> additionalGuis
      = { "libcollision_benchmark_test_gui.so" };
  gzMultiWorld->Load(physicsEngines, physicsEnabled,
                    loadMirror, enforceContactCalc,
                    allowControlViaMirror, additionalGuis);

  // Load the two models to the world(s)
  ///////////////////////////////
  GzWorldManager::Ptr worldManager = gzMultiWorld->GetWorldManager();
  assert(worldManager);

  // load primitive shapes
  typedef GzWorldManager::ModelLoadResult ModelLoadResult;
  int modelNum = 0;
  for (std::vector<std::string>::const_iterator it = unitShapes.begin();
       it != unitShapes.end(); ++it, ++modelNum)
  {
    const std::string& shapeID = *it;
    Shape::Ptr shape;
    if (shapeID == "sphere")
    {
      shape.reset(PrimitiveShape::CreateSphere(1));
    }
    else if (shapeID == "cylinder")
    {
      shape.reset(PrimitiveShape::CreateCylinder(1, 1));
    }
    else if (shapeID == "cube")
    {
      shape.reset(PrimitiveShape::CreateBox(1, 1, 1));
    }
    else
    {
      std::cerr << "Unknown shape type: " << shapeID << std::endl;
      return false;
    }
    std::string modelName = "model_" + std::to_string(modelNum);
    std::vector<ModelLoadResult> res
      = worldManager->AddModelFromShape(modelName, shape, shape);
    if (res.size() != worldManager->GetNumWorlds())
    {
      std::cerr << "Model must have been loaded in all worlds" << std::endl;
      return false;
    }
    assert(modelNum >= 0 && modelNum < 2);
    loadedModelNames[modelNum] = modelName;
  }

  // load models from SDF
  for (std::vector<std::string>::const_iterator it = sdfModels.begin();
       it != sdfModels.end(); ++it, ++modelNum)
  {
    const std::string& modelResource = *it;
    std::string modelSDF =
      GazeboModelLoader::GetModelSdfFilename(modelResource);
    std::string modelName = "model_" + std::to_string(modelNum);
    // std::cout << "Adding model " << modelName
    //          << " from resource " << modelSDF << std::endl;
    std::vector<ModelLoadResult> res =
      worldManager->AddModelFromFile(modelSDF, modelName);
    if (res.size() != worldManager->GetNumWorlds())
    {
      std::cerr << "Model must have been loaded in all worlds" << std::endl;
      return false;
    }
    assert(modelNum >= 0 && modelNum < 2);
    loadedModelNames[modelNum] = modelName;
  }

//#define TEST_AABB
#ifndef TEST_AABB
  // make sure the models are at the origin first
  BasicState modelState1;
  modelState1.SetPosition(0,0,0);
  modelState1.SetRotation(0,0,0,1);
  BasicState modelState2(modelState1);
  if ((worldManager->SetBasicModelState(loadedModelNames[0], modelState1)
       != worldManager->GetNumWorlds()) ||
      (worldManager->SetBasicModelState(loadedModelNames[1], modelState2)
       != worldManager->GetNumWorlds()))
  {
    std::cerr << "Could not set all model poses to origin" << std::endl;
    return false;
  }
#else
  BasicState modelState1, modelState2;
  // get the states of the models as loaded in their original pose
  if (GetBasicModelState(loadedModelNames[0], worldManager, modelState1) != 0)
  {
    std::cerr << "Could not get BasicModelState." << std::endl;
    return false;
  }
  if (GetBasicModelState(loadedModelNames[1], worldManager, modelState2) != 0)
  {
    std::cerr << "Could not get BasicModelState." << std::endl;
    return false;
  }
#endif

  if (!modelState1.PosEnabled() ||
      !modelState2.PosEnabled())
  {
    std::cerr << "Models are expected to have a position" << std::endl;
    return false;
  }


  // position models so they're not intersecting.
  ///////////////////////////////

  // First, get the AABB's of the two models.
  Vector3 min1, min2, max1, max2;
  bool local1, local2;
  if ((GetAABB(loadedModelNames[0], worldManager, min1, max1, local1) != 0) ||
      (GetAABB(loadedModelNames[1], worldManager, min2, max2, local2) != 0))
  {
    std::cerr << "Could not get AABBs of models" << std::endl;
    return false;
  }

#ifdef TEST_AABB
  // The following is only needed if objects are not at origin
  // to start with (because then global frame = local frame)

  // If the ABBs are not given in global coordinate frame, need to transform
  // the AABBs first.
  if (local1)
  {
    ignition::math::Matrix4d trans =
      collision_benchmark::GetMatrix<double>(modelState1.position,
                                             modelState1.rotation);
    ignition::math::Vector3d newMin, newMax;
    ignition::math::Vector3d ignMin(collision_benchmark::ConvIgn<double>(min1));
    ignition::math::Vector3d ignMax(collision_benchmark::ConvIgn<double>(max1));
    collision_benchmark::UpdateAABB(ignMin, ignMax, trans, newMin, newMax);
    min1 = newMin;
    max1 = newMax;
  }
  if (local2)
  {
    ignition::math::Matrix4d trans =
      collision_benchmark::GetMatrix<double>(modelState2.position,
                                             modelState2.rotation);
    ignition::math::Vector3d newMin, newMax;
    ignition::math::Vector3d ignMin(collision_benchmark::ConvIgn<double>(min2));
    ignition::math::Vector3d ignMax(collision_benchmark::ConvIgn<double>(max2));
    collision_benchmark::UpdateAABB(ignMin, ignMax, trans, newMin, newMax);
    min2 = newMin;
    max2 = newMax;
  }
#endif

  // std::cout << "AABB 1: " << min1 << " -- " << max1 << std::endl;
  // std::cout << "AABB 2: " << min2 << " -- " << max2 << std::endl;

  // Leave model 1 where it is and move model 2 away from it.
  const float aabb1LenOnAxis = (max1-min1).Dot(collisionAxis);
  const float aabb2LenOnAxis = (max2-min2).Dot(collisionAxis);

  // desired distance between models is \e distFact of the
  // larger AABBs on collisionAxis
  double distFact = 0.2;
  double desiredDistance = std::max(aabb1LenOnAxis*distFact,
                                    aabb2LenOnAxis*distFact);

  // distance between both AABBs in their orignal pose
  double dist = min2.Dot(collisionAxis) - max1.Dot(collisionAxis);
  double moveDistance = desiredDistance - dist;
  // std::cout << "Move model 2 along axis "
  //          << collisionAxis*moveDistance << std::endl;

  Vector3 moveAlongAxis = collisionAxis * moveDistance;
  // std::cout << "State of model 2: " << modelState2 << std::endl;
  collision_benchmark::Vector3 newModelPos2 = modelState2.position;
  newModelPos2.x += moveAlongAxis.X();
  newModelPos2.y += moveAlongAxis.Y();
  newModelPos2.z += moveAlongAxis.Z();
  modelState2.SetPosition(newModelPos2);
  worldManager->SetBasicModelState(loadedModelNames[1], modelState2);

  // set parameters of collision bar: starting at origin of Model 1,
  // along the chosen axis, and ending at origin of Model 2.
  ///////////////////////////////

  double cylLength = desiredDistance + aabb1LenOnAxis / 2.0f
                    + aabb2LenOnAxis / 2.0f;

  ignition::math::Vector3d modelPos1(modelState1.position.x,
                                     modelState1.position.y,
                                     modelState1.position.z);
  ignition::math::Vector3d collBarP
    = modelPos1 +
      ignition::math::Vector3d(collisionAxis.X(), collisionAxis.Y(),
                               collisionAxis.Z()) * cylLength/2;


  const Vector3 zAxis(0,0,1); // default axis of cylinder
  ignition::math::Quaterniond collBarQ;
  collBarQ.From2Axes(zAxis, collisionAxis);
  ignition::math::Pose3d collBarPose(collBarP, collBarQ);

  // start the thread to handle the collision bar
  ///////////////////////////////
  running = true;
  double cylRadius = 0.02;
  std::thread t(std::bind(&CollidingShapesTestFramework::CollisionBarHandler,
                          this, std::placeholders::_1, std::placeholders::_2,
                          std::placeholders::_3, std::placeholders::_4),
                          collBarPose, cylRadius, cylLength,
                          gzMultiWorld->GetMirrorName());


  // Subscribe to the GUI control
  ///////////////////////////////
  std::string controlTopic="collide_shapes_test/control";
  gazebo::transport::NodePtr node
    = gazebo::transport::NodePtr(new gazebo::transport::Node());
  node->Init();
  gazebo::transport::SubscriberPtr controlSub =
    node->Subscribe(controlTopic,
                    &CollidingShapesTestFramework::receiveControlMsg,
                    this);

  std::string feedbackTopic="collide_shapes_test/feedback";
  gazebo::transport::PublisherPtr feedbackPub =
    node->Advertise<gazebo::msgs::Any>(feedbackTopic);

  // Run the world(s)
  ///////////////////////////////
  bool waitForStart = false;
  gzMultiWorld->Run(waitForStart, false);

  std::cout << "CollidingShapesTestFramework: Wait until the simulation should "
            << "be started..." << std::endl;
  // Sleep while the start signal has not been triggered yet
  while (!gzMultiWorld->HasStarted())
    gazebo::common::Time::MSleep(100);
  std::cout << "CollidingShapesTestFramework: ... now starting simulation."
            << std::endl;

  double sliderStepSize = (cylLength / CollidingShapesParams::MaxSliderVal);
  // Variable holding how much the shapes were previously moved at
  // along the axis already via the controls.
  int shapesOnAxisPrev = CollidingShapesParams::MaxSliderVal;

  // run the main loop
  while (gzMultiWorld->IsClientRunning())
  {
      // while collision is not found, move models towards each other
      bool allWorlds = false;
      bool moveBoth = false;
      if (triggeredAutoCollide)
      {
        double dist = AutoCollide(allWorlds, moveBoth);
        int unitsMoved = dist / sliderStepSize;
        /* std::cout << "Units moved during auto collide: "
                  << unitsMoved << ". Current value is "
                  << shapesOnAxisPrev << std::endl; */
        shapesOnAxisPrev -= unitsMoved;
        gazebo::msgs::Any m;
        m.set_type(gazebo::msgs::Any::INT32);
        m.set_int_value(-unitsMoved);
        feedbackPub->Publish(m);
        triggeredAutoCollide = false;
      }
      if (shapesOnAxisPos != shapesOnAxisPrev)
      {
        /* std::cout << "Slider action: Moving shapes by "
                  << (shapesOnAxisPrev - shapesOnAxisPos)
                  << " steps." << std::endl;*/
        MoveModelsAlongAxis((shapesOnAxisPrev - shapesOnAxisPos)
                            * sliderStepSize, moveBoth);
        shapesOnAxisPrev = shapesOnAxisPos;
      }
      if (!triggeredSaveConfig.empty())
      {
        std::lock_guard<std::mutex> lock(triggeredSaveConfigMtx);
        std::cout << "SAVE! " << triggeredSaveConfig << std::endl;
        triggeredSaveConfig = "";
      }

      int numSteps = 1;
      worldManager->Update(numSteps);
  }

  std::cout << "CollidingShapesTestFramework: Client closed, "
            << "stopping simulation." << std::endl;
  gzMultiWorld->ShutdownServer();

  // end the thread to handle the collision bar
  running = false;
  //std::cout << "Joining thread" << std::endl;
  t.join();

  // std::cout << "Finished running CollidingShapesTestFramework." << std::endl;
  return true;
}


/////////////////////////////////////////////////
void CollidingShapesTestFramework::receiveControlMsg(ConstAnyPtr &_msg)
{
  // std::cout << "Any msg: "<<_msg->DebugString();
  switch (_msg->type())
  {
    case gazebo::msgs::Any::BOOLEAN:
      {
        std::cout <<"Triggered auto collide." << std::endl;
        triggeredAutoCollide = true;
        break;
      }
    case gazebo::msgs::Any::INT32:
      {
        // std::cout <<"Moving shapes to " << _msg->int_value() << std::endl;
        shapesOnAxisPos = _msg->int_value();
        if (shapesOnAxisPos > CollidingShapesParams::MaxSliderVal)
          shapesOnAxisPos = CollidingShapesParams::MaxSliderVal;
        if (shapesOnAxisPos < 0) shapesOnAxisPos = 0;
        break;
      }
    case gazebo::msgs::Any::STRING:
      {
        std::lock_guard<std::mutex> lock(triggeredSaveConfigMtx);
        triggeredSaveConfig = _msg->string_value();
        /* std::cout << "Saving configuration as "
                  << triggeredSaveConfig << std::endl;*/
        break;
      }


    default:
      std::cerr << "Unsupported AnyMsg type" << std::endl;
  }
}


//////////////////////////////////////////////////////////////////////////////
double CollidingShapesTestFramework::AutoCollide(bool allWorlds,
                                               bool moveBoth)
{
  GzWorldManager::Ptr worldManager = gzMultiWorld->GetWorldManager();
  assert(worldManager);
  // to allow for a certain animation effect, move the shapes at a maximum
  // distance per second.
  // There cannot be a minimum velocity because we have to make tiny
  // movements in order to capture the first point of collision as exact
  // as possible.
  const float maxMovePerSec = 0.4;
  double moved = 0;
  gazebo::common::Timer timer;
  timer.Start();
  // while collision is not found, move models towards each other
  while (!ModelsCollide(allWorlds) && gzMultiWorld->IsClientRunning())
  {
    gazebo::common::Time elapsed = timer.GetElapsed();
    // move the shapes towards each other in steps of this size
    const double stepSize = 1e-03;
    if (moved > 0)
    {
      // slow down the movement if it's too fast.
      // Not the most accurate way to achieve a maximum velocity,
      // but considering this is only for animation purposes, this will do.
      if (moved / elapsed.Double() > maxMovePerSec)
      {
        // slow down the move as we've already moved too far.
        // Sleep a tiny bit.
        gazebo::common::Time::MSleep(10);
        continue;
      }
    }
    MoveModelsAlongAxis(stepSize, moveBoth);
    moved += stepSize;
    int numSteps = 1;
    worldManager->Update(numSteps);
  }
  return moved;
}

//////////////////////////////////////////////////////////////////////////////
bool CollidingShapesTestFramework::ModelsCollide(bool allWorlds)
{
  GzWorldManager::Ptr worldManager = gzMultiWorld->GetWorldManager();
  assert(worldManager);

  std::vector<GzWorldManager::PhysicsWorldContactInterfacePtr>
    contactWorlds = worldManager->GetContactPhysicsWorlds();

  assert(contactWorlds.size() == worldManager->GetNumWorlds());

  int modelsColliding = 0;
  for (std::vector<GzWorldManager::PhysicsWorldContactInterfacePtr>::iterator
       it = contactWorlds.begin(); it != contactWorlds.end(); ++it)
  {
    GzWorldManager::PhysicsWorldContactInterfacePtr world = *it;
    std::vector<GzWorldManager::PhysicsWorldContactInterfaceT::ContactInfoPtr>
      contacts = world->GetContactInfo();
    if (!contacts.empty())
    {
      ++modelsColliding;
    }
  }

  // while collision is not found, move models towards each other
  if (allWorlds)
    // all worlds have to collide
    return modelsColliding == contactWorlds.size();

  // only one world has to collide
  return modelsColliding > 0;
}


//////////////////////////////////////////////////////////////////////////////
void CollidingShapesTestFramework::MoveModelsAlongAxis(const float moveDist,
                                                       const bool moveBoth)
{
  GzWorldManager::Ptr worldManager = gzMultiWorld->GetWorldManager();
  assert(worldManager);
  // get state of both models
  BasicState modelState1, modelState2;
  // get the states of the models as loaded in their original pose
  if (moveBoth &&
      (GetBasicModelState(loadedModelNames[0], worldManager, modelState1) != 0))
  {
    std::cerr << "Could not get BasicModelState." << std::endl;
    return;
  }
  if (GetBasicModelState(loadedModelNames[1], worldManager, modelState2) != 0)
  {
    std::cerr << "Could not get BasicModelState." << std::endl;
    return;
  }
  const ignition::math::Vector3d mv = collisionAxis * moveDist;

  if (moveBoth)
  {
    modelState1.SetPosition(modelState1.position.x + mv.X(),
                            modelState1.position.y + mv.Y(),
                            modelState1.position.z + mv.Z());
  }

  modelState2.SetPosition(modelState2.position.x - mv.X(),
                          modelState2.position.y - mv.Y(),
                          modelState2.position.z - mv.Z());

  if ((moveBoth &&
       (worldManager->SetBasicModelState(loadedModelNames[0], modelState1)
        != worldManager->GetNumWorlds())) ||
      (worldManager->SetBasicModelState(loadedModelNames[1], modelState2)
       != worldManager->GetNumWorlds()))
  {
    std::cerr << "Could not set all model poses to origin" << std::endl;
    return;
  }
}

//////////////////////////////////////////////////////////////////////////////
int CollidingShapesTestFramework::GetAABB(const std::string& modelName,
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


//////////////////////////////////////////////////////////////////////////////
int CollidingShapesTestFramework::GetBasicModelState(const std::string& modelName,
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


/////////////////////////////////////////////////////////////////////////////
void CollidingShapesTestFramework::CollisionBarHandler
        (const ignition::math::Pose3d& collBarPose,
         const float cylinderRadius,
         const float cylinderLength,
         const std::string& mirrorName)
{
  // make a model publisher which will publish the bar to gzclient
  gazebo::transport::NodePtr node =
    gazebo::transport::NodePtr(new gazebo::transport::Node());
  node->Init();
  gazebo::transport::PublisherPtr modelPub =
    node->Advertise<gazebo::msgs::Model>("~/model/info");

  std::cout << "CollidingShapesTestFramework::CollisionBarHandler: "
            << "Waiting for gzclient model subscriber..." << std::endl;
  while (!modelPub->HasConnections())
  {
    gazebo::common::Time::MSleep(500);
  }
  std::cout << "CollidingShapesTestFramework::CollisionBarHandler: "
            << "... gzclient connected to models." << std::endl;

  gazebo::transport::PublisherPtr posePub =
    node->Advertise<gazebo::msgs::PosesStamped>("~/pose/info");
  std::cout << "CollidingShapesTestFramework::CollisionBarHandler: "
            << "Waiting for gzclient pose subscriber..." << std::endl;
  while (!posePub->HasConnections())
  {
    gazebo::common::Time::MSleep(500);
  }
  std::cout << "CollidingShapesTestFramework::CollisionBarHandler: "
            << "...gzclient connected to pose." << std::endl;

  // visual ID to use for the bar. This will be the ID
  // which we can use to set (and keep enforcing) the position of
  // the bar in gzclient, so it cannot be moved with the move tools in
  // gzclient.
  int visualID = std::numeric_limits<uint32_t>::max()-1;
  // create the model message to add the collision bar model
  gazebo::msgs::Model modelMsg;
  modelMsg.set_name("collision_bar");
  modelMsg.set_id(visualID);
  modelMsg.set_is_static(true);
  gazebo::msgs::Set(modelMsg.mutable_pose(), collBarPose);

  // create visual
  collision_benchmark::Shape::Ptr shape
    (collision_benchmark::PrimitiveShape::CreateCylinder(cylinderRadius,
                                                         cylinderLength));
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
  gazebo::msgs::Material * mat = visualMsg.mutable_material();
  gazebo::msgs::Color * colDiffuse = mat->mutable_diffuse();
  colDiffuse->set_r(1);
  colDiffuse->set_g(0.5);
  colDiffuse->set_b(0.5);
  colDiffuse->set_a(0.5);
  gazebo::msgs::Color * colAmbient = mat->mutable_ambient();
  colAmbient->set_r(1);
  colAmbient->set_g(0.5);
  colAmbient->set_b(0.5);
  colAmbient->set_a(0.5);
/*  gazebo::msgs::Color * colSpecular = mat->mutable_specular();
  colSpecular->set_r(1);
  colSpecular->set_g(0);
  colSpecular->set_b(0);
  colSpecular->set_a(0.5);
  gazebo::msgs::Color * colEmissive = mat->mutable_emissive();
  colEmissive->set_r(1);
  colEmissive->set_g(0);
  colEmissive->set_b(0);
  colEmissive->set_a(0.5);*/
  // transparency doesn't really work unfortunately...
  // XXX TODO figure out how to do this
  visualMsg.set_transparency(0.5);
  visualMsg.set_cast_shadows(false);

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
    gazebo::msgs::PosesStamped poseMsg;
    gazebo::msgs::Set(poseMsg.mutable_time(), 0);
    gazebo::msgs::Pose * singlePoseMsg = poseMsg.add_pose();

    singlePoseMsg->set_name("collision_bar");
    singlePoseMsg->set_id(visualID);

    gazebo::msgs::Set(singlePoseMsg, collBarPose);
    posePub->Publish(poseMsg);
    gazebo::common::Time::MSleep(100);
  }
  // std::cout << "Stopping to publish collision bar." << std::endl;
}


