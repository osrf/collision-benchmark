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
#include <collision_benchmark/PrimitiveShape.hh>
#include <collision_benchmark/GazeboModelLoader.hh>
#include <collision_benchmark/GazeboWorldLoader.hh>
#include <collision_benchmark/BasicTypes.hh>
#include <collision_benchmark/MathHelpers.hh>

#include <gazebo/common/Timer.hh>

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>

#include <fstream>

#include "CollidingShapesTestFramework.hh"
#include "CollidingShapesParams.hh"
#include "BoostSerialization.hh"

using collision_benchmark::test::CollidingShapesTestFramework;
using collision_benchmark::test::CollidingShapesParams;
using collision_benchmark::test::CollidingShapesConfiguration;

using collision_benchmark::GazeboMultipleWorlds;
using collision_benchmark::GazeboModelLoader;
using collision_benchmark::Shape;
using collision_benchmark::PrimitiveShape;
using collision_benchmark::GazeboMultipleWorlds;
using collision_benchmark::BasicState;

/////////////////////////////////////////////////////////////////////////////
CollidingShapesTestFramework::CollidingShapesTestFramework()
  : collisionAxis(0, 1, 0),
    triggeredAutoCollide(false),
    shapesOnAxisPos(CollidingShapesParams::MaxSliderVal)
{
}

/////////////////////////////////////////////////////////////////////////////
bool CollidingShapesTestFramework::Run
    (const std::vector<std::string>& physicsEngines,
     const std::vector<std::string>& unitShapes,
     const std::vector<std::string>& sdfModels,
     const float modelsGap,
     const bool modelsGapIsFactor)
{
  if (unitShapes.size() + sdfModels.size() != 2)
  {
    std::cerr << "Have to specify exactly two shapes or models" << std::endl;
    std::cerr << "Specified shapes: " << std::endl;
    for (std::vector<std::string>::const_iterator it = unitShapes.begin();
         it != unitShapes.end(); ++it)
      std::cout << " " << *it << std::endl;
    std::cerr << "Models: " << std::endl;
    for (std::vector<std::string>::const_iterator it = sdfModels.begin();
         it != sdfModels.end(); ++it)
      std::cout << " " << *it << std::endl;
    std::cout << "which makes a total of " << (unitShapes.size() +
                 sdfModels.size()) << " models. " << std::endl;
    return false;
  }

  // create configuration adn add the model references
  configuration.reset(new CollidingShapesConfiguration(sdfModels, unitShapes));
  return RunImpl(physicsEngines, modelsGap, modelsGapIsFactor);
}


/////////////////////////////////////////////////////////////////////////////
bool CollidingShapesTestFramework::Run
    (const std::vector<std::string>& physicsEngines,
     const std::string configFile,
     const float modelsGap,
     const bool modelsGapIsFactor)
{
  std::ifstream ifs(configFile);
  if (!ifs.is_open())
  {
    std::cerr << "Cannot read configFile " << configFile << std::endl;
    return false;
  }
  CollidingShapesConfiguration readConf;
  boost::archive::text_iarchive ia(ifs);
  ia >> readConf;
  // archive and stream are closed when destructors are called

  std::cout << "Model states to load: " << std::endl
            << readConf.modelState1 << std::endl
            << readConf.modelState2 << std::endl;

  // create configuration adn add the model references
  configuration.reset(new CollidingShapesConfiguration(readConf));
  return RunImpl(physicsEngines, modelsGap, modelsGapIsFactor);
}

/////////////////////////////////////////////////////////////////////////////
bool CollidingShapesTestFramework::RunImpl
    (const std::vector<std::string>& physicsEngines,
     const float modelsGap,
     const bool modelsGapIsFactor)
{
  assert(configuration);
  assert(configuration->models.size() + configuration->shapes.size() == 2);

  // Initialize server
  ///////////////////////////////

  // we need to load the mirror world in order to use gzclient.
  bool loadMirror = true;
  // important to be true so contacts are calculated for all worlds,
  // not only the displayed one!
  bool enforceContactCalc = true;
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
  /////////////////////////////////////////

  GzWorldManager::Ptr worldManager = gzMultiWorld->GetWorldManager();
  assert(worldManager);

  // load primitive shapes
  typedef GzWorldManager::ModelLoadResult ModelLoadResult;
  int modelNum = 0;
  for (std::vector<std::string>::const_iterator
       it = configuration->shapes.begin();
       it != configuration->shapes.end(); ++it, ++modelNum)
  {
    const std::string &shapeID = *it;
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
    this->loadedModelNames[modelNum] = modelName;
  }

  // load models from SDF
  for (std::vector<std::string>::const_iterator
       it = configuration->models.begin();
       it != configuration->models.end(); ++it, ++modelNum)
  {
    const std::string &modelResource = *it;
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
    this->loadedModelNames[modelNum] = modelName;
  }

  // initialize the model collider helper
  if (!this->modelCollider.Init(worldManager, this->collisionAxis,
                                this->loadedModelNames[0],
                                this->loadedModelNames[1]))
  {
    std::cerr << "Could not initialize model collider" << std::endl;
    return false;
  }

  BasicState modelState1, modelState2;
  // place models into their default position using this helper
  if (!this->modelCollider.PlaceModels(modelsGap, modelsGapIsFactor,
                                 modelState1, modelState2))
  {
    std::cerr << "Could not place models in initial pose" << std::endl;
    return false;
  }

  // Set the collision bar: starting at origin of Model 1,
  // along the chosen axis, and ending at origin of Model 2.
  ///////////////////////////////

  // length of the collision axis (which is visibly going to be a cylinder)
  ignition::math::Vector3d modelPos1(modelState1.position.x,
                                     modelState1.position.y,
                                     modelState1.position.z);
  ignition::math::Vector3d modelPos2(modelState2.position.x,
                                     modelState2.position.y,
                                     modelState2.position.z);

  double collAxisLength = (modelPos1 - modelPos2).Length();

  ignition::math::Vector3d collBarP = modelPos1 +
      ignition::math::Vector3d(collisionAxis.X(), this->collisionAxis.Y(),
                               this->collisionAxis.Z()) * collAxisLength/2;
  // axis of cylinder
  const Vector3 zAxis(0, 0, 1);
  ignition::math::Quaterniond collBarQ;
  collBarQ.From2Axes(zAxis, this->collisionAxis);
  ignition::math::Pose3d collBarPose(collBarP, collBarQ);

  // start the thread to handle the collision bar
  running = true;
  double cylRadius = 0.02;
  std::thread t(std::bind(&CollidingShapesTestFramework::CollisionBarHandler,
                          this, std::placeholders::_1, std::placeholders::_2,
                          std::placeholders::_3, std::placeholders::_4),
                          collBarPose, cylRadius, collAxisLength,
                          gzMultiWorld->GetMirrorName());


  // Consider any model poses which
  // have been loaded from configuration
  //////////////////////////////////////

  // If the configuration has a valid pose, we need move the models
  // according to it. This will be done if the configuration has been
  // loaded from a file.
  // The pose given in the configuration file is relative to the
  // pose the model has when it has been placed at the origin.
  if (configuration->modelState1.PosEnabled()
      || configuration->modelState1.RotEnabled())
  {
    ignition::math::Matrix4d poseCurr =
      collision_benchmark::GetMatrix<double>(modelState1.position,
                                             modelState1.rotation);
    ignition::math::Matrix4d poseConfig =
      collision_benchmark::GetMatrix<double>
        (configuration->modelState1.position,
         configuration->modelState1.rotation);

    ignition::math::Matrix4d transformedPose = poseCurr * poseConfig;
    collision_benchmark::Vector3 newPos
      (collision_benchmark::Conv(transformedPose.Translation()));
    collision_benchmark::Quaternion newRot
      (collision_benchmark::Conv(transformedPose.Rotation()));
    modelState1.position = newPos;
    modelState1.rotation = newRot;
    worldManager->SetBasicModelState(loadedModelNames[0], modelState1);
  }

  if (configuration->modelState2.PosEnabled()
      || configuration->modelState2.RotEnabled())
  {
    ignition::math::Matrix4d poseCurr =
      collision_benchmark::GetMatrix<double>(modelState2.position,
                                             modelState2.rotation);
    ignition::math::Matrix4d poseConfig =
      collision_benchmark::GetMatrix<double>
        (configuration->modelState2.position,
         configuration->modelState2.rotation);

    ignition::math::Matrix4d transformedPose = poseCurr * poseConfig;
    collision_benchmark::Vector3 newPos
      (collision_benchmark::Conv(transformedPose.Translation()));
    collision_benchmark::Quaternion newRot
      (collision_benchmark::Conv(transformedPose.Rotation()));
    modelState2.position = newPos;
    modelState2.rotation = newRot;
    worldManager->SetBasicModelState(loadedModelNames[1], modelState2);
  }

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

  // step size along the collision bar, based on
  // step sizes of the slider in the GUI and the length of the collision axis.
  const double sliderStepSize
    = (collAxisLength / CollidingShapesParams::MaxSliderVal);

  // Variable holding how much the shapes were moved at
  // along the axis already via the controls.
  int shapesOnAxisPrev = CollidingShapesParams::MaxSliderVal;

  // In a variable, remember the value which model 2 was moved by either
  // AutoCollide or by the sliding axis. This value will not be considered when
  // saving the configuration.
  float model2MovedAlongAxis = 0;

  // run the main loop
  while (gzMultiWorld->IsClientRunning())
  {
      // while collision is not found, move models towards each other
      const bool allWorlds = false;
      // set to true to move both models towards/away from each other.
      // Note: At this point, the folliwng implementation assumes it is
      // false. Adjust implementation if a value of true is desired later.
      const bool moveBoth = false;
      if (triggeredAutoCollide)
      {
        // Flag: stop auto-collide if object center projections on axis have
        // passed each other and it is likely the objects will never collide
        // along this axis.
        const bool stopWhenPassed = true;
        // maximum move distance per second
        const double maxMovePerSec = 0.4;
        // move models at this step size
        const double stepSize = 1e-03;
        // auto-collide models
        const double dist =
          this->modelCollider.AutoCollide(allWorlds, moveBoth, stepSize,
                                          maxMovePerSec, stopWhenPassed);
        model2MovedAlongAxis += -dist;
        const int unitsMoved = dist / sliderStepSize;
        /* std::cout << "Units moved during auto collide: "
                  << unitsMoved << ". Current value is "
                  << shapesOnAxisPrev << std::endl;*/
        std::lock_guard<std::mutex> lock(shapesOnAxisPosMtx);
        shapesOnAxisPrev -= unitsMoved;
        // enforce the slider to go onto the same position
        shapesOnAxisPos = shapesOnAxisPrev;
        gazebo::msgs::Any m;
        m.set_type(gazebo::msgs::Any::INT32);
        m.set_int_value(shapesOnAxisPos);
        feedbackPub->Publish(m);
        // unset trigger
        triggeredAutoCollide = false;
      }
      { // lock scope
        std::lock_guard<std::mutex> lock(shapesOnAxisPosMtx);
        if (shapesOnAxisPos != shapesOnAxisPrev)
        {
          float moveDist = (shapesOnAxisPrev - shapesOnAxisPos)*sliderStepSize;
          this->modelCollider.MoveModelsAlongAxis(moveDist, moveBoth);
          model2MovedAlongAxis += -moveDist;
          shapesOnAxisPrev = shapesOnAxisPos;
        }
      }
      {  // lock scope
        std::lock_guard<std::mutex> lock(triggeredSaveConfigMtx);
        if (!triggeredSaveConfig.empty())
        {
          // This will save the model poses relative to their starting pose
          // which have been changed by the user. The amount the models were
          // slid towards or away from each other by means of AutoCollide or
          // using the GUI slider will not count. So when the configuration is
          // loaded again, the models will still start at each end of the
          // collision axis, and when they are collided, the same collision
          // configuration as saved will be achieved by AutoCollide
          // and/or the slider.

          std::lock_guard<std::mutex> lock(shapesOnAxisPosMtx);
          // Amount that model 2 has been moved along the collision axis
          // with the slider or AutoCollide - this will not count for the
          // configuration.
          // (NOTE: presuming moveBoth is false, otherwise we also have
          // to pass something for model 1)
          double model1Slide = 0;
          // model 2 has moved as we displaced it in
          // ModelCollider::PlaceModels(), and the moves via the shapes axis.
          double model2Slide = /*-moveM2Distance*/ -model2MovedAlongAxis;

          // get current state
          BasicState currModelState1, currModelState2;
          if (GetModelStates(currModelState1, currModelState2))
          {

            // revert the sliding that has been done during runtime
            RevertSlide(model1Slide, model2Slide,
                        currModelState1, currModelState2);

            // get the difference in pose the user has done during runtime
            // compared to the models initial poses
            BasicState m1Diff = GetTrans(modelState1, currModelState1);
            BasicState m2Diff = GetTrans(modelState2, currModelState2);
            SaveConfiguration(triggeredSaveConfig, m1Diff, m2Diff);
          }
          else
          {
            std::cerr << "Could not get model states, cannot save "
                      << "configuration." << std::endl;
          }
          triggeredSaveConfig = "";
        }
      }
      int numSteps = 1;
      worldManager->Update(numSteps);
  }

  std::cout << "CollidingShapesTestFramework: Client closed, "
            << "stopping simulation." << std::endl;
  gzMultiWorld->ShutdownServer();

  // end the thread to handle the collision bar
  running = false;
  t.join();
  // std::cout << "Finished running CollidingShapesTestFramework." << std::endl;
  return true;
}

/////////////////////////////////////////////////
void CollidingShapesTestFramework::RevertSlide(const double model1Slide,
                                               const double model2Slide,
                                               BasicState &modelState1,
                                               BasicState &modelState2) const
{
  // the movement that model 1 has been moved via the collision axis
  const ignition::math::Vector3d mv1 = this->collisionAxis * model1Slide;
  // the movement that model 2 has been moved via the collision axis
  const ignition::math::Vector3d mv2 = this->collisionAxis * model2Slide;

  // undo the move via the collision axis
  modelState1.SetPosition(modelState1.position.x + mv1.X(),
                          modelState1.position.y + mv1.Y(),
                          modelState1.position.z + mv1.Z());

  modelState2.SetPosition(modelState2.position.x + mv2.X(),
                          modelState2.position.y + mv2.Y(),
                          modelState2.position.z + mv2.Z());
}


/////////////////////////////////////////////////
BasicState CollidingShapesTestFramework::GetTrans(const BasicState &from,
                                                  const BasicState &to) const
{
  BasicState ret;
  if (!from.PosEnabled())
  {
    if (to.PosEnabled()) ret.SetPosition(to.position);
  }
  else if (!to.PosEnabled())
  {
    if (from.PosEnabled()) ret.SetPosition(from.position);
  }
  else
  {
    ret.SetPosition(to.position.x - from.position.x,
                    to.position.y - from.position.y,
                    to.position.z - from.position.z);
  }

  if (!from.RotEnabled())
  {
    if (to.RotEnabled()) ret.SetRotation(to.rotation);
  }
  else if (!to.RotEnabled())
  {
    if (from.RotEnabled()) ret.SetRotation(from.rotation);
  }
  else
  {
    ignition::math::Quaterniond
      fromQ(from.rotation.w, from.rotation.x,
            from.rotation.y, from.rotation.z);
    ignition::math::Quaterniond
      toQ(to.rotation.w, to.rotation.x,
          to.rotation.y, to.rotation.z);
    ignition::math::Quaterniond diff = fromQ.Inverse() * toQ;
    ret.SetRotation(diff.X(), diff.Y(), diff.Z(), diff.W());
  }
  return ret;
}

/////////////////////////////////////////////////
bool CollidingShapesTestFramework::GetModelStates(BasicState &modelState1,
                                                  BasicState &modelState2) const
{
  GzWorldManager::Ptr worldManager = gzMultiWorld->GetWorldManager();
  assert(worldManager);
  if (ModelColliderT::GetBasicModelState(loadedModelNames[0], 0,
                                         worldManager, modelState1) != 0)
  {
    std::cerr << "Could not get BasicModelState." << std::endl;
    return false;
  }
  if (ModelColliderT::GetBasicModelState(loadedModelNames[1], 0,
                                         worldManager, modelState2) != 0)
  {
    std::cerr << "Could not get BasicModelState." << std::endl;
    return false;
  }
  return true;
}

/////////////////////////////////////////////////
void CollidingShapesTestFramework::SaveConfiguration(const std::string &file,
                                        const BasicState& modelState1,
                                        const BasicState& modelState2) const
{
  std::ofstream ofs(file);
  if (!ofs.is_open())
  {
    std::cerr << "Cannot write to file " << file << std::endl;
    return;
  }

  this->configuration->modelState1 = modelState1;
  this->configuration->modelState2 = modelState2;
  CollidingShapesConfiguration::Ptr
    writeConf(new CollidingShapesConfiguration(*this->configuration));

  if (!writeConf)
  {
    std::cerr << "Could not get configuration. " << std::endl;
    return;
  }
  /*std::cout << "Model states to save (slides: "
            << model1Slide << ", " << model2Slide << "): " << std::endl
            << writeConf->modelState1 << std::endl
            << writeConf->modelState2 << std::endl;
  */
  std::cout << "Saving configuration to " << file << std::endl;
  boost::archive::text_oarchive oa(ofs);
  oa << *writeConf;
  // archive and stream are closed when destructors are called
}

/////////////////////////////////////////////////////////////////////////////
void CollidingShapesTestFramework::CollisionBarHandler
        (const ignition::math::Pose3d &collBarPose,
         const float cylinderRadius,
         const float cylinderLength,
         const std::string &mirrorName)
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
  sdf::ElementPtr shapeGeom = shape->GetShapeSDF();
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

/////////////////////////////////////////////////
void CollidingShapesTestFramework::receiveControlMsg(ConstAnyPtr &_msg)
{
  // std::cout << "Any msg: " << _msg->DebugString();
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
        std::lock_guard<std::mutex> lock(shapesOnAxisPosMtx);
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
