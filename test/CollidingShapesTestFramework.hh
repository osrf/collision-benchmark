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
#ifndef COLLISION_BENCHMARK_TEST_COLLIDINGSHAPESFRAMEWORK_H
#define COLLISION_BENCHMARK_TEST_COLLIDINGSHAPESFRAMEWORK_H

#include <test/CollidingShapesConfiguration.hh>
#include <collision_benchmark/GazeboMultipleWorlds.hh>

#include <thread>
#include <mutex>
#include <atomic>
#include <string>
#include <vector>
#include <unistd.h>
#include <sys/wait.h>

using collision_benchmark::GazeboMultipleWorlds;

namespace collision_benchmark
{
namespace test
{
/**
 * \brief Test framework for colliding two shapes along an axis.
 *
 * Uses collision_benchmark::GazeboMultipleWorlds to load up a Gazebo
 * server and client with multiple physics engines, and loads two shapes
 * into the world(s), one at each end of a "collision bar".

 * Two shapes are loaded, one at each end of a "collision bar". The collision
 * bar is the axis along which the objects will be moved to test the collision.
 * A slider can be used to step-wise move the objects towards and apart from
 * each other along the collision axis.
 * An "Auto-collide" function will move the objects along the collision
 * bar until at least one of the physics engines used for testing reports a
 * collision between the objects.
 *
 * The models' pose relative to the collision axis can be changed in order to
 * find critical collision configurations which happen when the models collide.
 * To change the pose of the models, the Gazebo client transformation tools
 * can be used.
 *
 * A configuration can then be saved an loaded again at a later point.
 * The saved configuration will not save the amount the models were
 * moved via the axis slider or the auto-collide function, but only the amount
 * they were moved around by the user (with Gazebo client transformation tools).
 * Before saving a configuration, it is advised to separate the models to the
 * maximum extent using the slider, to get a feeling of which configuration
 * will be saved. The configuration saved is the one at which models are
 * separated to the full extent via the collision axis slider.
 *
 * To load models, the test can use:
 * - SDF files which contain the model
 * - The name of the model (it has to be findable in ``GAZEBO_MODEL_PATH``)
 * - Unit shapes: Box, cylinder and sphere
 * However, only exactly two shapes are supported, regardless what type.
 *
 * Note that the collision axis cannot be moved. It is not a model in the world,
 * it is only displayed as a helping visualization.
 *
 * More details about how the implementation unfolds the test configuration:
 *
 * 1. The two models are loaded and placed *at the origin* (which means
 *    poses which they may have in the SDF files are ignored).
 * 2. The two models are separated along the axis, such that their AABBs do not
 *    intersect. Model 2 is moved away from model 1 along the collision axis.
 * 3. The slider or the "Auto Collide" function can be used to slide
 *    model 2 along the axis towards/away from model 1.
 * 4. The models' pose can be changed and also saved to the configuration.
 *
 * \author Jennifer Buehler
 * \date May 2017
 */
class CollidingShapesTestFramework
{
  private: typedef GazeboMultipleWorlds::GzWorldManager GzWorldManager;
  private: typedef GzWorldManager
            ::PhysicsWorldModelInterfaceT::Vector3 Vector3;

  public: CollidingShapesTestFramework();

  // \brief Runs the test framework with the physics engines \e physicsEngines
  // and the unit shapes \e unitShapes and SDF models \e sdfModels. There
  // may only be two models altogether specified in \e unitShapes and
  // \e sdfModels. The two models will be the ones which are tested for
  // collision with each other.
  //
  // This is a blocking call. It returns when the test window (gzclient) was
  // closed.
  //
  // \param[in] physicsEngines list of physics engines to use
  // \param[in] unitShapes the unit-sized shapes (can be "cylinder", "sphere"
  //    or "cube") to use. Can be maximum two shapes.
  // \param[in] sdfModels the SDF models to use. Can be maximum two shapes.
  //   Can either be the path to a SDF file, or the name of a model in the
  //   GAZEBO_MODEL_PATHS.
  // \param[in] modelsGap the distance or "gap" between the models.
  //    The distance is the length of the gap between the model's AABBs if
  //    \e modelsGapIsFactor is false, or a factor of the larger of the two
  //    AABB sizes along the collision axis if \e modelsGapIsFactor is true.
  //    This also determines the length of the collision axis, which goes from
  //    the center of the first model to the center of the second model.
  //    To disable this and use the default, set to negative value.
  // \param[in] modelGapIsFactor interpretation of parameter \e modelsGap.
  // \return true on successful run, false if the test could not be started
  //        due to an error.
  public: bool Run(const std::vector<std::string>& physicsEngines,
                   const std::vector<std::string>& unitShapes,
                   const std::vector<std::string>& sdfModels,
                   const float modelsGap = -1,
                   const bool modelsGapIsFactor = true);

  // \brief Runs the test framework with the physics engines \e physicsEngines
  // and using the configuration saved in the configuration file \e configFile
  // in a previous run.
  //
  // If the configuration was saved on another system, and there are any
  // file paths for specifying the models, this will not currently work.
  // If the models are given with names and the models can be found in the
  // current systems GAZEBO_MODEL_PATHS, it will work.
  //
  // This is a blocking call. It returns when the test window (gzclient) was
  // closed.
  //
  // \param[in] physicsEngines see documentation in other Run() method.
  // \param[in] configFile the configuration file
  // \param[in] modelsGap see documentation in other Run() method.
  // \param[in] modelsGapIsFactor see documentation in other Run() method.
  //
  // \return true on successful run, false if the test could not be started
  //        due to an error.
  public: bool Run(const std::vector<std::string>& physicsEngines,
                   const std::string configFile,
                   const float modelsGap = -1,
                   const bool modelsGapIsFactor = true);

  // \brief implementation of public Run() methods
  // Requires variable \e configuration to be set.
  private: bool RunImpl(const std::vector<std::string>& physicsEngines,
                        const float modelsGap,
                        const bool modelsGapIsFactor);

  // \brief Handles the collision bar visual in gzclient.
  // This will add a visual cylinder of given radius and length (visual
  // is oriented along z axis) to the gzclient scene.
  // It will keep re-publishing the collision bar pose in order to enforce
  // that it cannot be moved with the move tools of gzclient.
  // The model will not be added to the world(s), it will only be added to
  // gzclient. Trying to manipulate the pose of the collision bar may
  // print errors of that the model cannot be found. This can be ignored.
  private: void CollisionBarHandler(const ignition::math::Pose3d &collBarPose,
                           const float cylinderRadius,
                           const float cylinderLength,
                           const std::string &mirrorName);

  // \brief Helper fuction which returns the AABB of the model from the first
  // world in \e worldManager.
  // Presumes that the model exists in all worlds and the AABB would be
  // the same (or very, very similar) in all worlds.
  // See also collision_benchmark::GetConsistentAABB() (in test/TestUtils.hh)
  // which checks that all AABBs are the same in both worlds. Automated tests
  // have ensured that this is the case if the model has been loaded in
  // all collision engine worlds simultaneously and the world hasn't been
  // updated since the model was added.
  // \param[in] modelName name of the model
  // \param[in] worldManager the world manager
  // \param[out] min minimum point of AABB
  // \param[out] max maxium point of AABB
  // \param[out] inLocalFrame \e min and \e max given in local coordinate frame
  // \retval 0 success
  // \retval -1 no worlds in world manager
  // \retval -2 could not get AABB for model
  private: int GetAABB(const std::string &modelName,
              const GzWorldManager::Ptr &worldManager,
              Vector3 &min, Vector3 &max, bool &inLocalFrame);

  // \brief Helper fuction which returns the AABB of the model from the
  // \e idxWorld'th world in \e worldManager.
  // \param[in] modelName name of the model
  // \param[in] idxWorld index of the world
  // \param[in] worldManager the world manager
  // \param[out] min minimum point of AABB
  // \param[out] max maxium point of AABB
  // \param[out] inLocalFrame \e min and \e max given in local coordinate frame
  // \retval 0 success
  // \retval -1 world \e idxWorld does not exist in world manager
  // \retval -2 could not get AABB for model
  private: int GetAABB(const std::string &modelName,
              const unsigned int idxWorld,
              const GzWorldManager::Ptr &worldManager,
              Vector3 &min, Vector3 &max, bool &inLocalFrame);



  // \brief Helper function which gets state of the model in the first world of
  // the \e worldManager. Presumes that the model exists in all worlds and the
  // state would be the same (or very, very similar) in all worlds.
  // \param[in] modelName name of the model
  // \param[in] worldManager the manager for all worlds
  // \param[out] state the output state
  // \retval 0 success
  // \retval -1 the model does not exist in the first world.
  // \retval -2 no worlds in world manager
  private: int GetBasicModelState(const std::string &modelName,
                   const GzWorldManager::Ptr &worldManager,
                   BasicState &state);

  // \brief Helper function which gets state of the model in \e idxWorld'th
  // world in the \e worldManager.
  // \param[in] modelName name of the model
  // \param[in] idxWold number of the world
  // \param[in] worldManager the manager for all worlds
  // \param[out] state the output state
  // \retval 0 success
  // \retval -1 the model does not exist in the \e idxWorld'th world.
  // \retval -2 world \e idxWorld does not exist
  private: int GetBasicModelState(const std::string &modelName,
                   const unsigned int idxWorld,
                   const GzWorldManager::Ptr &worldManager,
                   BasicState &state);

  // \brief Helper function which moves models towards/away from each other
  // along the axis by distance \e moveDist. Model 1 will move at \e moveDist,
  // model 2 will move the opposite direction at \e -moveDist.
  // \param[in] moveDist distance to move each model along axis
  // \param[in] moveBoth if true, both models are moved towards each other.
  //    If false, only model 2 is moved towards model 1.
  private: void MoveModelsAlongAxis(const float moveDist,
                                    const bool moveBoth = false);

  // \brief Moves models along collision axis until they collide
  // \param[in] allWorlds collision criteria is only met if all physics
  //    engines report collision between the objects
  // \param[in] moveBoth if true, both models are moved towards each other.
  //    If false, only model 2 is moved towards model 1.
  // \param[in] stepSize size of step the models are moved towards each other
  //    at a time.
  // \return the distance the shape(s) have moved along the axis.
  //  If \e moveBoth was true, this is the distance that *both* shapes
  //  have moved along the axis (the overall distance decreased between
  //  the two objects will be twice this value). If \e moveBoth was false,
  //  this is the distance model 2 has traveled. For model 2, the distance
  //  moved will be the negative of the returned value.
  private: double AutoCollide(const bool allWorlds, const bool moveBoth,
                              const double stepSize = 1e-03);

  // \brief Checks whether models collide
  // \param[in] allWorlds collision criteria is only met if all physics
  //    engines report collision between the objects
  private: bool ModelsCollide(bool allWorlds);

  // \brief Receives control messages from the GUI
  private: void receiveControlMsg(ConstAnyPtr &_msg);

  // \brief Saves the configuration.
  // This will save the model poses relative to their starting pose which
  // have been changed by the user. The amount the models were slid towards
  // or away from each other by means of AutoCollide or using the GUI slider
  // will not count. So when the configuration is loaded again, the models
  // will still start at each end of the collision axis, and when they are
  // collided, the same collision configuration as saved by this function
  // will be achieved by AutoCollide and/or the slider.
  // \param[in] model1Slide the amount which model 1 has been slid along the
  //    collision axis via AutoCollide or the slider
  // \param[in] model2Slide the amount which model 2 has been slid along the
  //    collision axis via AutoCollide or the slider
  private: void SaveConfiguration(const std::string &file,
                                  const double model1Slide,
                                  const double model2Slide);

  // \brief updates the current configuration
  // by reading the current model poses.
  // \param[in] model1Slide the amount which model 1 has been slid along the
  //    collision axis via AutoCollide or the slider
  // \param[in] model2Slide the amount which model 2 has been slid along the
  //    collision axis via AutoCollide or the slider
  // \return a *copy* of the current (updated) configuration or nullptr
  //    upon error.
  private: CollidingShapesConfiguration::Ptr
              UpdateConfiguration(const double model1Slide,
                                  const double model2Slide);

  // \brief Pointer to the multiple worlds server/client
  private: collision_benchmark::GazeboMultipleWorlds::Ptr gzMultiWorld;

  // \brief Flag indicating whether the framework is running.
  // Mainly used for the thread handling the collision bar in
  // CollisionBarHandler.
  private: std::atomic<bool> running;

  // \brief is set to true when a message is received to auto-collide objects
  private: std::atomic<bool> triggeredAutoCollide;

  // \brief is set to when a message is received to save the config to this file
  private: std::string triggeredSaveConfig;
  // \brief mutex for triggeredSaveConfig
  private: std::mutex triggeredSaveConfigMtx;

  // \brief position of the shapes along the axis.
  // 0 is when objects are moved as far as the axis allowed towards each
  // other, collision_benchmark::test::CollidingShapesParams::MaxSliderVal
  // is as far apart as axis allowed (considering the original object pose,
  // without having moved the objects via gclient).
  private: int shapesOnAxisPos;
  // \brief mutex for shapesOnAxisPos
  private: std::mutex shapesOnAxisPosMtx;

  // \brief Axis to use for collision.
  // Can be unit x, y or z axis
  private: const Vector3 collisionAxis;

  // \brief Names of both loaded models
  private: std::string loadedModelNames[2];

  // \brief currently loaded configuration.
  // Ensure this is updated with UpdateConfiguration() before use.
  private: CollidingShapesConfiguration::Ptr configuration;
};  // class

}  // namespace test
}  // namespace collision_benchmark

#endif  // COLLISION_BENCHMARK_TEST_COLLIDINGSHAPESFRAMEWORK_H
