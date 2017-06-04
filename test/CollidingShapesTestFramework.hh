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

#include <collision_benchmark/GazeboMultipleWorlds.hh>

#include <thread>
#include <unistd.h>
#include <sys/wait.h>

using collision_benchmark::GazeboMultipleWorlds;

namespace collision_benchmark
{
namespace test
{

/**
 * \brief Uses collision_benchmark::GazeboMultipleWorlds to load up a gazebo
 * server and client with multiple physics engines, and loads two shapes
 * into the world(s), one at each end of a "collision bar".
 * The shapes can be moved around relative to the collision bar, and will then
 * be made to collide along the direction of the bar. Step-wise moving the
 * objects towards and apart from each other can help to analyse how contact
 * points are moving when moving the two shapes relative to each other.
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
  // \param[in] unitShapes the unit-sized shapes (can be "cylinder", "sphere"
  //    or "cube") to use. Can be maximum two shapes.
  // \param[in] sdfModels the SDF models to use. Can be maximum two shapes.
  //   Can either be the path to a SDF file, or the name of a model in the
  //   GAZEBO_MODEL_PATHS.
  //
  // \return true on successful run, false if the test could not be started
  //        due to an error.
  public: bool Run(const std::vector<std::string>& physicsEngines,
                   const std::vector<std::string>& unitShapes,
                   const std::vector<std::string>& sdfModels);

  // \brief Callback function for the main loop in GazeboMultipleWorlds
  private: void LoopCallback(int iter);

  // \brief Handles the collision bar visual in gzclient.
  // This will add a visual cylinder of given radius and length (visual
  // is oriented along z axis) to the gzclient scene.
  // It will keep re-publishing the collision bar pose in order to enforce
  // that it cannot be moved with the move tools of gzclient.
  // The model will not be added to the world(s), it will only be added to
  // gzclient. Trying to manipulate the pose of the collision bar may
  // print errors of that the model cannot be found. This can be ignored.
  private: void CollisionBarHandler(const ignition::math::Pose3d& collBarPose,
                           const float cylinderRadius,
                           const float cylinderLength,
                           const std::string& mirrorName);

  // \brief Helper fuction which returns the AABB of the model from the first
  // world in \e worldManager.
  // Presumes that the model exists in all worlds and the AABB would be
  // the same (or very, very similar) in all worlds.
  // See also collision_benchmark::GetConsistentAABB() (in test/TestUtils.hh)
  // which checks that all AABBs are the same in both worlds. Automated tests
  // have ensured that this is the case if the model has been loaded in
  // all collision engine worlds simultaneously and the world hasn't been
  // updated since the model was added.
  //
  // \retval 0 success
  // \retval -1 the model does not exist in the first world.
  // \retval -2 no worlds in world manager
  // \retval -3 could not get AABB from model
  private: int GetAABB(const std::string& modelName,
              const GzWorldManager::Ptr& worldManager,
              Vector3& min, Vector3& max, bool& inLocalFrame);

  // \brief Helper function which gets state of the model in the first world of
  // \the e worldManager. Presumes that the model exists in all worlds and the
  // state would be the same (or very, very similar) in all worlds.
  // \retval 0 success
  // \retval -1 the model does not exist in the first world.
  // \retval -2 no worlds in world manager
  // \retval -3 could not get state of model
  private: int GetBasicModelState(const std::string& modelName,
                   const GzWorldManager::Ptr& worldManager,
                   BasicState& state);

  // \brief Helper function which moves models towards/away from each other
  // along the axis by distance \e moveDist.
  // \param[in] moveDist distance to move each model along axis
  private: void MoveModelsAlongAxis(const float moveDist);

  private: collision_benchmark::GazeboMultipleWorlds::Ptr gzMultiWorld;

  // \brief Flag indicating whether the framework is running.
  // Mainly used for the thread handling the collision bar in
  // CollisionBarHandler.
  private: std::atomic<bool> running;

  // \brief Axis to use for collision.
  // Can be unit x, y or z axis
  private: const Vector3 collisionAxis;

  // \brief Names of both loaded models
  private: std::string loadedModelNames[2];

};  // class

}  // namespace test
}  // namespace collision_benchmark

#endif  // COLLISION_BENCHMARK_TEST_COLLIDINGSHAPESFRAMEWORK_H
