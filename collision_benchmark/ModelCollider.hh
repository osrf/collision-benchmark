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
#ifndef COLLISION_BENCHMARK_MODELCOLLIDER_H
#define COLLISION_BENCHMARK_MODELCOLLIDER_H

#include <collision_benchmark/BasicTypes.hh>
#include <collision_benchmark/WorldManager.hh>

#include <thread>
#include <mutex>
#include <atomic>
#include <string>
#include <vector>
#include <unistd.h>
#include <sys/wait.h>

namespace collision_benchmark
{

/**
 * \brief Helper class which can be used to collide two models.
 * The models are collided by moving them towards each other along a
 * collision axis that the user can specify.
 * The two models have to be loaded in all of the worlds, and they should
 * be at the same pose in all worlds (function PlaceModels() can be used
 * to ensure this). The main purpose of this class is to
 * provide the functions AutoCollide() and MoveModelsAlongAxis().
 *
 * \author Jennifer Buehler
 * \date November 2017
 */
template<class WorldManagerType>
class ModelCollider
{
  private: typedef WorldManagerType WorldManagerT;
  private: typedef typename WorldManagerT::Vector3 Vector3;
  public: typedef typename WorldManagerT::Ptr WorldManagerPtr;

  public: ModelCollider();

  // \brief initializes the model collider
  // \param[in] worldManager the world manager
  // \param[in] collisionAxis the collision axis to use
  // \param[in] modelName1 name of first model. Must be loaded in all worlds.
  // \param[in] modelName2 name of second model. Must be loaded in all worlds.
  // \return success flag
  public: bool Init(const WorldManagerPtr &worldManager,
                    const ignition::math::Vector3d &collisionAxis,
                    const std::string &modelName1,
                    const std::string &modelName2);

  // \brief sets the collision axis
  // \param[in] collisionAxis the collision axis to use
  // \return axis accepted or not
  public: bool SetCollisionAxis(const ignition::math::Vector3d &axis);
  public: ignition::math::Vector3d GetCollisionAxis() const
          {
            return this->collisionAxis;
          }

  // \brief places the models at both ends of the collision axis.
  // This will move the first model to the origin and the second along the
  // collision axis, starting from the first model.
  // This function can be used to put the models in a suitable position for
  // the collision, or the user may choose to place the models by other means,
  // however AutoCollide() will only work if the shapes do collide when moved
  // from their poses along the axis direction towards each other. This
  // function aims at providing a helper to achieve such a condition with
  // high probability.
  // \param[in] modelsGap the distance or "gap" between the models.
  //    The distance is the length of the gap between the model's AABBs if
  //    \e modelsGapIsFactor is false, or a factor of the larger of the two
  //    AABB sizes along the collision axis if \e modelsGapIsFactor is true.
  //    This also determines the length of the collision axis, which goes from
  //    the center of the first model to the center of the second model.
  //    To disable this and use the default, set to negative value.
  // \param[in] modelGapIsFactor interpretation of parameter \e modelsGap.
  // \param[out] modelState1 the final state which model 1 was set to.
  // \param[out] modelState2 the final state which model 2 was set to.
  // \return false if not initialised properly
  public: bool PlaceModels(const float modelsGap,
                         const bool modelsGapIsFactor,
                         BasicState &modelState1,
                         BasicState &modelState2);

  // \brief Helper function which moves models towards/away from each other
  // along the axis by distance \e moveDist. Model 1 will move at \e moveDist,
  // model 2 will move the opposite direction at \e -moveDist.
  // Updates the worlds by one step (calls WorldManager::Update()).
  // \param[in] moveDist distance to move each model along axis
  // \param[in] moveBoth if true, both models are moved towards each other.
  //    If false, only model 2 is moved towards model 1.
  // \param[in] stopWhenPasses stop when the projection of the centres of the
  //    models onto the collision axis have passed each other in the move
  //    direction. This is not a reliable way to determine whether the models
  //    could theoretically not collide any more when moving in this direction,
  //    but in most cases (with convex shapes) this will be true.
  // \param[in] worldUpdate if true, the worlds are updated upon success.
  // \param[out] ms1 if set to not NULL, this is going to be the state
  //    of model1 after the move, or left unchanged if \e moveBoth is false.
  // \param[out] ms2 if set to not NULL, this is going to be the state
  //    of model2 after the move
  // \retval -1 error
  // \retval 0 success
  // \retval 1 \e stopWhenPassed was true and object centers have passed
  //      each other.
  public: int MoveModelsAlongAxis(const double moveDist,
                                  const bool moveBoth = false,
                                  const bool stopWhenPassed = false,
                                  const bool worldUpdate = true,
                                  BasicState *ms1 = NULL,
                                  BasicState *ms2 = NULL);

  // \brief Moves models along collision axis until they collide.
  // This requires that they do collide when slid towards each other along
  // the collision axis directon. See also PlaceModels() which can be used
  // to achieve such a state.
  // Slides the shapes towards each other in several steps of size \e stepSize
  // and updates the worlds (calls WorldManager::Update()) at each step.
  // \param[in] allWorlds collision criteria is only met if all physics
  //    engines report collision between the objects
  // \param[in] moveBoth if true, both models are moved towards each other.
  //    If false, only model 2 is moved towards model 1.
  // \param[in] stepSize size of step the models are moved towards each other
  //    at a time.
  // \param[in] maxMovePerSec to allow for an animation effect,
  //    move the shapes at this maximum distance per second.
  //    There cannot be a minimum velocity because we have to make tiny
  //    movements in order to capture the first point of collision as exact
  //    as possible. If negative, the velocity is not going to be controlled
  //    and the shapes will just move towards each other with steps of size
  //    \e stepSize
  // \param[in] stopWhenPassed see ModeModelsAlongAxis parameter stopWhenPassed
  // \param[out] ms1 if set to not NULL, this is going to be the state
  //    of model1 after the move, or left unchanged if \e moveBoth is false.
  // \param[out] ms2 if set to not NULL, this is going to be the state
  //    of model2 after the move
  // \return the distance the shape(s) have moved along the axis.
  //  If \e moveBoth was true, this is the distance that *both* shapes
  //  have moved along the axis (the overall distance decreased between
  //  the two objects will be twice this value). If \e moveBoth was false,
  //  this is the distance model 2 has traveled. For model 2, the distance
  //  moved will be the negative of the returned value.
  public: double AutoCollide(const bool allWorlds = false,
                             const bool moveBoth = false,
                             const double stepSize = 1e-03,
                             const float maxMovePerSec = 0.4,
                             const bool stopWhenPassed = false,
                             BasicState *ms1 = NULL,
                             BasicState *ms2 = NULL);

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
  private: static int GetAABB(const std::string &modelName,
              const WorldManagerPtr &worldManager,
              Vector3 &min, Vector3 &max, bool &inLocalFrame);

  // \brief Helper which returns the AABB of the model from the
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
  private: static int GetAABB(const std::string &modelName,
              const unsigned int idxWorld,
              const WorldManagerPtr &worldManager,
              Vector3 &min, Vector3 &max, bool &inLocalFrame);

  // \brief Helper which can be used to get the AABB coordinates
  // in global frame transformed by \e q
  // \param[in] q additional rotation of the global frame. When identity,
  //    AABB is therefore returned in global frame. Only rotation supported,
  //    therefore a quaternion is used.
  // \param[in] modelName name of the model
  // \param[in] min min AABB coordinate in local frame
  // \param[in] max max AABB coordinate in local frame
  // \param[out] newMin min coordinate in global frame
  // \param[out] newMax max coordinate in global frame
  // \return false if the state of the model could not be retrieved
  private: bool GetAABBInFrame(const ignition::math::Quaterniond& q,
                               const std::string &modelName,
                               const Vector3 &min, const Vector3 &max,
                               Vector3 &newMin, Vector3 &newMax) const;

  // \brief Helper function which gets state of the model in the first world of
  // the \e worldManager. Presumes that the model exists in all worlds and the
  // state would be the same (or very, very similar) in all worlds.
  // \param[in] modelName name of the model
  // \param[in] worldManager the manager for all worlds
  // \param[out] state the output state
  // \retval 0 success
  // \retval -1 the model does not exist in the first world.
  // \retval -2 no worlds in world manager
  private: static int GetBasicModelState(const std::string &modelName,
                   const WorldManagerPtr &worldManager,
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
  public: static int GetBasicModelState(const std::string &modelName,
                   const unsigned int idxWorld,
                   const WorldManagerPtr &worldManager,
                   BasicState &state);

  // \brief Checks whether models collide
  // \param[in] allWorlds collision criteria is only met if all physics
  //    engines report collision between the objects
  public: bool ModelsCollide(bool allWorlds) const;

  // \brief Checks if collision of models along the collision axis is excluded.
  // This is only an approximate test because the AABBs of the models are
  // used to check for impossible collision. So even if this function returns
  // false, the models may still not collide.
  public: bool CollisionExcluded() const;

  // \brief Gets the axis perpendicular to the collision axis, rotated around
  // \e angle about the collision axis.
  // Will always return the same vector for the same collision axis.
  // \param[in] angle the angle the perpendicular axis is rotated about
  //  the collision axis
  // \return the perpendicular axis used in this ModelCollider, rotated
  //    about \e angle. Is a unit length vector.
  public: ignition::math::Vector3d
            GetAxisPerpendicular(const double angle = 0) const;

  // \brief Helper: gets an axis perpendicular to the \e axis, rotated around
  // \e angle about \e axis.
  // Will always return the same vector for the same \e axis.
  // If \e axis is unit length, the returned vector is unit length too.
  private: static ignition::math::Vector3d
            GetAxisPerpendicular(const ignition::math::Vector3d &axis,
                                 const double angle);

  // \brief move a model perpendicular to the collision axis by this distance.
  // The perpendicular axis is always the same for the collision axis,
  // it is GetAxisPerpendicular().
  // \param[in] distance the distance
  // \param[in] angle the angle which the perpendicular axis is rotated around
  //    the collision axis
  // \param[in] model1 if true, model 1 is moved. Otherwise, model 2 is moved.
  // \param[in] worldUpdate if true, the worlds are updated upon success.
  // \param[in] fromState if set to not NULL, this pose is going to be as
  //    current model pose from where to move it, instead of requesting the
  //    current model state in the world.
  // \param[out] endState if not NULL, this will be set to the model state
  //    after the move.
  // \return true on success, false if model state could not be set
  public: bool MoveModelPerpendicular(const double distance,
                                      const double angle,
                                      const bool model1,
                                      const bool worldUpdate = true,
                                      const BasicState *fromState = NULL,
                                      BasicState* endState = NULL) const;

  // \brief Aligns the model with the perpendicular axis.
  // The collision axis can be placed at a position \e axisOrigin.
  // The model itself will keep its orientation, only the models position
  // will change.
  // Essentially this action amounts to first translating the model to its
  // projection on the collision axis and then translating it by the same
  // distance along the perpendicular axis which is returned by
  // GetAxisPerpendicular(this->collisionAxis, angle).
  // \param[in] angle the angle
  // \param[in] axisOrigin the origin of the axis
  // \param[in] model1 if true, model 1 is moved. Otherwise, model 2 is moved.
  // \param[in] worldUpdate if true, the worlds are updated upon success.
  // \return true on success, false if model state could not be set
  public: bool RotateModelToPerpendicular(const double angle,
                           const ignition::math::Vector3d &axisOrigin,
                           const bool model1,
                           const bool worldUpdate = true) const;

  // \brief the world manager
  private: WorldManagerPtr worldManager;

  // \brief Names of both loaded models
  private: std::string modelNames[2];

  // \brief Axis to use for collision.
  private: ignition::math::Vector3d collisionAxis;

};  // class

}  // namespace collision_benchmark

#include "ModelCollider-inl.hh"
#endif  // COLLISION_BENCHMARK_MODELCOLLIDER_H
