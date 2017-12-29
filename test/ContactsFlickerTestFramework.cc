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
#include <test/ContactsFlickerTestFramework.hh>

#include <collision_benchmark/PrimitiveShape.hh>
#include <collision_benchmark/SimpleTriMeshShape.hh>
#include <collision_benchmark/BasicTypes.hh>
#include <collision_benchmark/Helpers.hh>

#include <ignition/math/Vector3.hh>

#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>


#include <sstream>
#include <thread>
#include <atomic>

using collision_benchmark::Shape;
using collision_benchmark::BasicState;
using collision_benchmark::Vector3;
using collision_benchmark::Quaternion;
using collision_benchmark::PhysicsWorldBaseInterface;

////////////////////////////////////////////////////////////////
void ContactsFlickerTestFramework::FlickerTest(const std::string &modelName1,
                                               const std::string &modelName2,
                                               const bool interactive,
                                               const std::string &outputBasePath,
                                               const std::string &outputSubdir)
{
  GzMultipleWorldsServer::Ptr mServer = GetServer();
  ASSERT_NE(mServer.get(), nullptr) << "Could not create and start server";
  GzWorldManager::Ptr worldManager = mServer->GetWorldManager();
  ASSERT_NE(worldManager.get(), nullptr) << "No valid world manager created";

  worldManager->SetDynamicsEnabled(false);
  worldManager->SetPaused(true);

  int numWorlds = worldManager->GetNumWorlds();

  ignition::math::Vector3d collisionAxis(0,1,0);
  // initialize the model collider helper
  bool collInit = this->modelCollider.Init(worldManager, collisionAxis,
                                modelName1, modelName2);
  ASSERT_TRUE(collInit) << "Could not initialize model collider";

  BasicState initModelState1, initModelState2;
  // place models into their default position, with only a tiny gap
  // to ensure also boxes initially not colliding
  float modelsGap = 1e-2;
  bool placeSuccess = this->modelCollider.PlaceModels(modelsGap, false,
                                                      initModelState1,
                                                      initModelState2);
  ASSERT_TRUE(placeSuccess) << "Could not place models in initial pose";

  ASSERT_TRUE(initModelState1.PosEnabled() &&
              initModelState2.PosEnabled())
    << "Models are expected to have a position";

  worldManager->SetPaused(false);
  if (interactive)
  {
    std::cout << "Now start gzclient if you would like "
              << "to view the test. " << std::endl;
    std::cout << "Press [Enter] to continue." << std::endl;
    getchar();
  }


  std::cout << "Now starting test." << std::endl;

  ///// Iterate through all tests states.
  //////////////////////////////////////
  /// Move model 2 in circles of gradually increasing size relative to model 2
  /// (alternatively we could iterate through a *grid* lying on the plane
  /// to which the collision axis is the normal, but circles on this plane
  /// will also cover the space which the two models can collide on).
  /// Each time the model is moved one step along a circle, auto-collide the
  /// models in order to get the exact point of collision.
  /// We call this circle model2 is moved on the "outer circle".
  /// Then there are two tests:
  /// (a) The "inner circle" is a much smaller circle and
  /// this is used at each pose on an outer circle to slightly vary the
  /// models pose to do the test. The point moves within a small
  /// neighbourhood (inner circle radius): the contacts are not meant to vary
  /// greatly, if the collision status (collide yes/no) remains the same.
  /// (b) The shape is "wiggled" around, changing the orientation slightly,
  /// at the current pose along the outer circle.

  // Auto-collide parameters
  // ----------------
  const bool acAllWorlds = false;
  const double acStepSize = 1e-03;
  ASSERT_LT(acStepSize, modelsGap)
    << "Test setup inconsistency: gap should be > auto-collide step size";
  const bool acStopWhenPassed = true;
  float acMaxMovePerSec = 0.4;
  if (!interactive) acMaxMovePerSec = -1;

  // Circle parameters
  // ----------------

  // number of outer circles
  // TODO: This should be determined by the projection of both shapes on the plane
  const int numOuterCircles = 20;
  const int numInnerCircles = 2;
  // Radius increase of outer circle per iteration
  const float outerCircleRadiusInc = 0.1;
  // Radius increase of inner circle per iteration
  const float innerCircleRadiusInc = 0.02;
  // number of subdivisions for the outer circle
  const int numOuterCircleSubdivisions = 30;
  // number of subdivisions for the inner circle
  const int numInnerCircleSubdivisions = 30;
  // the inner circle test can be disabled
  const bool innerCircleTestEnabled = false;

  // Orientation change paramters
  // ---------------------------

  // the orientation test may be disabled
  const bool orientationTestEnabled = true;
  // subdivisions for orientation change test (see code below)
  const int numOriSubdivisions = 30;
  // the orientation is changed as if the shape was fixed to an axis
  // which walks along the surface of a cone around the collision axis.
  // This is the angle/opening of this cone.
  const float coneAngle = 0.5 * M_PI/180;

  std::cout << "Now iterating through all states." << std::endl;
  for (int oc = 0; oc < numOuterCircles; ++oc)
  {
    // std::cout << "Outer circle " << oc << std::endl;
    // move model along the outer circle in the requested number of subdivisions
    for (int ocSubDiv = 0; ocSubDiv < numOuterCircleSubdivisions; ++ocSubDiv)
    {
      // rotate the model to the right place on the circle
      const static double outerAngleStep
        = 360.0/numOuterCircleSubdivisions * M_PI/180;
      double outerAngle = ocSubDiv * outerAngleStep;

      // move models along circle Radius
      BasicState outerCurrMs2;
      this->modelCollider.MoveModelPerpendicular((oc+1)*outerCircleRadiusInc,
                     outerAngle, false, true, &initModelState2, &outerCurrMs2);
      const ignition::math::Vector3d outerCurrModelPos
        = collision_benchmark::ConvIgn<double>(outerCurrMs2.position);
      const ignition::math::Quaterniond outerCurrModelRot
        = collision_benchmark::ConvIgn<double>(outerCurrMs2.rotation);

      if (this->modelCollider.CollisionExcluded())
      {
        std::cout << "Collision excluded on outer angle "
                  << outerAngle << ", circle " << oc << std::endl;
        continue;
      }
      // auto-collide models
      // std::cout << "Auto-collide at angle " << outerAngle << std::endl;
      this->modelCollider.AutoCollide(acAllWorlds, false, acStepSize,
                                      acMaxMovePerSec, acStopWhenPassed,
                                      NULL, &outerCurrMs2);

      // do the inner circle movement and the test
      ////////////////////////////////////////////////
      if (innerCircleTestEnabled)
      {
        for (int ic = 0; ic < numInnerCircles; ++ic)
        {
          // std::cout << "Inner circle " << ic << std::endl;
          // move models along circle Radius
          this->modelCollider.MoveModelPerpendicular((ic+1)*innerCircleRadiusInc,
                                                   0, false, true, &outerCurrMs2);
          // move model along the inner circle in the requested
          // number of subdivisions
          for (int icSubDiv = 0; icSubDiv < numInnerCircleSubdivisions; ++icSubDiv)
          {
            // first rotate the model to the right place on the circle
            const static double innerAngleStep
                = 360.0/numInnerCircleSubdivisions * M_PI/180;
            double innerAngle = icSubDiv * innerAngleStep;
            this->modelCollider.RotateModelToPerpendicular(innerAngle,
                                                           outerCurrModelPos,
                                                           false, true);
            if (interactive) gazebo::common::Time::MSleep(10);
          }
        }
      }

      // re-set to model pose before the inner circle, and then do slight
      // perturbations in the orientation
      ASSERT_EQ(worldManager->SetBasicModelState(modelName2, outerCurrMs2),
                worldManager->GetNumWorlds())
        << "Could not set model pose to required pose";

      /// Orientation change test
      /////////////////////
      if (orientationTestEnabled)
      {
        // First, get the coordinate system around the collision axis
        const ignition::math::Vector3d collisionAxis = this->modelCollider.GetCollisionAxis();
        const ignition::math::Vector3d perpAxis = this->modelCollider.GetAxisPerpendicular(0);
        const ignition::math::Vector3d coneRotAxis = collisionAxis.Cross(perpAxis);
        // Compute quaternion for the cone opening
        const ignition::math::Quaterniond qCone(coneRotAxis, coneAngle);

        // Iterate through one round around the cone
        for (int oriSubDiv = 0; oriSubDiv < numOriSubdivisions; ++oriSubDiv)
        {
          const static double oriAngleStep = 360.0/numOriSubdivisions * M_PI/180;
          double oriAngle = oriSubDiv * oriAngleStep;
          double f = oriSubDiv/(double)numOriSubdivisions * 2*M_PI;
          ignition::math::Quaterniond q(cos(f)*coneAngle, sin(f)*coneAngle, 0);
          q = q * outerCurrModelRot;

          BasicState oriState = outerCurrMs2;
          oriState.rotation = collision_benchmark::Conv(q);
          ASSERT_EQ(worldManager->SetBasicModelState(modelName2, oriState),
                  worldManager->GetNumWorlds())
              << "Could not set model pose to required pose";
          worldManager->Update(1);
          if (interactive) gazebo::common::Time::MSleep(10);
        }
        // reset model pose again
        ASSERT_EQ(worldManager->SetBasicModelState(modelName2, outerCurrMs2),
                  worldManager->GetNumWorlds())
          << "Could not set model pose to required pose";
      }
    }
  }
/*  while (true)
  {
    const int numSteps = 1;
    worldManager->Update(numSteps);
    gazebo::common::Time::MSleep(1000);
  }*/
  std::cout << "ContactsFlicker test finished. " << std::endl;
}
