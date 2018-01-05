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
#include <collision_benchmark/StartWaiter.hh>

#include <ignition/math/Vector3.hh>

#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>


#include <sstream>
#include <thread>
#include <atomic>

#define EXIT_SIGNAL -1
#define NEXT_SIGNAL 0
#define PREV_SIGNAL 1

using collision_benchmark::Shape;
using collision_benchmark::BasicState;
using collision_benchmark::Vector3;
using collision_benchmark::Quaternion;
using collision_benchmark::PhysicsWorldBaseInterface;
using collision_benchmark::StartWaiter;
using collision_benchmark::SignalReceiver;

////////////////////////////////////////////////////////////////
ignition::math::Vector3d getClosest(const ignition::math::Vector3d& v,
                                  const std::vector<ignition::math::Vector3d> c,
                                  double& dist)
{
  if (c.empty())
    throw std::runtime_error("Don't call getClosest() with an empty vector");
  double minDist = std::numeric_limits<double>::max();
  ignition::math::Vector3d ret(minDist, minDist, minDist);
  for (std::vector<ignition::math::Vector3d>::const_iterator it = c.begin();
       it != c.end(); ++it)
  {
    // std::cout << "Check " << v << std::endl;
    ignition::math::Vector3d diff = v-*it;
    double len = diff.Length();
    if (len < minDist)
    {
      ret = *it;
      minDist = len;
      dist = minDist;
    }
  }
  return ret;
}

////////////////////////////////////////////////////////////////
ContactsFlickerTestFramework::ContactsFlickerTestFramework():
  MultipleWorldsTestFramework()
{
}

////////////////////////////////////////////////////////////////
ContactsFlickerTestFramework::~ContactsFlickerTestFramework()
{
}


////////////////////////////////////////////////////////////////
bool ContactsFlickerTestFramework::SignificantContactDiff(
      const std::vector<ignition::math::Vector3d> &contacts1,
      const std::vector<ignition::math::Vector3d> &contacts2,
      const double contactsMoveTolerance) const
{
  // if one of the states had no contacts, any change in contact
  // points is accepted
  if (contacts1.empty() || contacts2.empty())
  {
    return false;
  }

  // the loop below has to use the larger cluster
  const std::vector<ignition::math::Vector3d> &cLarge =
    contacts1.size() > contacts2.size() ? contacts1 : contacts2;
  const std::vector<ignition::math::Vector3d> &cSmall =
    contacts1.size() <= contacts2.size() ? contacts1 : contacts2;

  // equal number of clusters: No cluster should have moved further
  // than the tolerance from any cluster in the previous iteration
  for (std::vector<ignition::math::Vector3d>::const_iterator
       it = cLarge.begin(); it != cLarge.end(); ++it)
  {
    const ignition::math::Vector3d &contact = *it;
    double dist = -1;
    ignition::math::Vector3d closest =
      getClosest(contact, cSmall, dist);
    if (dist > contactsMoveTolerance)
    {
      std::cout << "Failed due to point distance " << dist
                << ". # Clusters: " << contacts1.size() << ", " << contacts2.size() << std::endl;
      return true;
    }
  }
  return false;
}

////////////////////////////////////////////////////////////////
bool ContactsFlickerTestFramework::CheckClientExit() const
{
  if (!GetMultipleWorlds())
  {
    std::cerr << "Server is down" << std::endl;
    return true;
  }
  return !GetMultipleWorlds()->IsClientRunning();
}


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

  if (interactive)
  {
    // initialize signalReceiver for the StepGui interface.
    // This should not be done in the constructor because the constructors
    // are called before transport is initialized.
    signalReceiver.InitAnyMsg("/test/cmd");
    signalReceiver.AddIntSignal(NEXT_SIGNAL, 1);
    signalReceiver.AddIntSignal(PREV_SIGNAL, -1);
    std::function<bool(void)> cb =
      std::bind(&ContactsFlickerTestFramework::CheckClientExit, this);
    signalReceiver.AddCallback(EXIT_SIGNAL, cb);

    ASSERT_NE(GetMultipleWorlds(), nullptr) << "Server is down";
    while (!GetMultipleWorlds()->IsClientRunning())
    {
      std::cout << "Waiting for client to be up" << std::endl;
      gazebo::common::Time::MSleep(100);
    }
    // switch on contacts display and transparent view
    gazebo::common::Time::Sleep(1);  // XXX TODO do a wait for transport instead of this
    gazebo::transport::requestNoReply("mirror",
                                      "show_contact", "all");
    gazebo::transport::requestNoReply("mirror",
                                      "set_transparent", "all");
  }

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
    std::cout << "Press [>>] to start the test." << std::endl;
    std::set<int> sig =
      signalReceiver.WaitForSignal({NEXT_SIGNAL, EXIT_SIGNAL});
    if (sig.find(EXIT_SIGNAL) != sig.end())
    {
      std::cout << "EXIT test." << std::endl;
      return;
    }
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


  // General parameters
  // ----------------

  // in interactive mode, slow down movements
  const bool slowDown = false;
  const double slowDownMS = 10;
  // minimum absolute tolerance the contacts are allowed to move
  const double minMoveTolerance = 8e-02;
  // the contacts are allowed to move as much as the model has moved
  // times this tolerance factor. So if the model moved by x, the contacts
  // are allowed to move by max(x*toleranceFact, minMoveTolerance).
  const double toleranceFact = 1.0;
  // Contact points are clustered before doing the check.
  // This is the minimum absolute cluster size
  const double minClusterSize = 1e-02;
  // The size of the cluster is std::max(m*clusterFact, minClusterSize),
  // where m is the distance the model moved
  const double clusterFact = 0.1;
  // the inner circle test can be disabled
  const bool innerCircleTestEnabled = false;
  // the orientation test may be disabled
  const bool orientationTestEnabled = true;
  // the world index (in WorldManager) to test
  const int testWorldIdx = 0;


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
  const int numOuterCircles = 40;
  const int numInnerCircles = 1;
  // Radius increase of outer circle per iteration
  const float outerCircleRadiusInc = 0.05;
  // Radius increase of inner circle per iteration
  const float innerCircleRadiusInc = 0.001;
  // number of subdivisions for the outer circle
  const int numOuterCircleSubdivisions = 50;
  // number of subdivisions for the inner circle
  const int numInnerCircleSubdivisions = 50;

  // Orientation change paramters
  // ---------------------------

  // subdivisions for orientation change test (see code below)
  const int numOriSubdivisions = 30;
  // the orientation is changed as if the shape was fixed to an axis
  // which walks along the surface of a cone around the collision axis.
  // This is the angle/opening of this cone.
  const float coneAngle = 0.2 * M_PI/180;


  // Iteration start paramters
  // ---------------------------
#if 1
  // start outer loop at this index
  const int ocStart = 0;
  // start outer loop of circle subdivisions at this index
  const int ocSubDivStart = 0;
  // start inner loop at this index
  const int icSubDivStart = 0;
  // start orientation loop at this index
  const int oriSubDivStart = 0;
#else
  const int ocStart = 7;
  const int ocSubDivStart = 6;
  const int icSubDivStart = 0;
  const int oriSubDivStart = 12;
#endif

  // XXX ODE test cases:
  // oc=2, ocSubDiv=42, oriSubDiv=16
  // oc=7, ocSubDiv=6, oriSubDiv=12  pretty good


  std::cout << "Now iterating through all states." << std::endl;
  for (int oc = ocStart; oc < numOuterCircles; ++oc)
  {
    // std::cout << "Outer circle " << oc << std::endl;
    // move model along the outer circle in the requested number of subdivisions
    for (int ocSubDiv = ocSubDivStart;
         ocSubDiv < numOuterCircleSubdivisions; ++ocSubDiv)
    {
      ASSERT_NE(GetMultipleWorlds(), nullptr) << "Server is down";
      if (interactive && !GetMultipleWorlds()->IsClientRunning())
      {
        std::cout << "Client stopped. " << std::endl;
        return;
      }
      // rotate the model to the right place on the circle
      const static double outerAngleStep
        = 360.0/numOuterCircleSubdivisions * M_PI/180;
      double outerAngle = ocSubDiv * outerAngleStep;

      // move models along circle Radius
      BasicState tmp;
      this->modelCollider.MoveModelPerpendicular((oc+1)*outerCircleRadiusInc,
                     outerAngle, false, true, &initModelState2, &tmp);

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
                                      NULL, &tmp);

      if (!this->modelCollider.ModelsCollide(acAllWorlds))
      {
        std::cout << "Models don't collide, skip test" << std::endl;
        continue;
      }

      // ensure outerCurrMs2 is const (and bugproof), therefore use tmp above
      const BasicState outerCurrMs2 = tmp;
      const ignition::math::Vector3d outerCurrModelPos
        = collision_benchmark::ConvIgn<double>(outerCurrMs2.position);
      const ignition::math::Quaterniond outerCurrModelRot
        = collision_benchmark::ConvIgn<double>(outerCurrMs2.rotation);

      // do the inner circle movement and the test
      ////////////////////////////////////////////////
      if (innerCircleTestEnabled)
      {
        for (int ic = 0; ic < numInnerCircles; ++ic)
        {
          // std::cout << "Inner circle " << ic << std::endl;
          // move models along circle Radius
          this->modelCollider.MoveModelPerpendicular
                ((ic+1)*innerCircleRadiusInc, 0, false, true, &outerCurrMs2);

          // to remember the last iterations contact points
          std::vector<ignition::math::Vector3d> lastConts;
          // flag on whether the current iteration is only to view the state.
          // The following iteration will do the test again.
          bool viewLastIteration = false;
          // move model along the inner circle in the requested
          // number of subdivisions
          for (int icSubDiv = icSubDivStart;
               icSubDiv < numInnerCircleSubdivisions; ++icSubDiv)
          {
            if (interactive && slowDown)
              gazebo::common::Time::MSleep(slowDownMS);
            // first rotate the model to the right place on the circle
            const static double innerAngleStep
                = 360.0/numInnerCircleSubdivisions * M_PI/180;
            double innerAngle = icSubDiv * innerAngleStep;

            BasicState innerCurrMs;
            this->modelCollider.RotateModelToPerpendicular(innerAngle,
                                                           outerCurrModelPos,
                                                           false, true,
                                                           &innerCurrMs);
            const ignition::math::Vector3d innerCurrModelPos
              = collision_benchmark::ConvIgn<double>(innerCurrMs.position);

            if (viewLastIteration)
            {
              std::cout << "Press [>>] to continue." << std::endl;
              std::set<int> sigs =
                signalReceiver.WaitForSignal({EXIT_SIGNAL, NEXT_SIGNAL});
              if (sigs.find(EXIT_SIGNAL) != sigs.end())
              {
                std::cout << "Exit client." << std::endl;
                return;
              }
              viewLastIteration = false;
              // continue to avoid setting of lastConts
              continue;
            }

            // Do the test
            // ***********************
            const double modelMoved =
              (outerCurrModelPos - innerCurrModelPos).Length();
            // XXX TODO: the orientation would need to be considered too.
            // The maximum amount a contact may have moved is then the maximum
            // distance a point in the model would have traveled if only the
            // orientation had changed.
            const double clusterSize =
              std::max(modelMoved * clusterFact, minClusterSize);
            const double contactsMoveTolerance
              = std::max(modelMoved * toleranceFact, minMoveTolerance);
            std::vector<ignition::math::Vector3d> conts
              = this->modelCollider.GetClusteredContacts(testWorldIdx,
                                                         clusterSize);
            // do the diff test only for the 2nd contact because we
            // need lastConts
            if ((icSubDiv > icSubDivStart) &&
                SignificantContactDiff(lastConts, conts, contactsMoveTolerance))
            {
              std::cout << "Movement test: Stop at oc=" << oc
                << ", ocSubDiv=" << ocSubDiv << ", icSubDiv="
                << icSubDiv << std::endl;
              if (interactive)
              {
                std::cout << "Press [<<] or [>>] to continue." << std::endl;
                std::set<int> sigs = signalReceiver.WaitForAnySignal();
                if (sigs.find(EXIT_SIGNAL) != sigs.end())
                {
                  std::cout << "Exit client." << std::endl;
                  return;
                }
                else if (sigs.find(PREV_SIGNAL) != sigs.end())
                {
                  icSubDiv -= 2;
                  viewLastIteration = true;
                  // continue in order to avoid
                  continue;
                }
              }
            }
            lastConts = conts;
            // ***********************
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
        // Get the coordinate system around the collision axis
        const ignition::math::Vector3d collisionAxis =
          this->modelCollider.GetCollisionAxis();
        const ignition::math::Vector3d perpAxis =
          this->modelCollider.GetAxisPerpendicular(0);
        const ignition::math::Vector3d coneRotAxis =
          collisionAxis.Cross(perpAxis);
        // Compute quaternion for the cone opening
        const ignition::math::Quaterniond qCone(coneRotAxis, coneAngle);


        // flag on whether the current iteration is only to view the state.
        // The following iteration will do the test again.
        bool viewLastIteration = false;
        // for remebering the last iteration's AABB
        ignition::math::Vector3d lastMinAABB, lastMaxAABB;
        // to remember the last iterations contact points
        std::vector<ignition::math::Vector3d> lastConts;
        // Iterate through one round around the cone
        for (int oriSubDiv = oriSubDivStart;
             oriSubDiv < numOriSubdivisions; ++oriSubDiv)
        {
          const static double oriAngleStep
            = 360.0/numOriSubdivisions * M_PI/180;
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
          if (interactive && slowDown)
            gazebo::common::Time::MSleep(slowDownMS);

          if (viewLastIteration)
          {
            std::cout << "VIEW ITERATION: Orientation test oc=" << oc
              << ", ocSubDiv=" << ocSubDiv << ", oriSubDiv="
              << oriSubDiv << std::endl;
            std::cout << "Press [>>] to continue." << std::endl;
            std::set<int> sigs =
              signalReceiver.WaitForSignal({EXIT_SIGNAL, NEXT_SIGNAL});
            if (sigs.find(EXIT_SIGNAL) != sigs.end())
            {
              std::cout << "Exit client." << std::endl;
              return;
            }
            viewLastIteration = false;
            // continue to avoid setting of lastConts
            continue;
          }

          // Do the test
          // ***********************
          ignition::math::Vector3d minAABB, maxAABB;
          bool aabbInLocal;
          this->modelCollider.GetAABB(modelName2, testWorldIdx,
                                      minAABB, maxAABB, aabbInLocal);
          EXPECT_FALSE(aabbInLocal)
            << "Adjust implementation to support AABB in local frame";

          // Check min and max points of AABBs of this and last iteration
          // to see how much it may have moved at most.
          // If there is no last iteration, assume a move of 0 so the cluster
          // size is minimal. The actual test won't be performed then.
          double maxMove =
              (oriSubDiv == oriSubDivStart) ? 0 :
                                 std::max(fabs((lastMinAABB-minAABB).Length()),
                                          fabs((lastMaxAABB-maxAABB).Length()));
          const double clusterSize =
              std::max(maxMove * clusterFact, minClusterSize);
          const double contactsMoveTolerance =
            std::max(maxMove * toleranceFact, minMoveTolerance);

          /*std::cout << "DEBUG: Orientation test: Stop at oc=" << oc
              << ", ocSubDiv=" << ocSubDiv << ", oriSubDiv="
              << oriSubDiv << ", max move tol = " << contactsMoveTolerance
              << " ( maxMove=" << maxMove << ")" << std::endl;*/

          std::vector<ignition::math::Vector3d> conts
            = this->modelCollider.GetClusteredContacts(testWorldIdx,
                                                       clusterSize);

          /*std::cout  << "... Number of contacts: " << conts.size() << std::endl;
          for (auto ii=conts.begin(); ii!=conts.end(); ++ii)
            std::cout << *ii << " ";
          std::cout << std::endl << "LAST: " << std::endl;
          for (auto ii=lastConts.begin(); ii!=lastConts.end(); ++ii)
            std::cout << *ii << " ";*/

          // do the diff test only for the 2nd contact because we
          // need lastConts and lastMin/MaxAABB
          if ((oriSubDiv > oriSubDivStart) &&
              SignificantContactDiff(lastConts, conts, contactsMoveTolerance))
          {
            std::cout << "Orientation test: Stop at oc=" << oc
              << ", ocSubDiv=" << ocSubDiv << ", oriSubDiv="
              << oriSubDiv << ", max move tol = " << contactsMoveTolerance
              << " ( maxMove=" << maxMove << ")" << std::endl;
            if (interactive)
            {
              std::cout << "Press [<<] or [>>] to continue." << std::endl;
              std::set<int> sigs = signalReceiver.WaitForAnySignal();
              if (sigs.find(EXIT_SIGNAL) != sigs.end())
              {
                std::cout << "Exit client." << std::endl;
                return;
              }
              else if (sigs.find(PREV_SIGNAL) != sigs.end())
              {
                oriSubDiv -= 2;
                viewLastIteration = true;
                continue;
              }
            }
          }
          lastConts = conts;
          lastMinAABB = minAABB;
          lastMaxAABB = maxAABB;
          // ***********************
        }
        // reset model pose again
        ASSERT_EQ(worldManager->SetBasicModelState(modelName2, outerCurrMs2),
                  worldManager->GetNumWorlds())
          << "Could not set model pose to required pose";
      }
    }
  }
  std::cout << "ContactsFlicker test finished. " << std::endl;
/*  if (interactive)
  {
    std::cout << "Now entering endless update loop, kill with Ctrl+C"
              << std::endl;
    while (true)
    {
      const int numSteps = 1;
      worldManager->Update(numSteps);
      gazebo::common::Time::MSleep(1000);
    }
  }*/
}
