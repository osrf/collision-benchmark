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
  worldManager->SetPaused(false);

  int numWorlds = worldManager->GetNumWorlds();

  // Set models to their initial pose.
  // First, place models at the origin in default orientation
  // (in case they were loaded form SDF they may have a pose different
  // to the origin, but here we want to start them at the origin).
  BasicState originPose;
  originPose.SetPosition(Vector3(0, 0, 0));
  originPose.SetRotation(Quaternion(0, 0, 0, 1));
  int cnt1 = worldManager->SetBasicModelState(modelName1, originPose);
  ASSERT_EQ(cnt1, numWorlds) << "All worlds should have been updated";
  int cnt2 = worldManager->SetBasicModelState(modelName2, originPose);
  ASSERT_EQ(cnt2, numWorlds) << "All worlds should have been updated";

  if (interactive)
  {
    std::cout << "Now start gzclient if you would like "
              << "to view the test. " << std::endl;
    std::cout << "Press [Enter] to continue." << std::endl;
    getchar();
  }

  std::cout << "Now starting test." << std::endl;
  while (true)
  {
    int numSteps = 1;
    worldManager->Update(numSteps);
    gazebo::common::Time::MSleep(1000);
  }
  std::cout << "ContactsFlicker test finished. " << std::endl;
}
