/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#ifndef COLLISION_BENCHMARK_TEST_CONTACTSFLICKERFRAMEWORK_H
#define COLLISION_BENCHMARK_TEST_CONTACTSFLICKERFRAMEWORK_H

#include <test/MultipleWorldsTestFramework.hh>
#include <test/TestUtils.hh>
#include <collision_benchmark/Shape.hh>
#include <collision_benchmark/ModelCollider.hh>
#include <collision_benchmark/SignalReceiver.hh>

#include <string>
#include <vector>

/**
 * \brief Framework for the flickering contacts test.
 *
 * This test is experimental and has not been finalised.
 *
 * \author Jennifer Buehler
 * \date April 2017
 */
class ContactsFlickerTestFramework : public MultipleWorldsTestFramework
{
 protected:
  typedef GzWorldManager::PhysicsWorldContactInterfaceT::ContactInfo
            GzContactInfo;
  typedef GzContactInfo::Ptr GzContactInfoPtr;

  typedef collision_benchmark::ModelCollider<GzWorldManager> ModelColliderT;

  ContactsFlickerTestFramework();
  virtual ~ContactsFlickerTestFramework();

  void FlickerTest(const std::string &modelName1,
                   const std::string &modelName2,
                   const bool interactive,
                   const std::string &outputBasePath,
                   const std::string &outputSubdir);

  private:
  // Helper function which determines whether the difference between contact1
  // and contacts2 are significant and the test will fail
  // \return true if the difference is significant
  bool SignificantContactDiff(
      const std::vector<ignition::math::Vector3d> &contacts1,
      const std::vector<ignition::math::Vector3d> &contacts2,
      const double contactsMoveTolerance = 1e-02) const;

  // Helper function which checks if the client is running, when using
  // interactive mode.
  bool CheckClientExit() const;

  // the helper for colliding models
  private: ModelColliderT modelCollider;
  // Receiver for commands from the StepGui interface
  private: collision_benchmark::SignalReceiver signalReceiver;
};

#endif  // COLLISION_BENCHMARK_TEST_CONTACTSFLICKERFRAMEWORK_H
