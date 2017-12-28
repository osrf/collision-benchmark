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

#include <string>
#include <vector>

/**
 * \brief Framework for the flickering contacts test.
 *
 * Is meant to be used with the gtest framework.
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

  ContactsFlickerTestFramework():
    MultipleWorldsTestFramework()
  {}
  virtual ~ContactsFlickerTestFramework()
  {}

  void FlickerTest(const std::string &modelName1,
                   const std::string &modelName2,
                   const bool interactive,
                   const std::string &outputBasePath,
                   const std::string &outputSubdir);

  // the helper for colliding models
  private: ModelColliderT modelCollider;
};

#endif  // COLLISION_BENCHMARK_TEST_CONTACTSFLICKERFRAMEWORK_H
