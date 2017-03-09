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
#ifndef COLLISION_BENCHMARK_TEST_BASICTESTFRAMEWORK_H
#define COLLISION_BENCHMARK_TEST_BASICTESTFRAMEWORK_H

#include <gtest/gtest.h>
#include <gazebo/gazebo.hh>

class BasicTestFramework : public ::testing::Test {
  protected:

  BasicTestFramework()
  :fakeProgramName("BasicTestFramework")
  {
  }
  virtual ~BasicTestFramework()
  {
  }

  virtual void SetUp()
  {
    // irrelevant to pass fake argv, so make an exception
    // and pass away constness, so that fakeProgramName can be
    // initialized easily in constructor.
    gazebo::setupServer(1, (char**)&fakeProgramName);
  }

  virtual void TearDown()
  {
    gazebo::shutdown();
  }

  private:
  const char * fakeProgramName;
};

#endif  // COLLISION_BENCHMARK_TEST_BASICTESTFRAMEWORK_H
