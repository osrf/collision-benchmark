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
#ifndef COLLISION_BENCHMARK_TEST_STATICTESTFRAMEWORK_H
#define COLLISION_BENCHMARK_TEST_STATICTESTFRAMEWORK_H

#include <collision_benchmark/Shape.hh>
#include "MultipleWorldsTestFramework.hh"

class StaticTestFramework : public MultipleWorldsTestFramework {
  protected:

  StaticTestFramework():
    MultipleWorldsTestFramework()
  {}
  virtual ~StaticTestFramework()
  {}

  // does the static test with two shapes
  void TwoShapes(const collision_benchmark::Shape::Ptr& shape1,
                 const std::string& modelName1,
                 const collision_benchmark::Shape::Ptr& shape2,
                 const std::string& modelName2,
                 const std::vector<std::string>& engines);
};

#endif  // COLLISION_BENCHMARK_TEST_STATICTESTFRAMEWORK_H