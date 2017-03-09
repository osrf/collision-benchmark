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
  struct AABB
  {
    typedef ignition::math::Vector3d Vec3;
    Vec3 min, max;
  };

  StaticTestFramework():
    MultipleWorldsTestFramework()
  {}
  virtual ~StaticTestFramework()
  {}

  // Loads the two shapes. Call PrepareWorld() before.
  //
  // Throws gtest assertions so needs to be called from top-level
  // test function (nested function calls will not work correctly)
  void LoadShapes(const collision_benchmark::Shape::Ptr& shape1,
                 const std::string& modelName1,
                 const collision_benchmark::Shape::Ptr& shape2,
                 const std::string& modelName2);

  // loads the empty world with all the given engines and creates
  // the world manager.
  //
  // Throws gtest assertions so needs to be called from top-level
  // test function (nested function calls will not work correctly)
  void PrepareWorld(const std::vector<std::string>& engines);



  // Does the static test in which the two models (must already have
  // been loaded) are moved to various poses, and all engines have to
  // agree on the collision state (boolean collision).
  // You need to use a loading function such as LoadShapes() before.
  //
  // Throws gtest assertions so needs to be called from top-level
  // test function (nested function calls will not work correctly)
  void TwoModels(const std::string& modelName1,
                 const std::string& modelName2);
private:

  // checks that AABB of model 1 and 2 are the same in all worlds and
  // returns the two AABBs
  // \return true if worlds are consistent, falsle otherwise
  bool GetAABBs(const std::string& modelName1,
                const std::string& modelName2,
                AABB& m1, AABB& m2);

};

#endif  // COLLISION_BENCHMARK_TEST_STATICTESTFRAMEWORK_H
