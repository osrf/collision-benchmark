/*
 * Copyright (C) 2014-2016 Open Source Robotics Foundation
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
#ifndef COLLISION_BENCHMARK_TEST_COLLIDINGSHAPESCONFIGURATION_H
#define COLLISION_BENCHMARK_TEST_COLLIDINGSHAPESCONFIGURATION_H

#include <collision_benchmark/BasicTypes.hh>
namespace collision_benchmark
{
namespace test
{

/**
 * \brief Details of one saved configuration for CollideShapesTestFramework test
 * \author Jennifer Buehler
 * \date May 2017
 */
class CollidingShapesConfiguration
{
  // helper struct to allow external boost serialization
  public: struct access;

  public: std::vector<std::string> models;
  public: std::vector<std::string> shapes;
  public: collision_benchmark::BasicState model1State, model2State;
};

}  // namespace test
}  // namespace collision_benchmark

#endif  // COLLISION_BENCHMARK_TEST_COLLIDINGSHAPESCONFIGURATION_H
