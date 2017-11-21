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

#include <memory>
#include <vector>
#include <string>

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

  public: typedef std::shared_ptr<CollidingShapesConfiguration> Ptr;
  public: typedef std::shared_ptr<const CollidingShapesConfiguration> ConstPtr;

  public: CollidingShapesConfiguration() {}
  public: CollidingShapesConfiguration(const std::vector<std::string>& _models,
                        const std::vector<std::string>& _shapes,
                        const collision_benchmark::BasicState &_modelState1 =
                              collision_benchmark::BasicState(),
                        const collision_benchmark::BasicState &_modelState2 =
                              collision_benchmark::BasicState()):
    models(_models),
    shapes(_shapes),
    modelState1(_modelState1),
    modelState2(_modelState2) {}

  public: CollidingShapesConfiguration(const CollidingShapesConfiguration &o):
    models(o.models),
    shapes(o.shapes),
    modelState1(o.modelState1),
    modelState2(o.modelState2) {}

  public: std::vector<std::string> models;
  public: std::vector<std::string> shapes;
  public: collision_benchmark::BasicState modelState1, modelState2;
};

}  // namespace test
}  // namespace collision_benchmark

#endif  // COLLISION_BENCHMARK_TEST_COLLIDINGSHAPESCONFIGURATION_H
