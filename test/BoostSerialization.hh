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
#ifndef COLLISION_BENCHMARK_TEST_SERIALIZATION_H
#define COLLISION_BENCHMARK_TEST_SERIALIZATION_H

#include "CollidingShapesConfiguration.hh"

#include <collision_benchmark/BoostSerialization.hh>
#include <boost/serialization/vector.hpp>

namespace collision_benchmark
{
namespace test
{

struct CollidingShapesConfiguration::access
{
  template<class Archive>
  static void serialize(Archive &ar, CollidingShapesConfiguration &conf,
                        const unsigned int version)
  {
    ar & BOOST_SERIALIZATION_NVP(conf.models);
    ar & BOOST_SERIALIZATION_NVP(conf.shapes);
    ar & BOOST_SERIALIZATION_NVP(conf.modelState1);
    ar & BOOST_SERIALIZATION_NVP(conf.modelState2);
  }
};

}  // namespace test
}  // namespace collision_benchmark

namespace boost
{
namespace serialization
{
  template<class Archive>
  void serialize(Archive &ar,
                 collision_benchmark::test::CollidingShapesConfiguration &conf,
                 const unsigned int version)
  {
    collision_benchmark::test::CollidingShapesConfiguration
      ::access::serialize(ar, conf, version);
  }

} // namespace serialization
} // namespace boost

#endif  // COLLISION_BENCHMARK_TEST_SERIALIZATION_H
