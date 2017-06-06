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
#ifndef COLLISION_BENCHMARK_SERIALIZATION_H
#define COLLISION_BENCHMARK_SERIALIZATION_H

#include <boost/serialization/nvp.hpp>

namespace collision_benchmark
{

struct Vector3::access
{
  template<class Archive>
  static void serialize(Archive &ar, Vector3& v, const unsigned int version)
  {
    ar & BOOST_SERIALIZATION_NVP(v.x);
    ar & BOOST_SERIALIZATION_NVP(v.y);
    ar & BOOST_SERIALIZATION_NVP(v.z);
  }
};

struct Quaternion::access
{
  template<class Archive>
  static void serialize(Archive &ar, Quaternion &q, const unsigned int version)
  {
    ar & BOOST_SERIALIZATION_NVP(q.x);
    ar & BOOST_SERIALIZATION_NVP(q.y);
    ar & BOOST_SERIALIZATION_NVP(q.z);
    ar & BOOST_SERIALIZATION_NVP(q.w);
  }
};

struct BasicState::access
{
  template<class Archive>
  static void serialize(Archive &ar, BasicState &b, const unsigned int version)
  {
    ar & BOOST_SERIALIZATION_NVP(b.position);
    ar & BOOST_SERIALIZATION_NVP(b.rotation);
    ar & BOOST_SERIALIZATION_NVP(b.scale);
    ar & BOOST_SERIALIZATION_NVP(b.posEnabled);
    ar & BOOST_SERIALIZATION_NVP(b.rotEnabled);
    ar & BOOST_SERIALIZATION_NVP(b.scaleEnabled);
  }
};

}  // namespace collision_benchmark

namespace boost
{
namespace serialization
{
  template<class Archive>
  void serialize(Archive &ar, collision_benchmark::Vector3& b,
                 const unsigned int version)
  {
    collision_benchmark::Vector3::access::serialize(ar, b, version);
  }

  template<class Archive>
  void serialize(Archive &ar, collision_benchmark::Quaternion &b,
                 const unsigned int version)
  {
    collision_benchmark::Quaternion::access::serialize(ar, b, version);
  }

  template<class Archive>
  void serialize(Archive &ar, collision_benchmark::BasicState &b,
                 const unsigned int version)
  {
    collision_benchmark::BasicState::access::serialize(ar, b, version);
  }

} // namespace serialization
} // namespace boost

#endif  // COLLISION_BENCHMARK_SERIALIZATION_H
