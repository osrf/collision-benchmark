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
#ifndef COLLISION_BENCHMARK_MATH_HELPERS
#define COLLISION_BENCHMARK_MATH_HELPERS

#include <string>
#include <collision_benchmark/BasicTypes.hh>

#include <ignition/math/Vector3.hh>
#include <ignition/math/Matrix4.hh>

namespace collision_benchmark
{


template<typename Float>
collision_benchmark::Vector3 Conv(const ignition::math::Vector3<Float>& v)
{
  return collision_benchmark::Vector3(v.X(), v.Y(), v.Z());
}

template<typename Float>
ignition::math::Vector3<Float> ConvIgn(const collision_benchmark::Vector3& v)
{
  return ignition::math::Vector3<Float>(v.x, v.y, v.z);
}

template<typename Float>
ignition::math::Vector3<Float> ConvIgn(const ignition::math::Vector3<Float>& v)
{
  // no conversion required
  return v;
}

template<typename Float>
collision_benchmark::Quaternion Conv(const ignition::math::Quaternion<Float>& v)
{
  return collision_benchmark::Quaternion(v.X(), v.Y(), v.Z(), v.W());
}


template<typename Float>
ignition::math::Quaternion<Float> ConvIgn(const collision_benchmark::Quaternion& v)
{
  return ignition::math::Quaternion<Float>(v.w, v.x, v.y, v.z);
}

template<typename Float>
ignition::math::Quaternion<Float>
ConvIgn(const ignition::math::Quaternion<Float>& q)
{
  // no conversion required
  return q;
}

template<typename Float>
ignition::math::Matrix4<Float> GetMatrix(const collision_benchmark::Vector3& p,
                                       const collision_benchmark::Quaternion& q)
{
  ignition::math::Vector3<Float> pos(ConvIgn<Float>(p));
  ignition::math::Quaternion<Float> quat(ConvIgn<Float>(q));
  ignition::math::Matrix4<Float> m(quat);
  // Translate() only sets the position to the last column
  m.Translate(pos);
  return m;
}

template<typename Float>
ignition::math::Matrix4<Float>
  GetMatrix(const ignition::math::Vector3<Float>& pos,
            const ignition::math::Quaternion<Float>& quat)
{
  ignition::math::Matrix4<Float> m(quat);
  // Translate() only sets the position to the last column
  m.Translate(pos);
  return m;
}

/**
 * \brief Transforms the axis-aligned bounding box by \e transform.
 */
template<typename Float>
void UpdateAABB(const ignition::math::Vector3<Float>& initialMin,
                const ignition::math::Vector3<Float>& initialMax,
                const ignition::math::Matrix4<Float>& transform,
                ignition::math::Vector3<Float>& newMin,
                ignition::math::Vector3<Float>& newMax)
{
#if 1
    // brute force implementation
    Float xCoords[2] = { initialMin.X(), initialMax.X() };
    Float yCoords[2] = { initialMin.Y(), initialMax.Y() };
    Float zCoords[2] = { initialMin.Z(), initialMax.Z() };

    newMin = ignition::math::Vector3<Float>(FLT_MAX, FLT_MAX, FLT_MAX);
    newMax = -newMin;
    for (int x=0; x<2; ++x)
      for (int y=0; y<2; ++y)
        for (int z=0; z<2; ++z)
        {
          ignition::math::Vector3<Float> v(xCoords[x], yCoords[y], zCoords[z]);
          v = transform * v;
          newMin = ignition::math::Vector3<Float>(std::min(v.X(), newMin.X()),
                     std::min(v.Y(), newMin.Y()),
                     std::min(v.Z(), newMin.Z()));
          newMax = ignition::math::Vector3<Float>(std::max(v.X(), newMax.X()),
                     std::max(v.Y(), newMax.Y()),
                     std::max(v.Z(), newMax.Z()));
        }
#else
    // alternative implementation: see book "real-time collision detection"
    // by Ericson, Vol 1, chapter 4.2.6, p. 86, function UpdateAABB

#endif
}

}  // namespace

#endif  // COLLISION_BENCHMARK_MATH_HELPERS
