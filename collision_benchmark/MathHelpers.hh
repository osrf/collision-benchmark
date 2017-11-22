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
/*
 * Author: Jennifer Buehler
 */
#ifndef COLLISION_BENCHMARK_MATH_HELPERS
#define COLLISION_BENCHMARK_MATH_HELPERS

#include <collision_benchmark/BasicTypes.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Matrix4.hh>
#include <string>

namespace collision_benchmark
{
template<typename Float>
collision_benchmark::Vector3 Conv(const ignition::math::Vector3<Float>& v);

template<typename Float>
ignition::math::Vector3<Float> ConvIgn(const collision_benchmark::Vector3& v);

template<typename Float>
ignition::math::Vector3<Float> ConvIgn(const ignition::math::Vector3<Float>& v);

template<typename Float>
collision_benchmark::Quaternion
  Conv(const ignition::math::Quaternion<Float>& v);

template<typename Float>
ignition::math::Quaternion<Float>
  ConvIgn(const collision_benchmark::Quaternion &v);

template<typename Float>
ignition::math::Quaternion<Float>
ConvIgn(const ignition::math::Quaternion<Float>& q);

template<typename Float>
ignition::math::Matrix4<Float>
  GetMatrix(const collision_benchmark::Vector3& p,
            const collision_benchmark::Quaternion &q);

template<typename Float>
ignition::math::Matrix4<Float>
  GetMatrix(const ignition::math::Vector3<Float>& pos,
            const ignition::math::Quaternion<Float>& quat);

template<typename Float1, typename Float2>
bool EqualFloats(const Float1& f1, const Float2& f2, const double &t);

template<typename Float>
bool EqualVectors(const ignition::math::Vector3<Float>& v1,
                  const ignition::math::Vector3<Float>& v2,
                  const double &t);


// \brief Transforms the axis-aligned bounding box by \e transform.
template<typename Float>
void UpdateAABB(const ignition::math::Vector3<Float>& initialMin,
                const ignition::math::Vector3<Float>& initialMax,
                const ignition::math::Matrix4<Float>& transform,
                ignition::math::Vector3<Float>& newMin,
                ignition::math::Vector3<Float>& newMax);

}  // namespace

#include <collision_benchmark/MathHelpers-inl.hh>

#endif  // COLLISION_BENCHMARK_MATH_HELPERS
