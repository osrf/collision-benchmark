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

template<typename FloatRet, typename Float>
ignition::math::Vector3<FloatRet>
    ConvIgn(const ignition::math::Vector3<Float>& v);

template<typename Float>
collision_benchmark::Quaternion
  Conv(const ignition::math::Quaternion<Float>& v);

template<typename Float>
ignition::math::Quaternion<Float>
  ConvIgn(const collision_benchmark::Quaternion &v);

template<typename FloatRet, typename Float>
ignition::math::Quaternion<FloatRet>
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

// \brief compares the individual coordinates (x,y,z) of the vectors
// against the tolerance using EqualFloats().
template<typename Float>
bool EqualVectors(const ignition::math::Vector3<Float>& v1,
                  const ignition::math::Vector3<Float>& v2,
                  const double &t);


// \brief Transforms the axis-aligned bounding box by \e transform.
// If the box is rotated by \e transform, the dimensions of the box may change.
template<typename Float>
void UpdateAABB(const ignition::math::Vector3<Float>& initialMin,
                const ignition::math::Vector3<Float>& initialMax,
                const ignition::math::Matrix4<Float>& transform,
                ignition::math::Vector3<Float>& newMin,
                ignition::math::Vector3<Float>& newMax);

// \brief Projects an axis-aligned bounding box onto the axis \e projAxis.
// \param[in] aabbMin min point of AABB
// \param[in] aabbMax max point of AABB
// \param[in] projAxis axis to project on (goes through origin)
// \param[out] onAxisMin minimum point of axis. \e projAxis * \e onAxisMin
//    will be the point on the axis (the axis goes through the origin)
// \param[out] onAxisMax max point, like \e onAxisMin
template<typename Float>
void ProjectAABBOnAxis(const ignition::math::Vector3<Float>& aabbMin,
                       const ignition::math::Vector3<Float>& aabbMax,
                       const ignition::math::Vector3<Float>& projAxis,
                       Float& onAxisMin,
                       Float& onAxisMax);

// \brief Tests if two segments along a 1D line overlap
// and returns the overlap if desired.
// \param[in] min1 minimum of first point
// \param[in] max1 maximum of first point
// \param[in] min2 minimum of second point
// \param[in] max2 maximum of second point
// \param[out] overlapFact the overlap factor. If NULL, it is not computed.
// \return true if there is an overlap
//////////////////////////////////////////////////////////////////////////////
template<typename Float>
bool SegmentsOverlap(const Float min1, const Float max1,
                     const Float min2, const Float max2,
                     Float *overlapFact = NULL);

}  // namespace

#include <collision_benchmark/MathHelpers-inl.hh>

#endif  // COLLISION_BENCHMARK_MATH_HELPERS
