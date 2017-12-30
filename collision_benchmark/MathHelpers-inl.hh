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
 * Date: 2017
 */

#include <collision_benchmark/MathHelpers.hh>
#include <algorithm>

//////////////////////////////////////////////////////////////////////////////
template<typename Float>
collision_benchmark::Vector3
collision_benchmark::Conv(const ignition::math::Vector3<Float>& v)
{
  return collision_benchmark::Vector3(v.X(), v.Y(), v.Z());
}

//////////////////////////////////////////////////////////////////////////////
template<typename Float>
ignition::math::Vector3<Float>
collision_benchmark::ConvIgn(const collision_benchmark::Vector3& v)
{
  return ignition::math::Vector3<Float>(v.x, v.y, v.z);
}

//////////////////////////////////////////////////////////////////////////////
template<typename FloatRet, typename Float>
ignition::math::Vector3<FloatRet>
collision_benchmark::ConvIgn(const ignition::math::Vector3<Float>& v)
{
  // no conversion required
  return v;
}

//////////////////////////////////////////////////////////////////////////////
template<typename Float>
collision_benchmark::Quaternion
collision_benchmark::Conv(const ignition::math::Quaternion<Float>& v)
{
  return collision_benchmark::Quaternion(v.X(), v.Y(), v.Z(), v.W());
}

//////////////////////////////////////////////////////////////////////////////
template<typename Float>
ignition::math::Quaternion<Float>
collision_benchmark::ConvIgn(const collision_benchmark::Quaternion &v)
{
  return ignition::math::Quaternion<Float>(v.w, v.x, v.y, v.z);
}

//////////////////////////////////////////////////////////////////////////////
template<typename FloatRet, typename Float>
ignition::math::Quaternion<FloatRet>
collision_benchmark::ConvIgn(const ignition::math::Quaternion<Float>& q)
{
  // no conversion required
  return q;
}

//////////////////////////////////////////////////////////////////////////////
template<typename Float>
ignition::math::Matrix4<Float>
collision_benchmark::GetMatrix(const collision_benchmark::Vector3& p,
                                       const collision_benchmark::Quaternion &q)
{
  ignition::math::Vector3<Float> pos(ConvIgn<Float>(p));
  ignition::math::Quaternion<Float> quat(ConvIgn<Float>(q));
  ignition::math::Matrix4<Float> m(quat);
  // Translate() only sets the position to the last column
  m.Translate(pos);
  return m;
}

//////////////////////////////////////////////////////////////////////////////
template<typename Float>
ignition::math::Matrix4<Float>
collision_benchmark::GetMatrix(const ignition::math::Vector3<Float>& pos,
            const ignition::math::Quaternion<Float>& quat)
{
  ignition::math::Matrix4<Float> m(quat);
  // Translate() only sets the position to the last column
  m.Translate(pos);
  return m;
}

//////////////////////////////////////////////////////////////////////////////
template<typename Float1, typename Float2>
bool
collision_benchmark::EqualFloats(const Float1& f1, const Float2& f2,
                                 const double &t)
{
  return fabs(f1-f2) < t;
}

//////////////////////////////////////////////////////////////////////////////
template<typename Float>
bool
collision_benchmark::EqualVectors(const ignition::math::Vector3<Float>& v1,
                  const ignition::math::Vector3<Float>& v2, const double &t)
{
  return EqualFloats(v1.X(), v2.X(), t)
      && EqualFloats(v1.Y(), v2.Y(), t)
      && EqualFloats(v1.Z(), v2.Z(), t);
}

// Helper function for ignition::math::Vector3.
// Unfortunately, we can't do this:
//  ``v[i] = ...``
// because ignition::math::Vector3::operator[] returns no l-value.
// This function achieves this. Supported are idx in range [0..2].
template<typename Float>
void SetIdx(const int idx,
            const Float val,
            ignition::math::Vector3<Float>& v)
{
  uint32_t index = idx < 0 ? 0u : idx > 2 ? 2 : idx;
  switch (index)
  {
    case 0:
      {
        v.X(val);
        break;
      }
    case 1:
      {
        v.Y(val);
        break;
      }
    case 2:
      {
        v.Z(val);
        break;
      }
  }
}

//////////////////////////////////////////////////////////////////////////////
template<typename Float>
void collision_benchmark::ProjectAABBOnAxis
        (const ignition::math::Vector3<Float>& aabbMin,
         const ignition::math::Vector3<Float>& aabbMax,
         const ignition::math::Vector3<Float>& projAxis,
         Float& onAxisMin,
         Float& onAxisMax)
{
    // make sure to use normalized axis
    ignition::math::Vector3<Float> axis(projAxis);
    axis.Normalize();
    Float xCoords[2] = { aabbMin.X(), aabbMax.X() };
    Float yCoords[2] = { aabbMin.Y(), aabbMax.Y() };
    Float zCoords[2] = { aabbMin.Z(), aabbMax.Z() };

    onAxisMin = std::numeric_limits<Float>::max();
    onAxisMax = -onAxisMin;
    for (int x = 0; x < 2; ++x)
      for (int y = 0; y < 2; ++y)
        for (int z = 0; z < 2; ++z)
        {
          ignition::math::Vector3<Float> v(xCoords[x], yCoords[y], zCoords[z]);
          Float projLength = v.Dot(axis);
          onAxisMin = std::min(onAxisMin, projLength);
          onAxisMax = std::max(onAxisMax, projLength);
        }
}

//////////////////////////////////////////////////////////////////////////////
template<typename Float>
void collision_benchmark::UpdateAABB
        (const ignition::math::Vector3<Float>& initialMin,
         const ignition::math::Vector3<Float>& initialMax,
         const ignition::math::Matrix4<Float>& transform,
         ignition::math::Vector3<Float>& newMin,
         ignition::math::Vector3<Float>& newMax)
{
#if 0
    // brute force implementation
    Float xCoords[2] = { initialMin.X(), initialMax.X() };
    Float yCoords[2] = { initialMin.Y(), initialMax.Y() };
    Float zCoords[2] = { initialMin.Z(), initialMax.Z() };

    newMin = ignition::math::Vector3<Float>(FLT_MAX, FLT_MAX, FLT_MAX);
    newMax = -newMin;
    for (int x = 0; x < 2; ++x)
      for (int y = 0; y < 2; ++y)
        for (int z = 0; z < 2; ++z)
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

    // for all 3 axes
    for (int i = 0; i < 3; ++i)
    {
      // Start by adding in translation
      SetIdx(i, transform(i, 3), newMin);
      SetIdx(i, transform(i, 3), newMax);
      // form extent by summing smaller and larger terms respectively
      for (int j = 0; j < 3; ++j)
      {
        Float e = transform(i, j) * initialMin[j];
        Float f = transform(i, j) * initialMax[j];
        if (e < f)
        {
          SetIdx(i, newMin[i] + e, newMin);
          SetIdx(i, newMax[i] + f, newMax);
        }
        else
        {
          SetIdx(i, newMin[i] + f, newMin);
          SetIdx(i, newMax[i] + e, newMax);
        }
      }
    }
#endif
}

//////////////////////////////////////////////////////////////////////////////
template<typename Float>
bool collision_benchmark::SegmentsOverlap(const Float min1, const Float max1,
                                          const Float min2, const Float max2,
                                          Float *overlapFact)
{
  if (overlapFact) *overlapFact=0;

  // compute the overlap between both points.
  Float diff;
  if (min1 < min2) diff = std::min(max1, max2) - min2;
  else diff = std::min(max2, max1) - min1;

  static Float zeroEps = 1e-07;
  if (diff <= zeroEps)
  {
    // no overlap
    return false;
  }

  Float minLen = std::min(max1 - min1, max2 - min2);
  assert(minLen >= 0);

  // zero length of one side can always be considered an overlap
  // if the overlap was not excluded yet above
  if (minLen < zeroEps)
  {
    if (overlapFact) *overlapFact = 1;
    return true;
  }

  if (!overlapFact) return true;

  Float fact = diff / minLen;
  assert(fact > 0);
  assert(fact <= 1.0);
  *overlapFact = fact;
  return true;
}
