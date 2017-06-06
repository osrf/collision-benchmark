/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
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
 * Date: December 2016
 */

#include <collision_benchmark/MeshData.hh>

#include <random>
#include <vector>

template<typename VP, int FS>
void collision_benchmark::MeshData<VP, FS>::Perturb(const double min,
                                                    const double max,
                                                    const Vertex &center)
{
  static std::random_device r;
  static std::default_random_engine randEngine(r());

  // to generate random doubles between min and max with a uniform distribution
  std::uniform_real_distribution<double> dist(min, max);

  for (typename std::vector<Vertex>::iterator it = verts.begin();
       it != verts.end(); ++it)
  {
    Vertex &v = *it;
    Vertex diff = v - center;
    diff.Normalize();
    double randDisplace = dist(randEngine);
    v += diff * randDisplace;
  }
}

template<typename VP, int FS>
void collision_benchmark::MeshData<VP, FS>::Perturb(const double min,
                                                    const double max,
                                                    const Vertex &center,
                                                    const Vertex &dir)
{
  assert(dir.Length() > 1e-04);

  static std::random_device r;
  static std::default_random_engine randEngine(r());

  // normalized direction (just to be sure it is normal)
  Vertex dirNorm = dir;
  dirNorm.Normalize();

  // to generate random doubles between min and max with a uniform distribution
  std::uniform_real_distribution<double> dist(min, max);

  for (typename std::vector<Vertex>::iterator it = verts.begin();
       it != verts.end(); ++it)
  {
    Vertex &v = *it;

    // determine move direction which is orthogonal to the given line
    Vertex fromCenter = v - center;
    Vertex proj = dirNorm * (dirNorm.Dot(fromCenter));
    Vertex moveDir = fromCenter - proj;
    if (moveDir.Length() < 1e-02) continue;
    moveDir.Normalize();

    // if the move direction is not orthogonal to dir then it must
    // be on the line itself, in which case we won't perturb it.
    if (fabs(moveDir.Dot(dirNorm)) > 1e-04)
    {
      continue;
    }

    double randDisplace = dist(randEngine);
    v += moveDir * randDisplace;
  }
}
