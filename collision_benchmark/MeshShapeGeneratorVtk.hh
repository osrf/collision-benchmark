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
 * Date: March 2017
 */

#ifndef COLLISION_BENCHMARK_MESHSHAPEGENERATORVTK_H
#define COLLISION_BENCHMARK_MESHSHAPEGENERATORVTK_H

#include <collision_benchmark/MeshShapeGenerator.hh>

namespace collision_benchmark
{
/**
 * \brief Interface to generate triangle mesh representations out of shapes
 * \author Jennifer Buehler
 * \date March 2017
 */
template<typename VertexPrecision_ = float>
class MeshShapeGeneratorVtk
  : public MeshShapeGenerator<VertexPrecision_>
{
  public: typedef VertexPrecision_ VertexPrecision;
  private: typedef MeshShapeGeneratorVtk<VertexPrecision> Self;
  private: typedef MeshShapeGenerator<VertexPrecision> Super;
  public: typedef std::shared_ptr<Self> Ptr;
  public: typedef std::shared_ptr<const Self> ConstPtr;

  public: typedef typename Super::TriMeshData TriMeshData;
  public: typedef typename Super::TriMeshDataPtr TriMeshDataPtr;

  public: virtual TriMeshDataPtr MakeSphere(const double radius,
                                            const unsigned int theta,
                                            const unsigned int phi,
                                            const bool latLongTessel) const;

  public: virtual TriMeshDataPtr MakeCylinder(const double radius,
                                              const double height,
                                              const unsigned int resolution,
                                              const bool capping) const;

  public: virtual TriMeshDataPtr MakeBox(const double x,
                                         const double y,
                                         const double z) const;

  public: virtual TriMeshDataPtr MakeBox(const double xMin, const double xMax,
                                         const double yMin, const double yMax,
                                         const double zMin,
                                         const double zMax) const;

  public: virtual TriMeshDataPtr MakeCone(const double radius,
                                          const double height,
                                          const unsigned int resolution,
                                          const double angle_deg,
                                          const bool capping,
                                          const double dir_x = 0,
                                          const double dir_y = 0,
                                          const double dir_z = 1) const;

  public: virtual TriMeshDataPtr MakeDisk(const double innerRadius,
                                          const double outerRadius,
                                          const unsigned int radialRes,
                                          const unsigned int circumRes) const;

  public: virtual TriMeshDataPtr MakeEllipsoid(const double xRad,
                                               const double yRad,
                                               const double zRad,
                                               const unsigned int uRes,
                                               const unsigned int vRes) const;

  public: virtual TriMeshDataPtr MakeTorus(const double ringRadius,
                                           const double crossRadius,
                                           const unsigned int uRes,
                                           const unsigned int vRes) const;
};  // class
}  // namespace

#include <collision_benchmark/MeshShapeGeneratorVtk-inl.hh>

#endif  // COLLISION_BENCHMARK_MESHSHAPEGENERATORVTK_H
