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

#ifndef COLLISION_BENCHMARK_PRIMITIVESHAPE
#define COLLISION_BENCHMARK_PRIMITIVESHAPE

#include <collision_benchmark/Shape.hh>
#include <collision_benchmark/PrimitiveShapeParameters.hh>

#include <memory>

namespace collision_benchmark
{

/**
 * \brief A shape which is one of the primitives of the types BOX, SPHERE, CYLINDER, PLANE
 *
 * This serves also as factory class for creating instances, use the static factory methods
 * instead of the constructor.
 *
 * \author Jennifer Buehler
 * \date December 2016
 */
class PrimitiveShape: public Shape
{
  private: typedef double ParameterFloat;
  public: typedef std::shared_ptr<PrimitiveShape> Ptr;
  public: typedef std::shared_ptr<const PrimitiveShape> ConstPtr;

  private: PrimitiveShape(const Type& type,
                          const PrimitiveShapeParameters::Ptr& params_):
            Shape(type),
            params(params_) {}

  public: PrimitiveShape(const PrimitiveShape& o):
            Shape(o),
            params(o.params) {}

  public: virtual ~PrimitiveShape(){}

  // Creates a box
  public: static PrimitiveShape * CreateBox(double width, double height, double depth);

  // Creates a box  by calling the other CreateBox which takes the dimensions as
  // individual arguments
  public: static PrimitiveShape * CreateBox(const Vector3& size);

  // Creates a sphere
  public: static PrimitiveShape * CreateSphere(double radius);

  // Creates a cylinder
  public: static PrimitiveShape * CreateCylinder(double radius, double height);

  // Creates a plane.
  // \param normal normal of plane
  // \param dim dimensions (bounds) of plane in x and y direction.
  //    May be set to infinite, though this may have limits with the SDF.
  public: static PrimitiveShape * CreatePlane(const Vector3& normal, const Vector2& dim);

  // Creates a plane by calling the other CreatePlane() functon which uses the vector parameters
  public: static PrimitiveShape * CreatePlane(double nx, double ny, double nz,
                                              double dx, double dy);

  // Documentation inherited from parent class
  public: virtual sdf::ElementPtr GetShapeSDF(bool detailed=true, bool uriOnlyWithSubdir=false) const;

  private: PrimitiveShapeParameters::Ptr params;
};


}  // namespace

#endif  // COLLISION_BENCHMARK_PRIMITIVESHAPE
