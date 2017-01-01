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
  public: static PrimitiveShape * CreateBox(double width, double height, double depth)
          {
            PrimitiveShapeParameters::Ptr
              param(new Dim3Parameter<ParameterFloat>(width, height, depth));
            return new PrimitiveShape(BOX, param);
          }
  // Creates a box  by calling the other CreateBox which takes the dimensions as
  // individual arguments
  public: static PrimitiveShape * CreateBox(const Vector3& size)
          {
            return CreateBox(size.X(), size.Y(), size.Z());
          }

  // Creates a sphere
  public: static PrimitiveShape * CreateSphere(double radius)
          {
            PrimitiveShapeParameters::Ptr
              param(new RadiusParameter<ParameterFloat>(radius));
            return new PrimitiveShape(SPHERE, param);
          }
  // Creates a cylinder
  public: static PrimitiveShape * CreateCylinder(double radius, double height)
          {
            PrimitiveShapeParameters::Ptr
              param(new RadiusAndValueParameter<ParameterFloat>(radius, height));
            return new PrimitiveShape(CYLINDER, param);
          }

  // Creates a plane.
  // \param normal normal of plane
  // \param dim dimensions (bounds) of plane in x and y direction.
  //    May be set to infinite, though this may have limits with the SDF.
  public: static PrimitiveShape * CreatePlane(const Vector3& normal, const Vector2& dim)
          {
            return CreatePlane(normal.X(), normal.Y(), normal.Z(),
                               dim.X(), dim.Y());
          }
  // Creates a plane by calling the other CreatePlane() functon which uses the vector parameters
  public: static PrimitiveShape * CreatePlane(double nx, double ny, double nz,
                                              double dx, double dy)
          {
            PrimitiveShapeParameters::Ptr
              param(new BoundedPlaneParameter<ParameterFloat>(nx, ny, nz, 0, dx, dy));
            return new PrimitiveShape(PLANE, param);
          }

  public: virtual sdf::ElementPtr GetShapeSDF(bool detailed=true) const
          {
            sdf::ElementPtr geometry(new sdf::Element());
            geometry->SetName("geometry");
            sdf::ElementPtr geomChild(new sdf::Element());
            geometry->InsertElement(geomChild);
            sdf::ElementPtr geomElem(new sdf::Element());
            geomChild->InsertElement(geomElem);
            switch(GetType())
            {
              case BOX:
              {
                geomChild->SetName("box");
                geomElem->SetName("size");
                std::stringstream vals;
                vals<<params->Get(PrimitiveShapeParameters::DIMX)<<" "
                  <<params->Get(PrimitiveShapeParameters::DIMY)<<" "
                  <<params->Get(PrimitiveShapeParameters::DIMZ);
                geomElem->AddValue("vector3", vals.str(), true, "Box size");
                break;
              }
              case SPHERE:
              {
                geomChild->SetName("sphere");
                geomElem->SetName("radius");
                geomElem->AddValue("double", std::to_string(params->Get(PrimitiveShapeParameters::RADIUS)), true, "radius of cylinder");
                break;
              }

              case CYLINDER:
              {
                geomChild->SetName("cylinder");
                geomElem->SetName("radius");
                geomElem->AddValue("double", std::to_string(params->Get(PrimitiveShapeParameters::RADIUS)), true, "radius of cylinder");
                sdf::ElementPtr geomElem2(new sdf::Element());
                geomElem2->SetName("length");
                geomElem2->AddValue("double", std::to_string(params->Get(PrimitiveShapeParameters::LENGTH)), true, "length of cylinder");
                geomChild->InsertElement(geomElem2);
                break;
              }

              case PLANE:
              {
                geomChild->SetName("plane");

                geomElem->SetName("normal");
                std::stringstream valsN;
                valsN<<params->Get(PrimitiveShapeParameters::VALX)<<" "
                  <<params->Get(PrimitiveShapeParameters::VALY)<<" "
                  <<params->Get(PrimitiveShapeParameters::VALZ);
                geomElem->AddValue("vector3", valsN.str(), true, "Normal of the plane");

                sdf::ElementPtr geomElem2(new sdf::Element());
                geomElem2->SetName("size");
                geomChild->InsertElement(geomElem2);
                std::stringstream valsS;
                valsS<<params->Get(PrimitiveShapeParameters::DIMX)<<" "
                  <<params->Get(PrimitiveShapeParameters::DIMY);
//                  <<params->Get(PrimitiveShapeParameters::DIMZ);
                geomElem2->AddValue("vector2d", valsS.str(), true, "Length of each side of the plane");
                break;
              }

              default:
              {
                THROW_EXCEPTION("Unsupported primitive type in collision_benchmark::PrimitiveShape");
              }
            }
            return geometry;
          }
  private: PrimitiveShapeParameters::Ptr params;
};


}  // namespace

#endif  // COLLISION_BENCHMARK_PRIMITIVESHAPE
