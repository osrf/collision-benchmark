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

  public: static PrimitiveShape * CreateBox(double  width, double height, double depth)
          {
            PrimitiveShapeParameters::Ptr
              param(new Dim3Parameter<ParameterFloat>(width, height, depth));
            return new PrimitiveShape(BOX, param);
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
                vals<<params->Get(PrimitiveShapeParameters::VALX)<<" "
                  <<params->Get(PrimitiveShapeParameters::VALY)<<" "
                  <<params->Get(PrimitiveShapeParameters::VALZ);
                geomElem->AddValue("vector3", vals.str(), "0", "description");
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
