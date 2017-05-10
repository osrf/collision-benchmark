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
#include <collision_benchmark/PrimitiveShape.hh>

using collision_benchmark::PrimitiveShape;

PrimitiveShape * PrimitiveShape::CreateBox(double width,
                                           double height,
                                           double depth)
{
  PrimitiveShapeParameters::Ptr
    param(new Dim3Parameter<ParameterFloat>(width, height, depth));
  return new PrimitiveShape(BOX, param);
}


PrimitiveShape * PrimitiveShape::CreateBox(const Vector3& size)
{
  return CreateBox(size.X(), size.Y(), size.Z());
}

PrimitiveShape * PrimitiveShape::CreateSphere(double radius)
{
  PrimitiveShapeParameters::Ptr
    param(new RadiusParameter<ParameterFloat>(radius));
  return new PrimitiveShape(SPHERE, param);
}

PrimitiveShape * PrimitiveShape::CreateCylinder(double radius, double height)
{
  PrimitiveShapeParameters::Ptr
    param(new RadiusAndValueParameter<ParameterFloat>(radius, height));
  return new PrimitiveShape(CYLINDER, param);
}

PrimitiveShape * PrimitiveShape::CreatePlane(const Vector3& normal,
                                             const Vector2& dim)
{
  return CreatePlane(normal.X(), normal.Y(), normal.Z(),
                     dim.X(), dim.Y());
}

PrimitiveShape * PrimitiveShape::CreatePlane(double nx, double ny, double nz,
                                    double dx, double dy)
{
  PrimitiveShapeParameters::Ptr
    param(new BoundedPlaneParameter<ParameterFloat>(nx, ny, nz, 0, dx, dy));
  return new PrimitiveShape(PLANE, param);
}

sdf::ElementPtr
PrimitiveShape::GetShapeSDF(bool detailed,
                            const std::string& resourceDir,
                            const std::string& resourceSubDir,
                            const bool useFullPath) const
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
      geomElem->AddValue("double",
                 std::to_string(params->Get(PrimitiveShapeParameters::RADIUS)),
                 true, "radius of cylinder");
      break;
    }

    case CYLINDER:
    {
      geomChild->SetName("cylinder");
      geomElem->SetName("radius");
      geomElem->AddValue("double",
                 std::to_string(params->Get(PrimitiveShapeParameters::RADIUS)),
                 true, "radius of cylinder");
      sdf::ElementPtr geomElem2(new sdf::Element());
      geomElem2->SetName("length");
      geomElem2->AddValue("double",
                std::to_string(params->Get(PrimitiveShapeParameters::LENGTH)),
                true, "length of cylinder");
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
      geomElem2->AddValue("vector2d", valsS.str(), true,
                          "Length of each side of the plane");
      break;
    }

    default:
    {
      THROW_EXCEPTION("Unsupported primitive type in "
                      << "collision_benchmark::PrimitiveShape");
    }
  }
  return geometry;
}
