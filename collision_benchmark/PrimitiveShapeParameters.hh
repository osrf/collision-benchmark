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

#ifndef COLLISION_BENCHMARK_PRIMITIVESHAPE_PARAMETERS_H
#define COLLISION_BENCHMARK_PRIMITIVESHAPE_PARAMETERS_H

#include <collision_benchmark/Exception.hh>
#include <memory>

namespace collision_benchmark
{

/**
 * Interface for parameters that describe a primitive, such as radius or length.
 * Primitives can have a subset of parameters as defined in the type \e ParameterType,
 * all parameters are encoded as double values.
 * Subclasses ensure that the correct type of parameter is used with the getter and setter
 * and throw exceptions upon inconsistent use.
 *
 * \author Jennifer Buehler
 * \date October 2016
 */
class PrimitiveShapeParameters
{
  public: typedef std::shared_ptr<PrimitiveShapeParameters> Ptr;
  public: typedef std::shared_ptr<const PrimitiveShapeParameters> ConstPtr;

  public: PrimitiveShapeParameters(){}
  public: virtual ~PrimitiveShapeParameters(){}

  // Parameter values for a variety of primitive types.
  // VALX/Y/Z can be used for any primitive-specific type with x/y/z values,
  // such as a normal.
  public: enum ParameterType { RADIUS, LENGTH, VALX, VALY, VALZ};

  // Returns the value of the primitive parameter in \e result.
  // Throws a collision_benchmark::Exception if the primitive type does
  // not have the given \e type of paramter.
  public: virtual double Get(const ParameterType& type) = 0;
  // Sets the primitive parameter of this type.

  // Throws a collision_benchmark::Exception if the primitive type does
  // not have the given \e type of paramter.
  public: virtual void Set(const ParameterType& type, const double& val) = 0;

  // Clones the parameters
  public: virtual Ptr Clone() const = 0;
};

/**
 * All primitives which have only a radius
 */
template<typename Float=double>
class RadiusParameter: public PrimitiveShapeParameters
{
  public: typedef PrimitiveShapeParameters Super;
  public: explicit RadiusParameter(const Float& radius):
    radius(radius){}
  public: RadiusParameter(const RadiusParameter<Float>& o):
    radius(o.radius){}

  public: virtual double Get(const Super::ParameterType& type)
  {
    if (type!=RADIUS) THROW_EXCEPTION("RadiusParameter does hot have the type "<<type);
    return radius;
  }
  public: virtual void Set(const Super::ParameterType& type, const double& val)
  {
    if (type!=RADIUS) THROW_EXCEPTION("RadiusParameter does hot have the type "<<type);
    radius=val;
  }
  public: virtual Ptr Clone() const
  {
    return Ptr(new RadiusParameter(radius));
  }
  protected: Float radius;
};


/**
 * All primitives which have a radius and another value such as a value or height
 */
template<typename Float=double>
class RadiusAndValueParameter: public RadiusParameter<Float>
{
  private: typedef RadiusParameter<Float> Super;
  public: explicit RadiusAndValueParameter(const Float& radius, const Float& value):
    Super(radius),
    value(value){}
  public: RadiusAndValueParameter(const RadiusAndValueParameter<Float>& o):
    Super(o),
    value(o.value){}

  public: virtual double Get(const typename Super::ParameterType& type)
  {
    if (type!=Super::RADIUS && type!=Super::LENGTH) THROW_EXCEPTION("RadiusAndValueParameter does hot have the type "<<type);
    if (type==Super::RADIUS) return Super::Get(type);
    return value;
  }
  public: virtual void Set(const typename Super::ParameterType& type, const double& val)
  {
    if (type!=Super::RADIUS && type!=Super::LENGTH) THROW_EXCEPTION("RadiusAndValueParameter does hot have the type "<<type);
    if (type==Super::RADIUS) Super::Set(type, val);
    else value=val;
  }
  public: virtual typename Super::Ptr Clone() const
  {
    return typename Super::Ptr(new RadiusAndValueParameter(Super::radius, value));
  }

  protected: Float value;
};


/**
 * All primitives which have parameters related to 3 dimensions (x,y,z)
 */
template<typename Float=double>
class Dim3Parameter: public PrimitiveShapeParameters
{
  private: typedef PrimitiveShapeParameters Super;
  public: explicit Dim3Parameter(const Float& x_, const Float& y_, const Float& z_):
    x(x_),
    y(y_),
    z(z_)
  {}
  public: Dim3Parameter(const Dim3Parameter& o):
    x(o.x),
    y(o.y),
    z(o.z)
  {}

  public: virtual double Get(const typename Super::ParameterType& type)
  {
    if (type==VALX)
    {
      return x;
    }
    else if (type==VALY)
    {
      return y;
    }
    else if (type==VALZ)
    {
      return y;
    }
    THROW_EXCEPTION("Dim3Parameter does hot have the type "<<type);
  }
  public: virtual void Set(const typename Super::ParameterType& type, const double& val)
  {
    if (type==VALX)
    {
      x=val;
    }
    else if (type==VALY)
    {
      y=val;
    }
    else if (type==VALZ)
    {
      y=val;
    }
    else
    {
      THROW_EXCEPTION("Dim3Parameter does hot have the type "<<type);
    }
  }
  public: virtual typename Super::Ptr Clone() const
  {
    return typename Super::Ptr(new Dim3Parameter(x,y,z));
  }

  protected: Float x, y, z;
};

/**
 * Plane parameters using VALX/VALY/VALZ for the normal and LENGTH
 * for the distance from the origin.
 */
template<typename Float=double>
class PlaneParameter: public Dim3Parameter<double>
{
  private: typedef Dim3Parameter<double> Super;
  public: explicit PlaneParameter(const Float& x, const Float& y, const Float& z, const Float& dist):
    Super(x,y,z),
    distance(dist)
  {}
  public: PlaneParameter(const PlaneParameter& o):
    Super(o),
    distance(o.distance)
  {}

  public: virtual double Get(const typename Super::ParameterType& type)
  {
    if (type==LENGTH)
        return distance;
    return Super::Get(type);
  }

  public: virtual void Set(const typename Super::ParameterType& type, const double& val)
  {
    if (type==LENGTH)
      distance=val;
    Super::Set(type,val);
  }
  public: virtual typename Super::Ptr Clone() const
  {
    return typename Super::Ptr(new PlaneParameter(x,y,z, distance));
  }
  protected: Float distance;
};



}  // namespace

#endif  // COLLISION_BENCHMARK_PRIMITIVESHAPE_PARAMETERS_H
