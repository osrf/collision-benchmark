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
#ifndef COLLISION_BENCHMARK_SHAPE
#define COLLISION_BENCHMARK_SHAPE

#include <sdf/sdf.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Vector2.hh>
#include <memory>

namespace collision_benchmark
{

/**
 * \brief Basic interface for a shape, which can be either a primitive
 * or a mesh. All supported types are defined in the type \e Type.
 *
 * \author Jennifer Buehler
 * \date October 2016
 */
class Shape
{
  public: typedef std::shared_ptr<Shape> Ptr;
  public: typedef std::shared_ptr<const Shape> ConstPtr;
  public: typedef enum Types_{ BOX, SPHERE, CYLINDER, PLANE, MESH} Type;

  public: typedef ignition::math::Pose3<double> Pose3;
  public: typedef ignition::math::Vector3<double> Vector3;
  public: typedef ignition::math::Vector2<double> Vector2;

  public: Shape(const Type& type_):
          type(type_) {}
  public: Shape(const Shape& o):
          type(o.type),
          pose(o.pose) {}
  public: virtual ~Shape(){}

  public: inline Type GetType() const { return type; }
  public: inline void SetPose(const Pose3& p) { pose=p; }
  public: inline const Pose3& GetPose() const { return pose; }
  public: inline Pose3& GetPose() { return pose; }

  /// returns the pose as SDF element
  public: sdf::ElementPtr GetPoseSDF() const;

  /// returns the shape part as SDF element.
  /// \param detailed set to true to get the most detailed shape, or to false
  ///     to get an approximated shape, e.g. to use as collision shape.
  public: virtual sdf::ElementPtr GetShapeSDF(bool detailed=true) const = 0;

  private: Type type;
  private: Pose3 pose;
};


}  // namespace

#endif  // COLLISION_BENCHMARK_SHAPE
