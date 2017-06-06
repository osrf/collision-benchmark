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

  public: Shape(const Type &type_):
          type(type_) {}
  public: Shape(const Shape &o):
          type(o.type),
          pose(o.pose) {}
  public: virtual ~Shape(){}

  public: inline Type GetType() const { return type; }
  public: inline void SetPose(const Pose3& p) { pose = p; }
  public: inline const Pose3& GetPose() const { return pose; }
  public: inline Pose3& GetPose() { return pose; }

  /// returns the pose as SDF element
  public: sdf::ElementPtr GetPoseSDF() const;

  /// Returns the shape part as SDF element.
  ///
  /// Because some elements in the SDF file requires the data to be specified
  /// via a URI, GetShapeSDF() sometimes needs to write data to a file
  /// (e.g. for meshes).
  /// \sa SupportLowRes()
  /// \param detailed set to true to get the most detailed shape, or to false
  ///     to get an approximated shape, e.g. to use as collision shape.
  /// \param resourceDir if there are SDF elements which rely on data
  ///    being written to file, this method will write the data to file.
  ///    This parameter allows to set the directory where the file will be
  ///    written, together with \e resourceSubDir. The filename will be
  ///    generated.
  ///    If \e useFullPath is false, the URI in the SDF will be
  ///    ``file://<resourceSubDir>/<generated-filename>``.
  ///    If \e useFullPath is true, the URI in the SDF will be
  ///    ``file://<resourceDir>/<resourceSubDir>/<generated-filename>``.
  ///    If the combination of \e resourceDir and \e resourceSubDir is empty,
  ///    and the implementation requires a path, it is considered an error
  ///    and the function will return nullptr.
  ///    *Note:* For use with Gazebo, this location should be a path in the
  ///    GAZEBO_RESOURCE_PATH.
  /// \param resourceSubDir sub-directory path to add to \e resourceDir which
  ///   will appear in the path of the SDF URI if \e useFullPath is true.
  /// \param useFullPath use the absolute path in the URI instead of the
  ///   relative path only starting from \e resourceSubDir.
  public: virtual sdf::ElementPtr
                  GetShapeSDF(bool detailed = true,
                              const std::string &resourceDir = "/tmp/",
                              const std::string &resourceSubDir = "",
                              const bool useFullPath = false) const = 0;

  /// returns true if GetShapeSDF() returns a different mesh with parameter
  /// \e detailed set to false, in which case there is a low-res representation
  /// of the shape. If this returns false, there is only one (the detailed)
  /// representation.
  public: virtual bool SupportLowRes() const { return false; }

  private: Type type;
  private: Pose3 pose;
};


}  // namespace

#endif  // COLLISION_BENCHMARK_SHAPE
