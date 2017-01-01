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

#ifndef COLLISION_BENCHMARK_MESHSHAPE
#define COLLISION_BENCHMARK_MESHSHAPE

#include <collision_benchmark/Shape.hh>
#include <collision_benchmark/MeshData.hh>

#include <memory>
#include <string>

namespace collision_benchmark
{

/**
 * \brief A simple Shape which is implements the MESH type for triangle meshes, not supporting textures.
 *
 * Uses double as vertex precision and supports only triangle meshes.
 *
 * Important note about GetShapeSDF() implementation: Because the SDF file requires the mesh data
 * to be at a URI, GetShapeSDF() needs to write the mesh data to a file.
 * The static variable MESH_OUT_DIR can be used to change the directory it to which it writes the files.
 *
 * \author Jennifer Buehler
 * \date December 2016
 */
class SimpleTriMeshShape: public Shape
{
  private: typedef double ParameterFloat;
  public: typedef std::shared_ptr<SimpleTriMeshShape> Ptr;
  public: typedef std::shared_ptr<const SimpleTriMeshShape> ConstPtr;

  public: typedef MeshData<float, 3> MeshDataT;
  public: typedef MeshDataT::Ptr MeshDataPtr;

  public: typedef MeshDataT::Vertex Vertex;
  public: typedef MeshDataT::Face Face;

  // The directory into which the mesh data is written when GetShapeSDF() is called.
  // This is required because SDF needs the meshes to be written in a file that it can reference.
  // For use with Gazebo, this should be a path in the GAZEBO_RESOURCE_PATH.
  public: static const std::string MESH_OUT_DIR;

  // The file extension to be used for all mesh data written to file with GetShapeSDF().
  // May not start with a dot!
  public: static const std::string MESH_EXT;

  /**
   * \param data__ the mesh data
   * \param name_ a unique name identiying this mesh shape. This only is important for when
   *      GetShapeSDF() is called, which needs to write the mesh data to a unique file name.
   */
  public: SimpleTriMeshShape(const MeshDataPtr& data_, const std::string& name_):
            Shape(MESH),
            data(data_),
            name(name_)  {}

  public: SimpleTriMeshShape(const SimpleTriMeshShape& o):
            Shape(o),
            data(o.data) {}

  public: virtual ~SimpleTriMeshShape(){}

  // Documentation inherited from parent class
  public: virtual sdf::ElementPtr GetShapeSDF(bool detailed=true) const;

  private: MeshDataT::Ptr data;

  // unique name for this mesh data. Important for calls of GetShapeSDF().
  private: std::string name;
};

}  // namespace

#endif  // COLLISION_BENCHMARK_MESHSHAPE
