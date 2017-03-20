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
#ifndef COLLISION_BENCHMARK_MESHDATA
#define COLLISION_BENCHMARK_MESHDATA

#include <ignition/math/Vector3.hh>
#include <vector>
#include <memory>
#include <type_traits>

namespace collision_benchmark
{

/**
 * Simple class for mesh data. Includes vertex and face index array.
 *
 * \param VertexPrecision_ precision of the vertices, defaults to float.
 * \param FaceSize size of a face (3 for triangle meshes, which is the default).
 *        Must be at least 3.
 *
 * \author Jennifer Buehler
 * \date December 2016
 */
template<typename VertexPrecision_=float, int FaceSize=3>
class MeshData
{
  static_assert(FaceSize >= 3, "FaceSize must be at least 3");

  private: typedef MeshData<VertexPrecision_, FaceSize> Self;
  public: typedef VertexPrecision_ VertexPrecision;
  public: typedef ignition::math::Vector3<VertexPrecision> Vertex;

  public: typedef std::shared_ptr<Self> Ptr;
  public: typedef std::shared_ptr<const Self> ConstPtr;

  public: struct Face
          {
            Face(const std::size_t& i1,
                   const std::size_t& i2,
                   const std::size_t& i3)
            { val[0]=i1; val[1]=i2; val[2]=i3; }

            const std::size_t& operator[](int i) const { return val[i]; }
            std::size_t val[FaceSize];
          };

  MeshData(){}
  public: MeshData(const std::vector<Vertex>& vertices,
                   const std::vector<Face>& faces):
            verts(vertices),
            faces(faces) {}
  public: MeshData(const MeshData& o):
            verts(o.verts),
            faces(o.faces) {}
  public: ~MeshData(){}

  public: inline std::vector<Vertex>& GetVertices() { return verts; }
  public: inline const std::vector<Vertex>& GetVertices() const
                  { return verts; }

  public: inline std::vector<Face>& GetFaces() { return faces; }
  public: inline const std::vector<Face>& GetFaces() const { return faces; }

  // Perturbs each vertex by a random value between \e min and \e max along
  // the line from the vertex to \e center.
  public: void Perturb(const double min, const double max,
                       const Vertex& center = Vertex(0,0,0));

  // Perturbs each vertex by a random value between \e min and \e max *away
  // from the line* through \e center with direction \e dir.
  // This will move the vertex along the line orthogonal to the given line.
  // Vertices on the line will not be perturbed.
  public: void Perturb(const double min, const double max,
                       const Vertex& center, const Vertex& dir);


  private: std::vector<Vertex> verts;
  private:std::vector<Face> faces;

};

}

#include "MeshData-inl.hh"

#endif   //  COLLISION_BENCHMARK_MESHDATA
