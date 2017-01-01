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
#ifndef COLLISION_BENCHMARK_MESHHELPER
#define COLLISION_BENCHMARK_MESHHELPER

#include <collision_benchmark/MeshData.hh>
#include <assimp/Exporter.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <assimp/cimport.h>

#include <boost/filesystem.hpp>

#include <vector>

namespace collision_benchmark
{

std::string SetOrReplaceFileExtension(const std::string& in, const std::string& ext)
{
  boost::filesystem::path p(in);
  boost::filesystem::path swapped = p.replace_extension(ext);
  return swapped.string();
}


template<typename Float=float>
aiScene * CreateTrimeshScene(const typename collision_benchmark::MeshData<Float, 3>::ConstPtr& meshData)
{
  // Create new aiScene (aiMesh)
  aiScene *assimpScene = new aiScene;
  aiMesh *assimpMesh = new aiMesh;
  assimpScene->mNumMeshes = 1;
  assimpScene->mMeshes = new aiMesh*[1];
  assimpScene->mMeshes[0] = assimpMesh;
  assimpScene->mRootNode = new aiNode();

  int numVertices = meshData->GetVertices().size();
  int numFaces = meshData->GetFaces().size();

  // Set vertices and normals
  assimpMesh->mNumVertices = numVertices;
  assimpMesh->mVertices = new aiVector3D[numVertices];
  assimpMesh->mNormals = new aiVector3D[numVertices];
  aiVector3D itAIVector3d;

  typedef MeshData<Float, 3> MeshDataT;
  typedef typename MeshDataT::Vertex Vertex;
  typedef typename MeshDataT::Face Face;

  const std::vector<Vertex>& vertices=meshData->GetVertices();
  const std::vector<Face>& faces=meshData->GetFaces();

  for (unsigned int i = 0; i < numVertices; ++i)
  {
    itAIVector3d.Set(vertices[i].X(),
                     vertices[i].Y(),
                     vertices[i].Z());
    assimpMesh->mVertices[i] = itAIVector3d;
    assimpMesh->mNormals[i]  = itAIVector3d;
  }

  // Set faces
  assimpMesh->mNumFaces = numFaces;
  assimpMesh->mFaces = new aiFace[assimpMesh->mNumFaces];
  for (unsigned int i = 0; i < assimpMesh->mNumFaces; ++i)
  {
    aiFace* itAIFace = &assimpMesh->mFaces[i];
    itAIFace->mNumIndices = 3;
    itAIFace->mIndices = new unsigned int[3];
    itAIFace->mIndices[0] = faces[i].val[0];
    itAIFace->mIndices[1] = faces[i].val[1];
    itAIFace->mIndices[2] = faces[i].val[2];
  }

/*  const aiScene * checkScene = aiApplyPostProcessing (assimpScene, aiProcess_GenNormals);
  if (!checkScene || checkScene != assimpScene)
  {
    std::cerr<<"Could not apply postprocessing"<<std::endl;
    return NULL;
  }*/
  return assimpScene;
}

/**
 * \param filename file to write to. It does not need to have an extension, as \e outformat will
 *      be added as extension. If it has an extension, it will be replace by \e outformat
 * \param outformat format to write it as, must be of the types supported by assimp (e.g. "dae", "stl", "obj")
 * \param meshData the mesh data to be written
 */
template<typename Float=float>
bool WriteTrimesh(const std::string& filename,
                  const std::string& outformat,
                  const typename collision_benchmark::MeshData<Float, 3>::ConstPtr& meshData)
{
  aiScene * scene = CreateTrimeshScene(meshData);
  if (!scene)
  {
    std::cerr<<"Could not create trimesh scene"<<std::endl;
    return false;
  }
  std::cout<<"Created scene. "<<std::endl;

  std::string fname = SetOrReplaceFileExtension(filename, outformat);

  std::cout<<"Writing to file "<<fname<<std::endl;

  Assimp::Exporter exporter;
  if (exporter.Export(scene, outformat, fname) != AI_SUCCESS)
  {
    std::cerr<<"Could not write "<<fname<<" in the format "<<outformat<<std::endl;
    return false;
  }
  return true;
}

}  // namespace

#endif  //  COLLISION_BENCHMARK_MESHHELPER
