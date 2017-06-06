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
/*
 * Author: Jennifer Buehler
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
#include <string>

namespace collision_benchmark
{
std::string SetOrReplaceFileExtension(const std::string &in,
                                      const std::string &ext)
{
  boost::filesystem::path p(in);
  boost::filesystem::path swapped = p.replace_extension(ext);
  return swapped.string();
}

aiMaterial * GetDefaultMaterial()
{
  aiMaterial* mat = new aiMaterial();
  /*mat->AddProperty(&srcMat.diffuse,  1,AI_MATKEY_COLOR_DIFFUSE);
  mat->AddProperty(&srcMat.specular, 1,AI_MATKEY_COLOR_SPECULAR);
  mat->AddProperty(&srcMat.ambient,  1,AI_MATKEY_COLOR_AMBIENT);
  mat->AddProperty(&srcMat.transparency, 1, AI_MATKEY_SHININESS);
  int m = (int)aiShadingMode_Phong;
  mat->AddProperty(&m, 1, AI_MATKEY_SHADING_MODEL);

  if (srcMat.name.length)
      mat->AddProperty(&srcMat.name, AI_MATKEY_NAME);
  */
  return mat;
}

template<typename Float = float>
aiScene *
CreateTrimeshScene(const typename
                   collision_benchmark::MeshData<Float, 3>::ConstPtr &meshData)
{
  // Create new aiScene (aiMesh)
  aiScene *assimpScene = new aiScene;
  aiMesh *assimpMesh = new aiMesh;
  assimpScene->mNumMeshes = 1;
  assimpScene->mMeshes = new aiMesh*[1];
  assimpScene->mMeshes[0] = assimpMesh;
  aiNode * rootNode = new aiNode();
  rootNode->mName.Set("Root");
  rootNode->mNumMeshes = 1;
  rootNode->mMeshes = new unsigned int[1];
  rootNode->mMeshes[0] = 0;
  assimpScene->mRootNode = rootNode;

  // add an empty material (required otherwise get assimp errors
  // in PretransformVertices when exporting)
  assimpScene->mNumMaterials = 1;
  assimpScene->mMaterials = new aiMaterial*[1];
  assimpScene->mMaterials[0] = GetDefaultMaterial();
  assimpMesh->mMaterialIndex = 0;

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

  const std::vector<Vertex>& vertices = meshData->GetVertices();
  const std::vector<Face>& faces = meshData->GetFaces();

  for (unsigned int i = 0; i < numVertices; ++i)
  {
    itAIVector3d.Set(vertices[i].X(),
                     vertices[i].Y(),
                     vertices[i].Z());
    assimpMesh->mVertices[i] = itAIVector3d;
    // this normal has no meaning, it will have to be
    // calculated in a postprocessing step. Unfortunately had
    // problems doing it locally with
    // aiApplyPostProcessing (assimpScene, aiProcess_GenNormals)
    // so do it in the exporter instead.
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

  return assimpScene;
}

/**
 * \param filename file to write to. It does not need to have an extension,
 *        as \e outformat will be added as extension. If it has an extension,
 *        it will be replace by \e outformat
 * \param outformat format to write it as, must be of the types
 *        supported by assimp (e.g. "dae", "stl", "obj")
 * \param meshData the mesh data to be written
 */
template<typename Float = float>
bool WriteTrimesh(const std::string &filename,
                  const std::string &outformat,
                  const typename collision_benchmark::MeshData
                                 <Float, 3>::ConstPtr &meshData)
{
  aiScene * scene = CreateTrimeshScene(meshData);
  if (!scene)
  {
    std::cerr << "Could not create trimesh scene" << std::endl;
    return false;
  }
  std::string fname = SetOrReplaceFileExtension(filename, outformat);

  // std::cout << "Writing to file " << fname << std::endl;

  Assimp::Exporter exporter;
  if (exporter.Export(scene, outformat, fname, aiProcess_GenNormals)\
      != AI_SUCCESS)
  {
    std::cerr << "Could not write " << fname << " in the format "
              << outformat << ". Error: " << exporter.GetErrorString()
              << std::endl;
    return false;
  }
  delete scene;
  return true;
}

}  // namespace

#endif  //  COLLISION_BENCHMARK_MESHHELPER
