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
#include <collision_benchmark/SimpleTriMeshShape.hh>
#include <collision_benchmark/MeshHelper.hh>
#include <collision_benchmark/Helpers.hh>

using collision_benchmark::SimpleTriMeshShape;

const std::string SimpleTriMeshShape::MESH_EXT="stl";

sdf::ElementPtr
SimpleTriMeshShape::GetShapeSDF(bool detailed,
                                const std::string &resourceDir,
                                const std::string &resourceSubDir,
                                const bool useFullPath) const
{
  if (resourceDir.empty() && resourceSubDir.empty())
  {
    std::cerr << "Resource directory to write mesh data to is empty, "
              << "so cannot create SDF for mesh." << std::endl;
    return sdf::ElementPtr();
  }

  // full path of subdirectory
  std::string subdir =  boost::filesystem::path(resourceSubDir).native();

  // full path to resources
  std::string fulldir = (boost::filesystem::path(resourceDir) /
                          boost::filesystem::path(subdir)).native();

  // filename only with the subdirectory structure
  std::string subname = (boost::filesystem::path(subdir) /
                         boost::filesystem::path(name +
                        (detailed ? "" : "_lowres") + "." + MESH_EXT)).native();

  // filename with the full directory structure
  std::string fullname = (boost::filesystem::path(resourceDir) /
                          boost::filesystem::path(subname)).native();

  if (!collision_benchmark::makeDirectoryIfNeeded(fulldir))
  {
    std::cerr << "Could not create directory to write mesh data to"
              << std::endl;
    return sdf::ElementPtr();
  }

  std::string useURI;

  if (useFullPath)
  {
    useURI="file://"+fullname;
  }
  else
  {
    useURI="file://"+subname;
  }

  if (!collision_benchmark::WriteTrimesh(fullname, MESH_EXT, data))
  {
    std::cerr << "Could not write mesh data!" << std::endl;
    return sdf::ElementPtr();
  }

  sdf::ElementPtr geometry(new sdf::Element());
  geometry->SetName("geometry");
  sdf::ElementPtr meshElem(new sdf::Element());
  meshElem->SetName("mesh");
  geometry->InsertElement(meshElem);

  sdf::ElementPtr uriElem(new sdf::Element());
  meshElem->InsertElement(uriElem);
  uriElem->SetName("uri");
  uriElem->AddValue("string", useURI, true, "URI to mesh file");

  sdf::ElementPtr scaleElem(new sdf::Element());
  meshElem->InsertElement(scaleElem);
  scaleElem->SetName("scale");
  scaleElem->AddValue("vector3", "1.0 1.0 1.0", false, "scale of mesh");

  return geometry;
}
