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

#include <boost/filesystem.hpp>

using collision_benchmark::SimpleTriMeshShape;

const std::string SimpleTriMeshShape::MESH_SUB_DIR="meshes";
const std::string SimpleTriMeshShape::MESH_EXT="stl";

// checks if \e path is a directory, or a potential path to a
// non-existing directory
bool isDirectory(const std::string& path)
{
  boost::filesystem::path dir(path);
  // is an existing directory
  // or a non-existing directory
  return boost::filesystem::is_directory(dir)
         || (!path.empty() && dir.extension().empty());
}

/**
 * Creates a directory if required.
 */
bool makeDirectoryIfNeeded(const std::string& dPath)
{
  if (!isDirectory(dPath))
  {
    std::cerr<<"Trying to access a directory which has the format "
             <<"of a file name"<<std::endl;
    return false;
  }

  try
  {
    boost::filesystem::path dir(dPath);
    boost::filesystem::path buildPath;

    for (boost::filesystem::path::iterator
         it(dir.begin()), it_end(dir.end()); it != it_end; ++it)
    {
      buildPath /= *it;

      if (!boost::filesystem::exists(buildPath) &&
          !boost::filesystem::create_directory(buildPath))
      {
        std::cerr<<"Could not create directory " << buildPath;
        return false;
      }
    }
  }
  catch (const boost::filesystem::filesystem_error& ex)
  {
    std::cerr<<ex.what()<<std::endl;
    return false;
  }
  return true;
}


sdf::ElementPtr SimpleTriMeshShape::GetShapeSDF(bool detailed,
                                                bool uriOnlyWithSubdir) const
{
  // full path of subdirectories
  std::string subdir =  (boost::filesystem::path(RESOURCE_SUB_DIR) /
                          boost::filesystem::path(MESH_SUB_DIR)).native();

  // full path to resources
  std::string fulldir = (boost::filesystem::path(RESOURCE_OUT_DIR) /
                          boost::filesystem::path(subdir)).native();

  // filename only with the subdirectory structure
  std::string subname = (boost::filesystem::path(subdir) /
                         boost::filesystem::path(name +
                        (detailed ? "" : "_lowres") + "." + MESH_EXT)).native();

  // filename with the full directory structure
  std::string fullname = (boost::filesystem::path(RESOURCE_OUT_DIR) /
                          boost::filesystem::path(subname)).native();

  if (!makeDirectoryIfNeeded(fulldir))
  {
    std::cerr<<"Could not create directory to write mesh data to"<<std::endl;
    return sdf::ElementPtr();
  }

  std::string useURI;
  if (uriOnlyWithSubdir)
  {
    useURI="file://"+subname;
  }
  else
  {
    useURI="file://"+fullname;
  }

  if (!collision_benchmark::WriteTrimesh(fullname, MESH_EXT, data))
  {
    std::cerr<<"Could not write mesh data!"<<std::endl;
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
