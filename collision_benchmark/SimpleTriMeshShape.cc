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

const std::string SimpleTriMeshShape::MESH_OUT_DIR="/tmp/";
const std::string SimpleTriMeshShape::MESH_EXT="stl";


// checks if \e path is a directory, or a potential path to a non-existing directory
bool isDirectory(const std::string& path)
{
  boost::filesystem::path dir(path);
  return boost::filesystem::is_directory(dir) // is an existing directory...
         || (!path.empty() && dir.extension().empty()); // or a non-existing directory
}

/**
 * Creates a directory if required.
 */
bool makeDirectoryIfNeeded(const std::string& dPath)
{
  if (!isDirectory(dPath))
  {
    std::cerr<<"Trying to access a directory which has the format of a file name"<<std::endl;
    return false;
  }

  try
  {
    boost::filesystem::path dir(dPath);
    boost::filesystem::path buildPath;

    for (boost::filesystem::path::iterator it(dir.begin()), it_end(dir.end()); it != it_end; ++it)
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


sdf::ElementPtr SimpleTriMeshShape::GetShapeSDF(bool detailed) const
{
  sdf::ElementPtr geometry(new sdf::Element());
  geometry->SetName("geometry");
  sdf::ElementPtr geomChild(new sdf::Element());
  geometry->InsertElement(geomChild);
  sdf::ElementPtr geomElem(new sdf::Element());
  geomChild->InsertElement(geomElem);

  if (!makeDirectoryIfNeeded(MESH_OUT_DIR))
  {
    std::cerr<<"Could not create directory to write mesh data to"<<std::endl;
    return sdf::ElementPtr();
  }

  std::string filename = (boost::filesystem::path(MESH_OUT_DIR) / boost::filesystem::path(name)).native();

  std::cout<<"Filename: "<<filename<<std::endl;

  if (!collision_benchmark::WriteTrimesh(filename,MESH_EXT, data))
  {
    std::cerr<<"Could not write mesh data!"<<std::endl;
  }

  return geometry;
}
