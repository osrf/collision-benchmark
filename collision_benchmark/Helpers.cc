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
 * Date: December 2016
 */

#include <collision_benchmark/Helpers.hh>
#include <boost/filesystem.hpp>
#include <iostream>

////////////////////////////////////////////////////////////////
bool collision_benchmark::isDirectory(const std::string &path)
{
  boost::filesystem::path dir(path);
  // is an existing directory
  // or a non-existing directory
  return boost::filesystem::is_directory(dir)
         || (!path.empty() && dir.extension().empty());
}

////////////////////////////////////////////////////////////////
bool collision_benchmark::makeDirectoryIfNeeded(const std::string &dPath)
{
  if (!isDirectory(dPath))
  {
    std::cerr << "Trying to access a directory which has the format "
             <<"of a file name" << std::endl;
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
        std::cerr << "Could not create directory " << buildPath;
        return false;
      }
    }
  }
  catch(const boost::filesystem::filesystem_error &ex)
  {
    std::cerr << ex.what() << std::endl;
    return false;
  }
  return true;
}



