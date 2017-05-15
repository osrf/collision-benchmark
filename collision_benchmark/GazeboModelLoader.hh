/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#ifndef COLLISION_BENCHMARK_GAZEBOMODELLOADER_H
#define COLLISION_BENCHMARK_GAZEBOMODELLOADER_H

#include <gazebo/common/CommonIface.hh>
#include <sdf/sdf_config.h>
#include <boost/filesystem.hpp>
#include <tinyxml.h>

namespace collision_benchmark
{

/**
 * \brief Helper to load a model of a given name from the gazebo model paths.
 *
 * \author Jennifer Buehler
 * \date May 2017
 */
class GazeboModelLoader
{
  //
  public: static std::string
          GetModelSdfFilename(const std::string& modelNameOrFile)
  {
    // first, check if \e modelNameOrFile is a file
    boost::filesystem::path modelNamePath(modelNameOrFile);
    if (boost::filesystem::exists(modelNamePath) &&
        boost::filesystem::is_regular_file(modelNamePath))
    {
      // XXX TODO: Check that it is a SDF file too
      return modelNameOrFile;
    }

    // this must be a model name, so try to find it in the gazebo paths
    std::string modelPath =
        gazebo::common::find_file("model://" + modelNameOrFile);
    std::cout << "Found model: " << modelNameOrFile << std::endl;

    std::string sdfFilename;
    // find the manifest.config (or .xml) to read the SDF file from
    boost::filesystem::path manifestPath = modelPath;
    if (boost::filesystem::exists(manifestPath / "model.config"))
    {
      manifestPath /= "model.config";
    }
    else if (boost::filesystem::exists(manifestPath / "model.xml"))
    {
      std::cerr << "The manifest.xml for a model is deprecated. "
                << "Please rename manifest.xml to "
                << "model.config" << std::endl;

      manifestPath /= "manifest.xml";
    }
    else
    {
      std::cerr << "Cannot find model files for '" << modelNameOrFile
                << "'" << std::endl;
    }

    TiXmlDocument manifestDoc;
    if (manifestDoc.LoadFile(manifestPath.string()))
    {
      TiXmlElement *modelXML = manifestDoc.FirstChildElement("model");
      if (!modelXML)
        std::cerr << "No <model> element in manifest "
                  << manifestPath << std::endl;
      else
      {
        TiXmlElement *sdfXML = modelXML->FirstChildElement("sdf");

        TiXmlElement *sdfSearch = sdfXML;

        // Find the SDF element that matches the current SDF version.
        while (sdfSearch)
        {
          if (sdfSearch->Attribute("version") &&
              std::string(sdfSearch->Attribute("version")) == SDF_VERSION)
          {
            sdfXML = sdfSearch;
            break;
          }

          sdfSearch = sdfSearch->NextSiblingElement("sdf");
        }

        sdfFilename = modelPath + "/" + sdfXML->GetText();
      }
    }
    else
    {
      std::cerr << "Error parsing XML in file '"
                << manifestPath.string() << "': "
                << manifestDoc.ErrorDesc() << std::endl;
    }
    return sdfFilename;
  }
};  // class GazeboModelLoader

} // namespace

#endif  // COLLISION_BENCHMARK_GAZEBOMODELLOADER_H
