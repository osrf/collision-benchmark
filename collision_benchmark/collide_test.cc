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

#include <collision_benchmark/GazeboMultipleWorlds.hh>
#include <collision_benchmark/PrimitiveShape.hh>
#include <collision_benchmark/GazeboModelLoader.hh>

#include <boost/program_options.hpp>

#include <unistd.h>
#include <sys/wait.h>

using collision_benchmark::GazeboMultipleWorlds;
using collision_benchmark::GazeboModelLoader;
using collision_benchmark::Shape;
using collision_benchmark::PrimitiveShape;

namespace po = boost::program_options;

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  std::vector<std::string> selectedEngines;
  std::vector<std::string> unitShapes;
  std::vector<std::string> sdfModels;

  // description for engine options as stream so line doesn't go over 80 chars.
  std::stringstream descEngines;
  descEngines <<  "Specify one or several physics engines. " <<
      "Can contain [ode, bullet, dart, simbody]. Default is [ode].";

  // Declare the supported options.
  po::options_description desc("Allowed options");
  desc.add_options()
    ("help,h", "Produce help message")
    ("shape,s",
      po::value<std::vector<std::string>>(&unitShapes)->multitoken(),
      "Unit shape specification, can be any of [sphere, cylinder, cube]")
    ("model,m",
      po::value<std::vector<std::string>>(&sdfModels)->multitoken(),
      std::string(std::string("Model specification, can be either the ") +
      std::string("name of a model in the gazebo model paths, or a ") +
      std::string("path to a SDF file")).c_str())
    ;

  po::options_description desc_hidden("Positional options");
  desc_hidden.add_options()
    ("engines,e",
      po::value<std::vector<std::string>>(&selectedEngines)->multitoken(),
      descEngines.str().c_str())
    ;

  po::variables_map vm;
  po::positional_options_description p;
  // positional arguments default to "engines" argument
  p.add("engines", -1);

  po::options_description desc_composite;
  desc_composite.add(desc).add(desc_hidden);

  po::command_line_parser parser{argc, argv};
  parser.options(desc_composite).positional(p); // .allow_unregistered();
  po::parsed_options parsedOpt = parser.run();
  po::store(parsedOpt, vm);
  po::notify(vm);

  if (vm.count("help"))
  {
    std::cout << argv[0] <<" <list of engines> " << std::endl;
    std::cout << desc << std::endl;
    return 1;
  }

  if (vm.count("engines"))
  {
    std::cout << "Engines to load: " << std::endl;
    for (std::vector<std::string>::iterator it = selectedEngines.begin();
         it != selectedEngines.end(); ++it)
    {
      std::cout<<*it<<std::endl;
    }
  }
  else
  {
    std::cout << "No engines were specified, so using 'ode'" << std::endl;
    selectedEngines.push_back("ode");
  }

/*  if (vm.count("shape"))
  {
    std::cout << "Shapes specified " << vm.count("shape") << std::endl;
    for (std::vector<std::string>::iterator it = unitShapes.begin();
         it != unitShapes.end(); ++it)
    {
      std::cout<<*it<<std::endl;
    }
  }

  if (vm.count("model"))
  {
    std::cout << "Models specified " << vm.count("model") << std::endl;
    for (std::vector<std::string>::iterator it = sdfModels.begin();
         it != sdfModels.end(); ++it)
    {
      std::cout<<*it<<std::endl;
    }
  }
*/

  if (unitShapes.size() + sdfModels.size() != 2)
  {
    std::cerr << "Have to specify exactly two shapes or models" << std::endl;
    std::cerr << "Specified shapes: " << std::endl;
    for (std::vector<std::string>::iterator it = unitShapes.begin();
         it != unitShapes.end(); ++it)
      std::cout<<" " << *it<<std::endl;
    std::cerr << "Models: " << std::endl;
    for (std::vector<std::string>::iterator it = sdfModels.begin();
         it != sdfModels.end(); ++it)
      std::cout<<" " << *it<<std::endl;
    std::cout << "which makes a total of " << (unitShapes.size() +
                 sdfModels.size()) << " models. " << std::endl;
    return 1;
  }


  // Initialize server
  bool loadMirror = true;
  bool enforceContactCalc=false;
  bool allowControlViaMirror = true;
  GazeboMultipleWorlds gzMultiWorld;
  gzMultiWorld.Load(selectedEngines, loadMirror, enforceContactCalc);

  // Load extra models to the world
  typedef GazeboMultipleWorlds::GzWorldManager GzWorldManager;
  GzWorldManager::Ptr worldManager
    = gzMultiWorld.GetWorldManager();
  assert(worldManager);

  std::vector<std::string> loadedModelNames;

  // load all primitive shapes
  typedef GzWorldManager::ModelLoadResult ModelLoadResult;
  int i = 0;
  for (std::vector<std::string>::iterator it = unitShapes.begin();
       it != unitShapes.end(); ++it, ++i)
  {
    const std::string& shapeID = *it;
    Shape::Ptr shape;
    if (shapeID == "sphere")
    {
      float radius = 1;
      shape.reset(PrimitiveShape::CreateSphere(radius));
    }
    else if (shapeID == "cylinder")
    {
    }
    else if (shapeID == "cube")
    {
    }
    else
    {
      std::cerr << "Unknown shape type: " << shapeID << std::endl;
      return 1;
    }
    std::string modelName = "unit_" + shapeID + "_" + std::to_string(i);
    std::vector<ModelLoadResult> res
      = worldManager->AddModelFromShape(modelName, shape, shape);
    if (res.size() != worldManager->GetNumWorlds())
    {
      std::cerr << "Model must have been loaded in all worlds" << std::endl;
      return 1;
    }
    loadedModelNames.push_back(modelName);
  }

  // load all models:
  i = 0;
  for (std::vector<std::string>::iterator it = sdfModels.begin();
       it != sdfModels.end(); ++it, ++i)
  {
    const std::string& modelResource = *it;
    std::string modelSDF =
      GazeboModelLoader::GetModelSdfFilename(modelResource);
    // std::cout << "Loading model: " << modelSDF << std::endl;
    std::string modelName = "model" + std::to_string(i);
    std::vector<ModelLoadResult> res =
      worldManager->AddModelFromFile(modelSDF, modelName);
    if (res.size() != worldManager->GetNumWorlds())
    {
      std::cerr << "Model must have been loaded in all worlds" << std::endl;
      return 1;
    }
    loadedModelNames.push_back(modelName);
  }

  // physics should be disable as this test only is meant to
  // display the contacts.
  bool physicsEnabled = false;
  gzMultiWorld.Run(physicsEnabled);

  std::cout << "Bye, bye." << std::endl;
  return 0;
}
