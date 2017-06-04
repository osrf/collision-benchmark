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

#include <test/CollidingShapesTestFramework.hh>
#include <boost/program_options.hpp>

namespace po = boost::program_options;

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  std::vector<std::string> selectedEngines;
  std::vector<std::string> unitShapes;
  std::vector<std::string> sdfModels;

  // Read command line parameters
  // ----------------------

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

  collision_benchmark::test::CollidingShapesTestFramework csTest;
  bool success = csTest.Run(selectedEngines, unitShapes, sdfModels);

  std::cout << "Bye, bye." << std::endl;
  return success ? 0 : 1;
}
