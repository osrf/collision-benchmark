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
/*
 * Author: Jennifer Buehler
 * Date: December 2016
 */

#include <collision_benchmark/GazeboPhysicsWorld.hh>
#include <collision_benchmark/GazeboMultipleWorlds.hh>
#include <collision_benchmark/GazeboMultipleWorldsServer.hh>
#include <collision_benchmark/WorldManager.hh>
#include <boost/program_options.hpp>


using collision_benchmark::WorldManager;
using collision_benchmark::GazeboPhysicsWorld;
using collision_benchmark::GazeboMultipleWorlds;
using collision_benchmark::MultipleWorldsServer;
using collision_benchmark::GazeboMultipleWorldsServer;
using collision_benchmark::GazeboPhysicsWorldTypes;

namespace po = boost::program_options;

typedef MultipleWorldsServer<GazeboPhysicsWorldTypes::WorldState,
                             GazeboPhysicsWorldTypes::ModelID,
                             GazeboPhysicsWorldTypes::ModelPartID,
                             GazeboPhysicsWorldTypes::Vector3,
                             GazeboPhysicsWorldTypes::Wrench>
                                GzMultipleWorldsServer;

typedef WorldManager<GazeboPhysicsWorldTypes::WorldState,
                     GazeboPhysicsWorldTypes::ModelID,
                     GazeboPhysicsWorldTypes::ModelPartID,
                     GazeboPhysicsWorldTypes::Vector3,
                     GazeboPhysicsWorldTypes::Wrench>
          GzWorldManager;

// the server
collision_benchmark::GazeboMultipleWorlds::Ptr g_server;

// will be called at each loop iteration
void LoopIter(int iter)
{
}

////////////////////////////////////////////////////////////////
void InitWorlds(const std::vector<std::string>& engines,
                const std::vector<std::string>& worldFiles,
                const std::vector<std::string>& additionalGuis,
                const bool keepWorldNames = false)
{
  bool loadMirror = true;
  bool allowControlViaMirror = false;
  bool enforceContactCalc = true;
  g_server.reset(new GazeboMultipleWorlds());
  g_server->Init(loadMirror, enforceContactCalc,
               allowControlViaMirror, true, additionalGuis);

  GzMultipleWorldsServer::Ptr mServer = g_server->GetServer();
  assert(mServer);

  // load the worlds as given in command line arguments
  // with the engine names given
  int i = 0;
  for (std::vector<std::string>::const_iterator it = worldFiles.begin();
       it != worldFiles.end(); ++it, ++i)
  {
    std::string worldfile = *it;
    std::cout << "Loading world " << worldfile <<std::endl;

    std::string worldPrefix;
    if (!engines.empty() || !keepWorldNames)
    {
      std::stringstream _worldPrefix;
      _worldPrefix << "world" << "_" << i;
      worldPrefix = _worldPrefix.str();
    }

    if (engines.empty())
    {
      if (mServer->AutoLoad(worldfile, worldPrefix) < 0)
        std::cerr << "Could not auto-load world " << worldfile << std::endl;
    }
    else
    {
      mServer->Load(worldfile, engines, worldPrefix);
    }
  }
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  std::vector<std::string> selectedEngines;
  std::vector<std::string> worldFiles;

  // description for engine options as stream so line doesn't go over 80 chars.
  std::stringstream descEngines;
  descEngines <<  "Specify one or several physics engines. " <<
      "Can contain [ode, bullet, dart, simbody]. When not specified, worlds " <<
      "are loaded with the engine specified in the file. If specified, all " <<
      "worlds are loaded with each of the engines specified.";

  // Declare the supported options.
  po::options_description desc("Allowed options");
  desc.add_options()
    ("help,h", "Produce help message")
    ("engines,e",
      po::value<std::vector<std::string> >(&selectedEngines)->multitoken(),
      descEngines.str().c_str())
    ("keep-name,k", "keep the names of the worlds as specified in the files. \
Only works when no engines are specified with -e.")
    ("worlds,w",
      po::value<std::vector<std::string> >(&worldFiles)->multitoken(),
      "World file(s).");

  po::variables_map vm;
  po::options_description desc_composite;
  desc_composite.add(desc); //.add(desc_hidden);

  po::positional_options_description p;
  // positional arguments default to "worlds" argument
  p.add("worlds", -1);
  po::command_line_parser parser(argc, argv);
  parser.options(desc_composite).positional(p);
  po::parsed_options parsedOpt = parser.run();
  po::store(parsedOpt, vm);
  po::notify(vm);

  if (vm.count("help"))
  {
    std::cout << argv[0] <<" <list of world files> " << std::endl;
    std::cout << desc << std::endl;
    return 1;
  }

  if (vm.count("engines"))
  {
    std::cout << "Engines to load: " << std::endl;
    for (std::vector<std::string>::iterator it = selectedEngines.begin();
         it != selectedEngines.end(); ++it)
    {
      std::cout << *it << std::endl;
    }
  }
  else
  {
    std::cout << "No engines were given, so using physics information "
              << "specified in world files" << std::endl;
  }

  if (!vm.count("worlds"))
  {
    std::cout << "You need to specify at least one world." << std::endl;
    return 0;
  }

  // we could load additional GUI elements here, but at the moment
  // none are used.
  std::vector<std::string> additionalGuis;
  InitWorlds(selectedEngines, worldFiles, additionalGuis, vm.count("keep-name"));
  assert(g_server);

  std::function<void(int)> loopCallback(&LoopIter);
  bool waitForStartSignal = true;
  bool blocking = true;
  g_server->Run(waitForStartSignal, blocking); //, loopCallback);
}
