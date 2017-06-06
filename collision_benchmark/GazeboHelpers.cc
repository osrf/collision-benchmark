#include <collision_benchmark/GazeboHelpers.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/ContactManager.hh>

#include <gazebo/gazebo_config.h>

#include <tinyxml.h>

/////////////////////////////////////////////////
void collision_benchmark::ClearModels(gazebo::physics::WorldPtr &world)
{
  bool pauseState = world->IsPaused();
  world->SetPaused(true);
  // XXX TODO CHECK: this does not clear the lights, but they should be
  // added/removed/changed in the subsequent diff state?!
  world->ClearModels();
  // need to reset physics engine in order to stop it
  // from publishing old contact points?
  gazebo::physics::PhysicsEnginePtr physics = world->Physics();
  if (physics) physics->GetContactManager()->Clear();
  world->SetPaused(pauseState);
}



/////////////////////////////////////////////////
std::set<std::string> collision_benchmark::GetSupportedPhysicsEngines()
{
  std::set<std::string> engines;
  // XXX TODO: Should use same names as used
  // in Gazebo/SDF to select physics engines. Use this as soon
  // as there a constant/macro for it.
  engines.insert("ode"); // ODE is always supported
#ifdef HAVE_BULLET
  engines.insert("bullet");
#endif
#ifdef HAVE_DART
  engines.insert("dart");
#endif
#ifdef HAVE_SIMBODY
  engines.insert("simbody");
#endif

  // could also use:
  // if (gazebo::physics::PhysicsFactory::IsRegistered("ode"))

  /*
  or with #include <gazebo/test/helper_physics_generator.hh>
  std::vector<std::string> engines =
      {"ode" BULLET_SUPPORT SIMBODY_SUPPORT DART_SUPPORT};
  std::cout << "Supported engines: " << std::endl;
  for (int i = 0; i<engines.size(); ++i)
    std::cout << engines[i]<<std::endl;*/

  return engines;
}

/////////////////////////////////////////////////
std::string collision_benchmark::getPhysicsSettingsSdfFor(const std::string &e)
{
  if (e=="bullet")
    return "physics_settings/bullet_default.sdf";
  else if (e=="dart")
    return "physics_settings/dart_default.sdf";
  else if (e=="ode")
    return "physics_settings/ode_default.sdf";
  else if (e=="simbody")
    return "physics_settings/simbody_default.sdf";
  return "";
}

/////////////////////////////////////////////////
std::map<std::string, std::string>
collision_benchmark::getPhysicsSettingsSdfFor
  (const std::vector<std::string>& engines)
{
  std::map<std::string, std::string> physics_filenames;
  std::set<std::string> supported_engines =
    collision_benchmark::GetSupportedPhysicsEngines();

  for (std::vector<std::string>::const_iterator
       eit = engines.begin(); eit!=engines.end(); ++eit)
  {
    std::string e=*eit;
    if (!supported_engines.count(e)) continue;
    std::string sdf = getPhysicsSettingsSdfFor(e);
    if (sdf.empty())
    {
      std::cout << "WARNING in collision_benchmark::getPhysicsSettingsSdfFor: "
                << "Physics engine " << e << " not supported. " << std::endl;
      continue;
    }
    physics_filenames[e] = sdf;
  }
  return physics_filenames;
}

/////////////////////////////////////////////////
std::map<std::string, std::string>
collision_benchmark::getPhysicsSettingsSdfForAllEngines()
{
  std::set<std::string> enginesSet =
    collision_benchmark::GetSupportedPhysicsEngines();
  std::vector<std::string> enginesVector(enginesSet.begin(), enginesSet.end());
  return getPhysicsSettingsSdfFor(enginesVector);
}



/////////////////////////////////////////////////
int collision_benchmark::isProperSDFFile(const std::string &filename,
                                         std::string * version)
{
  TiXmlDocument xmlDoc;
  if (!xmlDoc.LoadFile(filename))
  {
    // std::cout << "Could not read file " << filename
    //           << " so cannot check if SDF needs conversion" << std::endl;
    return -3;
  }

  TiXmlElement *elem = xmlDoc.FirstChildElement("sdf");
  if (!elem)
  {
    // std::cout << "No outer SDF tag" << std::endl;
    return -2;
  }

  if (!elem->Attribute("version"))
  {
    // std::cout<< "SDF Tag has no SDF version" << std::endl;
    return -1;
  }

  if (version)
  {
    *version = elem->Attribute("version");
    // std::cout << "SDF version "<<*version << std::endl;
  }

  return 0;
}

/////////////////////////////////////////////////
int collision_benchmark::isProperSDFString(const std::string &string,
                                           std::string * version)
{
  TiXmlDocument xmlDoc;
  xmlDoc.Parse(string.c_str());
  if (xmlDoc.Error())
  {
    return -3;
  }

  TiXmlElement *elem = xmlDoc.FirstChildElement("sdf");
  if (!elem)
  {
    //std::cout << "No outer SDF tag" << std::endl;
    return -2;
  }

  if (!elem->Attribute("version"))
  {
    //std::cout<< "SDF Tag has no SDF version" << std::endl;
    return -1;
  }

  if (version)
  {
    *version = elem->Attribute("version");
    // std::cout << "SDF version "<<*version << std::endl;
  }

  return 0;
}


void collision_benchmark::wrapSDF(std::string &sdf)
{
  std::stringstream mod;
  mod << "<sdf version='1.6'>" << sdf << "</sdf>";
  sdf = std::string(mod.str());
}

void collision_benchmark::wrapSDF(std::vector<std::string>& sdfs)
{
  for (std::vector<std::string>::iterator
       it = sdfs.begin(); it != sdfs.end(); ++it)
  {
    wrapSDF(*it);
  }
}
