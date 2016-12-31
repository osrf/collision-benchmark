#include <collision_benchmark/GazeboHelpers.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/ContactManager.hh>

#include <gazebo/gazebo_config.h>


void collision_benchmark::ClearModels(gazebo::physics::WorldPtr& world)
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
  std::vector<std::string> engines = {"ode" BULLET_SUPPORT SIMBODY_SUPPORT DART_SUPPORT};
  std::cout<<"Supported engines: "<<std::endl;
  for (int i=0; i<engines.size(); ++i)
    std::cout<<engines[i]<<std::endl;*/

  return engines;
}

std::map<std::string,std::string> collision_benchmark::getPhysicsSettingsSdfFor(const std::vector<std::string>& engines)
{
  std::map<std::string, std::string> physics_filenames;
  std::set<std::string> supported_engines=collision_benchmark::GetSupportedPhysicsEngines();

  for (std::vector<std::string>::const_iterator eit=engines.begin(); eit!=engines.end(); ++eit)
  {
    std::string e=*eit;
    if (!supported_engines.count(*eit)) continue;

    if (e=="bullet")
      physics_filenames["bullet"]="physics_settings/bullet_default.sdf";
    else if (e=="dart")
      physics_filenames["dart"]="physics_settings/dart_default.sdf";
    else if (e=="ode")
      physics_filenames["ode"]="physics_settings/ode_default.sdf";
    else if (e=="simbody")
      // XXX TODO add the empty_simbody.world file
      physics_filenames["simbody"] = "../physics_settings/simbody_default.world";
  }
  return physics_filenames;
}

std::map<std::string,std::string> collision_benchmark::getPhysicsSettingsSdfForAllEngines()
{
  std::set<std::string> enginesSet = collision_benchmark::GetSupportedPhysicsEngines();
  std::vector<std::string> enginesVector(enginesSet.begin(),enginesSet.end());
  return getPhysicsSettingsSdfFor(enginesVector);
}

