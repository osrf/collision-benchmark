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


