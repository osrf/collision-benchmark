#include <collision_benchmark/GazeboHelpers.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/ContactManager.hh>


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
