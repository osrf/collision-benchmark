#ifndef COLLISION_BENCHMARK_GAZEBOWORLDSTATE_
#define COLLISION_BENCHMARK_GAZEBOWORLDSTATE_

#include <gazebo/physics/World.hh>

namespace collision_benchmark
{

/**
 * Sets the \e world to the state \e targetState
 */
void SetWorldState(gazebo::physics::WorldPtr& world, const gazebo::physics::WorldState& targetState);

/**
 * Print the world state. Can be used for testing.
 */
void PrintWorldState(const gazebo::physics::WorldPtr world);

/**
 * Print the world state. Can be used for testing.
 */
void PrintWorldStates(const std::vector<gazebo::physics::WorldPtr>& worlds);

}

#endif   // COLLISION_BENCHMARK_GAZEBOWORLDSTATE_
