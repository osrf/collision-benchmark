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

#ifndef COLLISION_BENCHMARK_GAZEBOSTATE_COMPARE_HH
#define COLLISION_BENCHMARK_GAZEBOSTATE_COMPARE_HH

#include <ignition/math/Pose3.hh>

// forward declarations
namespace gazebo
{
  namespace physics
  {
    class WorldState;
    class ModelState;
    class LightState;
    class LinkState;
    class JointState;
  }
  namespace math
  {
    class Pose;
  }
}

namespace collision_benchmark
{
/**
 * Provides a number of functions to check for equality of
 * states within a Gazebo world to be equal
 */
struct GazeboStateCompare
{
  // tolerances for all kinds of of comparisons.
  // Put in one struct in order to reduce number of parameters in
  // the Equal() functions.
  struct Tolerances
  {
    static Tolerances CreateDefault(float accuracy = 1e-03)
    {
      Tolerances t;
      t.Position = accuracy;
      t.Orientation = accuracy;
      t.Force = accuracy;
      t.Torque = accuracy;
      t.Scale = accuracy;
      t.Velocity = accuracy;
      t.VelocityOrientation = accuracy;
      t.Acceleration = accuracy;
      t.AccelerationOrientation = accuracy;
      t.JointAngle = accuracy;
      t.CheckLinkCollisionStates = true;
      t.CheckDynamics = true;
      return t;
    }

    // static member always initialized to CreateDefault().
    static const Tolerances Default;

    // Tolerance for scale of a model
    double Scale;
    // Tolerance for positions of links, joints, models
    double Position;
    // Tolerance for orientation of links, joints, models.
    // Orientation is compared with the Euler angles
    // instead of with Quaternions, so this is the angular tolerance (radians).
    double Orientation;
    // Tolerance for forces on links
    double Force;
    // Tolerance for forques on links
    double Torque;
    // Tolerance for the velocity of links and models
    double Velocity;
    // Tolerance for orienation of velocity vector
    double VelocityOrientation;
    // Tolerance for the acceleration of links and models
    double Acceleration;
    // Tolerance for orienation of acceleration vector
    double AccelerationOrientation;
    // Tolerance for joint angles (radian)
    double JointAngle;

    // In links, check equality of CollisionState as well.
    bool CheckLinkCollisionStates;
    // In links, check force, velocity, acceleration if true, or skip if false.
    bool CheckDynamics;
  };


  static bool Equal(const gazebo::physics::WorldState &s1,
                    const gazebo::physics::WorldState &s2,
                    const Tolerances &tolerance = Tolerances::Default,
                    const bool checkLights = true);
  static bool Equal(const gazebo::physics::ModelState &s1,
                    const gazebo::physics::ModelState &s2,
                    const Tolerances &tolerance = Tolerances::Default);
  static bool Equal(const gazebo::physics::LightState &s1,
                    const gazebo::physics::LightState &s2,
                    const Tolerances &tolerance = Tolerances::Default);
  static bool Equal(const gazebo::physics::LinkState &s1,
                    const gazebo::physics::LinkState &s2,
                    const Tolerances &tolerance = Tolerances::Default);
  static bool Equal(const gazebo::physics::JointState &s1,
                    const gazebo::physics::JointState &s2,
                    const Tolerances &tolerance = Tolerances::Default);

  //  \param orientationTolerance orientation is compared with the Euler
  //         angles instead of with Quaternions, so this is the angular
  //         tolerance (radians).
  static bool Equal(const ignition::math::Pose3d &p1,
                    const ignition::math::Pose3d &p2,
                    const double &positionTolerance,
                    const double &orientationTolerance);
};
}
#endif  //  COLLISION_BENCHMARK_GAZEBOSTATE_COMPARE_HH
