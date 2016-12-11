#ifndef COLLISION_BENCHMARK_GAZEBOSTATE_COMPARE_HH
#define COLLISION_BENCHMARK_GAZEBOSTATE_COMPARE_HH


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
  // Put in one struct in order to reduce number of parameters in the Equal() functions.
  struct Tolerances
  {
    static Tolerances CreateDefault()
    {
      Tolerances t;
      t.Position=1e-07;
      t.Orientation=1e-07;
      t.Force=1e-07;
      t.Torque=1e-07;
      t.Scale=1e-07;
      t.Velocity=1e-07;
      t.VelocityOrientation=1e-07;
      t.Acceleration=1e-07;
      t.AccelerationOrientation=1e-07;
      t.JointAngle=1e-03;
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
  };


  static bool Equal(const gazebo::physics::WorldState& s1,
                    const gazebo::physics::WorldState& s2,
                    const Tolerances& tolerance=Tolerances::Default,
                    const bool checkLights=true);
  static bool Equal(const gazebo::physics::ModelState& s1,
                    const gazebo::physics::ModelState& s2,
                    const Tolerances& tolerance=Tolerances::Default);
  static bool Equal(const gazebo::physics::LightState& s1,
                    const gazebo::physics::LightState& s2,
                    const Tolerances& tolerance=Tolerances::Default);
  static bool Equal(const gazebo::physics::LinkState& s1,
                    const gazebo::physics::LinkState& s2,
                    const Tolerances& tolerance=Tolerances::Default,
                    const bool checkCollisionStates=true);
  static bool Equal(const gazebo::physics::JointState& s1,
                    const gazebo::physics::JointState& s2,
                    const Tolerances& tolerance=Tolerances::Default);

  //  \param orientationTolerance orientation is compared with the Euler
  //         angles instead of with Quaternions, so this is the angular tolerance (radians).
  static bool Equal(const gazebo::math::Pose& p1,
                    const gazebo::math::Pose& p2,
                    const double& positionTolerance,
                    const double& orientationTolerance);
};

}

#endif  //  COLLISION_BENCHMARK_GAZEBOSTATE_COMPARE_HH
