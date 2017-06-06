#include <collision_benchmark/GazeboStateCompare.hh>
#include <collision_benchmark/GazeboHelpers.hh>
#include <collision_benchmark/MathHelpers.hh>

#include <gazebo/physics/WorldState.hh>
#include <gazebo/physics/ModelState.hh>
#include <gazebo/physics/LightState.hh>
#include <gazebo/physics/LinkState.hh>
#include <gazebo/physics/JointState.hh>
#include <gazebo/physics/CollisionState.hh>
#include <ignition/math/Vector3.hh>

using gazebo::physics::WorldState;
using gazebo::physics::ModelState;
using gazebo::physics::LightState;
using gazebo::physics::LinkState;
using gazebo::physics::JointState;
using gazebo::physics::CollisionState;
using ignition::math::Pose3d;
using collision_benchmark::GazeboStateCompare;

const GazeboStateCompare::Tolerances GazeboStateCompare::Tolerances::Default
        = GazeboStateCompare::Tolerances::CreateDefault();


bool GazeboStateCompare::Equal(const WorldState &s1, const WorldState &s2,
                               const Tolerances &tolerances,
                               const bool checkLights)
{
  if (s1.Insertions().size() != s2.Insertions().size() ||
      s1.Deletions().size() != s2.Deletions().size() ||
      s1.GetModelStates().size() != s2.GetModelStates().size() ||
      s1.LightStates().size() != s2.LightStates().size())
  {
#ifdef DEBUG
    std::cout << "State sizes not equal" << std::endl;
#endif
    return false;
  }

  std::vector<std::string>::const_iterator iter_str1, iter_str2;

  // check for equal insertions and deletions
  // XXX TODO make this more robust by makeing all lowercase and remove
  // white spaces. Also, sort insertions and deletions lexicographically
  // before comparing pair by pair to make
  // sure that the comparison is order-independent
  if (s1.Insertions().size() > 0)
    // already checked that insertions sizes of both states are equal
    // so we're good to go ahead without checking this again
    for (iter_str1 = s1.Insertions().begin(),
         iter_str2 = s2.Insertions().begin();
         iter_str1 != s1.Insertions().end(),
         iter_str2 != s2.Insertions().end();
         ++iter_str1, ++iter_str2)
    {
      if (*iter_str1 != *iter_str2)
      {
#ifdef DEBUG
        std::cout << "Insertion not equal: " << std::endl
                  << *iter_str1<<std::endl<<*iter_str2<<std::endl;
#endif
        return false;
      }
    }
  if (s1.Deletions().size() > 0)
    // already checked that deletions sizes of both states are equal
    // so we're good to go ahead without checking this again
    for (iter_str1 = s1.Deletions().begin(),
         iter_str2 = s2.Deletions().begin();
         iter_str1 != s1.Deletions().end(),
         iter_str2 != s2.Deletions().end();
         ++iter_str1, ++iter_str2)
    {
      if (*iter_str1 != *iter_str2)
      {
#ifdef DEBUG
        std::cout << "Deletion not equal: " << std::endl
                 <<*iter_str1<<std::endl<<*iter_str2<<std::endl;
#endif
        return false;
      }
    }

  // Compare model and light states. Because the maps are ordered
  // lexicographically by their key, we can directly compare them.
  // First, do the quick check whether the names are equal, then do the more
  // detailed check whether the actual states are equal.
  gazebo::physics::ModelState_M::const_iterator iter_mdl1, iter_mdl2;
  gazebo::physics::LightState_M::const_iterator iter_lt1, iter_lt2;

  // compare model states names
  if (s1.GetModelStates().size() > 0)
    // already checked that modelStates sizes of both states are equal
    // so we're good to go ahead without checking this again
    for (iter_mdl1 = s1.GetModelStates().begin(),
         iter_mdl2 = s2.GetModelStates().begin();
         iter_mdl1 != s1.GetModelStates().end(),
         iter_mdl2 != s2.GetModelStates().end();
         ++iter_mdl1, ++iter_mdl2)
    {
      if (iter_mdl1->first != iter_mdl2->first)
      {
#ifdef DEBUG
        std::cout << "Model state names not equal: "
                  << iter_mdl1->first << ", " << iter_mdl2->first << std::endl;
#endif
        return false;
      }
    }
  // compare light states names
  if (checkLights && s1.LightStates().size() > 0)
    // already checked that lightStates sizes of both states are equal
    // so we're good to go ahead without checking this again
    for (iter_lt1 = s1.LightStates().begin(),
         iter_lt2 = s2.LightStates().begin();
         iter_lt1 != s1.LightStates().end(),
         iter_lt2 != s2.LightStates().end();
         ++iter_lt1, ++iter_lt2)
    {
      if (iter_lt1->first != iter_lt2->first)
      {
#ifdef DEBUG
        std::cout << "Light names not equal: "
                  << iter_lt1->first << ", " << iter_lt2->first << std::endl;
#endif
        return false;
      }
    }

  // compare model states
  if (s1.GetModelStates().size() > 0)
    for (iter_mdl1 = s1.GetModelStates().begin(),
         iter_mdl2 = s2.GetModelStates().begin();
         iter_mdl1 != s1.GetModelStates().end(),
         iter_mdl2 != s2.GetModelStates().end();
         ++iter_mdl1, ++iter_mdl2)
    {
      if (!Equal(iter_mdl1->second, iter_mdl2->second, tolerances))
      {
#ifdef DEBUG
        std::cout << "Model states not equal." << std::endl;
#endif
        return false;
      }
    }

  // Compare light states
  if (checkLights && s1.LightStates().size() > 0)
    for (iter_lt1 = s1.LightStates().begin(),
         iter_lt2 = s2.LightStates().begin();
         iter_lt1 != s1.LightStates().end(),
         iter_lt2 != s2.LightStates().end();
         ++iter_lt1, ++iter_lt2)
    {
      if (!Equal(iter_lt1->second, iter_lt2->second, tolerances))
      {
#ifdef DEBUG
        std::cout << "Light states not equal." << std::endl;
#endif
        return false;
      }
    }

  return true;
}

bool GazeboStateCompare::Equal(const gazebo::physics::ModelState &s1,
                               const gazebo::physics::ModelState &s2,
                               const Tolerances &tolerances)
{
  if (s1.GetName() != s2.GetName())
  {
#ifdef DEBUG
    std::cout << "Model names not equal: "
             <<s1.GetName() << ", " << s2.GetName() << std::endl;
#endif
    return false;
  }

  if (!Equal(s1.Pose(), s2.Pose(), tolerances.Position, tolerances.Orientation))
  {
#ifdef DEBUG
    std::cout << "Poses of model " << s1.GetName() << " not equal: "<< std::endl
              << s1.Pose()<< std::endl <<s2.Pose() << std::endl;
#endif
    return false;
  }

  if (!EqualVectors(s1.Scale(), s2.Scale(), tolerances.Scale))
  {
    return false;
  }


  // Compare link, joint and model states.
  // Because the maps are ordered lexicographically by their key,
  // we can directly compare them.
  // First, do the quick check whether the names are equal, then do the
  // more detailed check whether the actual states are equal.
  gazebo::physics::LinkState_M::const_iterator iter_lnk1, iter_lnk2;
  gazebo::physics::JointState_M::const_iterator iter_jnt1, iter_jnt2;
  gazebo::physics::ModelState_M::const_iterator iter_mdl1, iter_mdl2;

  // compare link states names
  if (s1.GetLinkStates().size() > 0)
    for (iter_lnk1 = s1.GetLinkStates().begin(),
         iter_lnk2 = s2.GetLinkStates().begin();
         iter_lnk1 != s1.GetLinkStates().end(),
         iter_lnk2 != s2.GetLinkStates().end();
         ++iter_lnk1, ++iter_lnk2)
    {
      if (iter_lnk1->first != iter_lnk2->first)
      {
#ifdef DEBUG
    std::cout << "Poses not equal: " << s1.Pose() << ", " << s2.Pose() << std::endl;
#endif
        return false;
      }
    }

  // compare joint states names
  if (s1.GetJointStates().size() > 0)
    for (iter_jnt1 = s1.GetJointStates().begin(),
         iter_jnt2 = s2.GetJointStates().begin();
         iter_jnt1 != s1.GetJointStates().end(),
         iter_jnt2 != s2.GetJointStates().end();
         ++iter_jnt1, ++iter_jnt2)
    {
      if (iter_jnt1->first != iter_jnt2->first)
      {
#ifdef DEBUG
        std::cout << "Joint names not equal: "
                  << iter_jnt1->first << ", " << iter_jnt2->first << std::endl;
#endif
        return false;
      }
    }

  // compare nested model states names
  if (s1.NestedModelStates().size() > 0)
    for (iter_mdl1 = s1.NestedModelStates().begin(),
         iter_mdl2 = s2.NestedModelStates().begin();
         iter_mdl1 != s1.NestedModelStates().end(),
         iter_mdl2 != s2.NestedModelStates().end();
         ++iter_mdl1, ++iter_mdl2)
    {
      if (iter_mdl1->first != iter_mdl2->first)
      {
#ifdef DEBUG
        std::cout << "Nested model names not equal: "
                  << iter_mdl1->first << ", " << iter_mdl2->first << std::endl;
#endif
        return false;
      }
    }

  // compare link states
  if (s1.GetLinkStates().size() > 0)
    for (iter_lnk1 = s1.GetLinkStates().begin(),
         iter_lnk2 = s2.GetLinkStates().begin();
         iter_lnk1 != s1.GetLinkStates().end(),
         iter_lnk2 != s2.GetLinkStates().end();
         ++iter_lnk1, ++iter_lnk2)
    {
      if (!Equal(iter_lnk1->second, iter_lnk2->second, tolerances))
      {
#ifdef DEBUG
        std::cout << "Link states not equal: " << iter_lnk1->first << ", "
                  << iter_lnk2->first << std::endl;
#endif
        return false;
      }
    }

  // compare joint states
  if (s1.GetJointStates().size() > 0)
    for (iter_jnt1 = s1.GetJointStates().begin(),
         iter_jnt2 = s2.GetJointStates().begin();
         iter_jnt1 != s1.GetJointStates().end(),
         iter_jnt2 != s2.GetJointStates().end();
         ++iter_jnt1, ++iter_jnt2)
    {
      if (!Equal(iter_jnt1->second, iter_jnt2->second, tolerances))
      {
#ifdef DEBUG
        std::cout << "Joint states not equal: "
                  << iter_jnt1->first << ", " << iter_jnt2->first << std::endl;
#endif
        return false;
      }
    }

  // compare nested model states
  if (s1.NestedModelStates().size() > 0)
    for (iter_mdl1 = s1.NestedModelStates().begin(),
         iter_mdl2 = s2.NestedModelStates().begin();
         iter_mdl1 != s1.NestedModelStates().end(),
         iter_mdl2 != s2.NestedModelStates().end();
         ++iter_mdl1, ++iter_mdl2)
    {
      if (!Equal(iter_mdl1->second, iter_mdl2->second, tolerances))
      {
#ifdef DEBUG
        std::cout << "Nested model tates not equal" << std::endl;
#endif
        return false;
      }
    }

  return true;
}

bool GazeboStateCompare::Equal(const gazebo::physics::LightState &s1,
                               const gazebo::physics::LightState &s2,
                               const Tolerances &tolerances)
{
  ignition::math::Vector3d q1(s1.Pose().Rot().Euler());
  ignition::math::Vector3d q2(s2.Pose().Rot().Euler());

  return EqualVectors(s1.Pose().Pos(), s2.Pose().Pos(), tolerances.Position) &&
         EqualVectors(q1, q2, tolerances.Orientation);
}

bool GazeboStateCompare::Equal(const gazebo::physics::LinkState &s1,
                               const gazebo::physics::LinkState &s2,
                               const Tolerances &tolerances)
{
  if (s1.GetName() != s2.GetName())
  {
    return false;
  }

  if (tolerances.CheckLinkCollisionStates &&
      s1.GetCollisionStates().size() != s2.GetCollisionStates().size())
  {
#ifdef DEBUG
    std::cout << "Collision states not equal" << std::endl;
#endif
    return false;
  }

  if (!Equal(s1.Pose(), s2.Pose(), tolerances.Position, tolerances.Orientation))
  {
#ifdef DEBUG
    std::cout << "Poses not equal: " << s1.Pose() << ", " << s2.Pose() << std::endl;
#endif
    return false;
  }

  if (/*tolerances.CheckDynamics && */!Equal(s1.Velocity(), s2.Velocity(),
                                             tolerances.Velocity,
                                             tolerances.VelocityOrientation))
  {
#ifdef DEBUG
    std::cout << "Velocities not equal: " << s1.Velocity()
              << ", " << s2.Velocity() << std::endl;
#endif
    return false;
  }

  if (tolerances.CheckDynamics && !Equal(s1.Acceleration(), s2.Acceleration(),
                                         tolerances.Acceleration,
                                         tolerances.AccelerationOrientation))
  {
#ifdef DEBUG
    std::cout << "Accelerations not equal: " << s1.Acceleration()
              << ", " << s2.Acceleration() << std::endl;
#endif
    return false;
  }

  if (tolerances.CheckDynamics && !EqualVectors(s1.Wrench().Pos(),
                                                s2.Wrench().Pos(),
                                                tolerances.Force))
  {
#ifdef DEBUG
    std::cout << "Force not equal: " << s1.Wrench().Pos()
              << ", " << s2.Wrench().Pos() << std::endl;
#endif
    return false;
  }

  if (tolerances.CheckDynamics)
  {
    ignition::math::Vector3d q1(s1.Wrench().Rot().Euler());
    ignition::math::Vector3d q2(s2.Wrench().Rot().Euler());
    if (!EqualVectors(q1,q2, tolerances.Torque))
    {
#ifdef DEBUG
      std::cout << "Torque not equal: " << s1.Wrench().Rot().Euler()
                << ", " << s2.Wrench().Rot().Euler() << std::endl;
#endif
      return false;
    }
  }

  if (!tolerances.CheckLinkCollisionStates)
    return true;

  std::vector<CollisionState>::const_iterator iter1, iter2;

  for (iter1 = s1.GetCollisionStates().begin(),
       iter2 = s2.GetCollisionStates().begin();
       iter1 != s1.GetCollisionStates().end(),
       iter2 != s2.GetCollisionStates().end();
       ++iter1, ++iter2)
  {
    const CollisionState &c1=*iter1;
    const CollisionState &c2=*iter2;
    if (c1.GetName() != c2.GetName())
    {
#ifdef DEBUG
      std::cout << "CollisionState name not equal: " << s1.GetName()
                << ", " << s2.GetName() << std::endl;
#endif
      return false;
    }
    if (!Equal(c1.Pose(), c2.Pose(), tolerances.Position,
               tolerances.Orientation))
    {
#ifdef DEBUG
      std::cout << "CollisionState pose not equal: " << s1.Pose()
                << ", " << s2.Pose() << std::endl;
#endif
      return false;
    }
  }

  return true;
}

bool GazeboStateCompare::Equal(const gazebo::physics::JointState &s1,
                               const gazebo::physics::JointState &s2,
                               const Tolerances &tolerances)
{
  if (s1.GetName() != s2.GetName())
  {
#ifdef DEBUG
    std::cout << "JointState name not equal: " << s1.GetName() << ", "
              << s2.GetName() << std::endl;
#endif
    return false;
  }

  int i = 0;
  std::vector<double>::const_iterator iter1, iter2;
  for (iter1 = s1.Positions().begin(),
       iter2 = s2.Positions().begin();
       iter1 != s1.Positions().end(),
       iter2 != s2.Positions().end();
       ++iter1, ++iter1, ++i)
  {
    if (!EqualFloats(*iter1, *iter2, tolerances.JointAngle))
    {
#ifdef DEBUG
      std::cout << "JointState " << i << " not equal: "
                << *iter1 << ", "<< *iter2 << std::endl;
#endif
      return false;
    }
  }

  return true;
}

bool GazeboStateCompare::Equal(const ignition::math::Pose3d &p1,
                               const ignition::math::Pose3d &p2,
                               const double &positionTolerance,
                               const double &orientationTolerance)
{
  ignition::math::Vector3d q1(p1.Rot().Euler());
  ignition::math::Vector3d q2(p2.Rot().Euler());

  return EqualVectors(p1.Pos(), p2.Pos(), positionTolerance) &&
         EqualVectors(q1, q2, orientationTolerance);
}
