# Questions for discussion

- Problem solved: **Iterations** are not counted globally if World::RunBlocking is called with steps!=0. Is this intended?
  In this case if we control the world update process from outside (repeated runBlocking calls) we can't really use
  iterations...

- Problem which was solved but needs later looking into to improve, because it's kind of a hack:
  **Rubble world sometimes doesn't load up the rubbles**, especially noticable if several worlds loaded. Tracked down to 
  World::Step():  LoadPlugins() not called because SensorsInitialized() return false for some worlds.
  However SensorManager::Update is never called - in fact World::dataPtr::sensorsInitialized ***is not initialized in constructor***,
  and it's set randomly to true or false. If always set to false, rubble world will never work!
  Actually sensors::run_once(true) can be called, which is done from Server::Run(), but we don't use this.
  We need to manually hack this by calling ``World::_SetSensorsInitialized(true);`` in GazeboWorldLoader.cc

- ODE does not compute contact points between static objects, while bullet does.
  Do we inted this? Example: Load up two-shapes test with cafe_table and bookshelf.

- colliding_shapes_test bullet ode -m beer -m coke_can: bullet detects collision too early

# Issues to address (may require PRs to gazebo)

- After setting the world to a state, and before advancing the world, the states should be completely equal even if dynamics is
  disabled - currently the acceleration has to be skipped in the comparison. See also transfer_world_state tutorial.

- **gzclient sometimes get stuck**. See the tutorial transfer_world_state: Sometimes the world updates only slowly to the rubble state. This is an issue which should
  be examined with Gazebo soon. The state comparison test in the tutorial code ensures that the states are in fact
  equal, but there must be a delay in communicating this to gzclient.
  UPDATE: Maybe this has been fixed? Could be related to [PR 2657](https://bitbucket.org/osrf/gazebo/pull-requests/2657), check again if this is still happening.
  See also:
      - https://bitbucket.org/osrf/gazebo/issues/821/apparent-transport-race-condition-on
      - Or: https://bitbucket.org/osrf/gazebo/issues/681

- Changing **DART shapes from Meshes to Primitives** can be done by somehow creating access to
  dart/constraint/ConstraintSolver to set FCLCollisionDetector::PRIMITIVE) in dart/collision/fcl/FclCollisionDetector
  (see call to setPrimitiveShapeType() in .cpp file line 73). Right now it's not recommended because of
  [issue 106](https://github.com/flexible-collision-library/fcl/issues/106)


# Improvements which can still be made

- The mirror world is still not perfect, it is a bit weird with forwarding information such as iterations and time.

- Extension for [PR 2661](https://bitbucket.org/osrf/gazebo/pull-requests/2661): Add the function for all entities
  See also [this issue](https://bitbucket.org/osrf/gazebo/issues/2242/adding-method-to-physics-world-which)

- Improvement of static test: Allow "agreement"/"disagreement" to be specified
  with strategy pattern instead of with simple boolean function only.
  Particularly useful would be to adjust penetration depth, so consider a state
  to be non-intersecting if the penetration depth is very slow, which can help
  when comparing primitive vs. mesh collisions of same shape (because mesh
  representation will sometimes not collide when the actual primitive does).
    - Variation: Instead of engine agreement, test for certain
      quality metrics (e.g. contact point quality metrics) or any other
      evaluation criteria.

- Improvement of static test in interactive mode: Don't save all results, but
 ask first whether this result should be saved, and display filename it would
 have.

- multiple_worlds_server.cc: Can only interrupt with Ctrl+c and then it's
 not shut down nicely. Find a better way to do this.

- Use collision_benchmark::GetConsistentAABB() (in test/TestUtils.hh)
  which checks that all AABBs are the same in both worlds. Separate tests
  still need to be designed using this function with a number of
  complex models.

# References

Libraries to maybe look into again for primitive/mesh generation

- [Ogre procedural](https://bitbucket.org/transporter/ogre-procedural) seems a bit outdated / not maintained, links broken
- [Libmesh](http://libmesh.github.io/doxygen/index.html)
- [GeometronLib](https://github.com/LukasBanana/GeometronLib)
- [generator](https://github.com/ilmola/generator)
- [geometrictools](www.geometrictools.com) always nice for algorithm references
