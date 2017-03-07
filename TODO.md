# Questions for discussion

- Changing **DART shapes from Meshes to Primitives** can be done by somehow creating access to
  dart/constraint/ConstraintSolver to set FCLCollisionDetector::PRIMITIVE) in dart/collision/fcl/FclCollisionDetector
  (see call to setPrimitiveShapeType() in .cpp file line 73). Right now it's not recommended because of
  [issue 106](https://github.com/flexible-collision-library/fcl/issues/106)

- Problem solved: **Iterations** are not counted globally if World::RunBlocking is called with steps!=0. Is this intended?
  In this case if we control the world update process from outside (repeated runBlocking calls) we can't really use
  iterations...

- Problem solved but needs later looking into to improve, because it's kind of a hack:
  **Rubble world sometimes doesn't load up the rubbles**, especially noticable if several worlds loaded. Tracked down to 
  World::Step():  LoadPlugins() not called because SensorsInitialized() return false for some worlds.
  However SensorManager::Update is never called - in fact World::dataPtr::sensorsInitialized ***is not initialized in constructor***,
  and it's set randomly to true or false. If always set to false, rubble world will never work!
  Actually sensors::run_once(true) can be called, which is done from Server::Run(), but we don't use this.
  We need to manually hack this by calling ``World::_SetSensorsInitialized(true);`` in GazeboWorldLoader.cc

- Problem solved with [PR 2629](https://bitbucket.org/osrf/gazebo/pull-requests/2629/possibility-to-enforce-contact-addition-in/diff):
  **Contacts are only added to ContactManager if there is a subscriber**. We should be able to enforce contact computation
  via the World interface directly.

- Problem solved (**new mirror world which only forwards topics**. Contacts can be displayed. Old mirror world in CodeVault.):
  The mirror world is still not perfect, it is a bit weird with forwarding information such as iterations and time.

# Issues to address soon

- After setting the world to a state, and before advancing the world, the states should be completely equal even if dynamics is
  disabled - currently the acceleration has to be skipped in the comparison. See also transfer_world_state tutorial.

- **gzclient sometimes get stuck** and doesn't update as frequently, though it seems that the underlying world is getting updated.
  Especially apparent with rubble world.

- UPDATE: Has this problem been solved by updated code (now using world->Step(i) instead of gazebo::runWorld(), or addressed
                                                        ruble state issue discussed above)?
  See the tutorial transfer_world_state: Sometimes the world updates only slowly to the rubble state. This is an issue which should
  be examined with Gazebo soon. The state comparison test in the tutorial code ensures that the states are in fact
  equal, but there must be a delay in communicating this to gzclient.
    - Known bug on gzclient side, probably a message being dropped
        - https://bitbucket.org/osrf/gazebo/issues/821/apparent-transport-race-condition-on
        - Or: https://bitbucket.org/osrf/gazebo/issues/681
