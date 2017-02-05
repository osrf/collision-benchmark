# Questions for next chat

- Iterations are not counted globally if World::RunBlocking is called with steps!=0. Is this intended?
  In this case if we control the world update process from outside (repeated runBlocking calls) we can't really use
  iterations...
- Problem solved: Rubble world sometimes doesn't load up the rubbles, especially noticable if several worlds loaded. Tracked down to 
  World::Step():  LoadPlugins() not called because SensorsInitialized() return false for some worlds.
  However SensorManager::Update is never called - in fact World::dataPtr::sensorsInitialized ***is not initialized in constructor***,
  and it's set randomly to true or false. If always set to false, rubble world will never work!
  Actually sensors::run_once(true) has to be called, which is done from Server::Run(), but we don't use this.
  We need to manually hack this by calling World::_SetSensorsInitialized(true); in GazeboWorldLoader.cc


# Issues to address soon

- After setting the world to a state, and before advancing the world, the states should be completely equal even if dynamics is
  disabled - currently the acceleration has to be skipped in the comparison. See also transfer_world_state tutorial.
    --> look into it
- Contacts are only added to ContactManager if there is a subscriber. We should be able to enforce contact computation
  via the World interface directly.
- The mirror world is still not great. It is a bit weird with forwarding information such as iterations and time.
  Also, it cannot display the actual contacts, and it cannot be used to manipulate the world via gzclient. Should think about
  a better solution here.
    --> could maybe make a GUI plugin which disables controls fields (e.g. moving models, pausing world)?
- See the tutorial transfer_world_state: Sometimes the world updates only slowly to the rubble state. This is an issue which should
  be examined with Gazebo soon. The state comparison test in the tutorial code ensures that the states are in fact
  equal, but there must be a delay in communicating this to gzclient.
    - Known bug on gzclient side, probably a message being dropped
        - https://bitbucket.org/osrf/gazebo/issues/821/apparent-transport-race-condition-on
        - Or: https://bitbucket.org/osrf/gazebo/issues/681

-
