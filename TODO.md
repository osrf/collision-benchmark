# Questions for next chat

- Iterations are not counted globally if World::RunBlocking is called with steps!=0. Is this intended?
  In this case if we control the world update process from outside (repeated runBlocking calls) we can't really use
  iterations...
- Problems with topic forwarding:
    - rendering::Scene keeps visuals associated with parents, and that's the world for the root, so there's problems switching (see Scene::ProcessVisualMsg)
    - No methods are provided to delete models from server-side. All has to be tricked, e.g. creating modified model messages where
      visual's delete_me field is set to true, and even then it wasn't deleting the shapes, I abandoned it for the other "hacky" solution reasons before getting it to work.
      The visuals were gone (not in the map any more) but still shown and there were still messages kept in the system which did try to keep deleting it (see e.g. AddPendingChild and instances
      of ProcessVisualMsg returning false).

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
