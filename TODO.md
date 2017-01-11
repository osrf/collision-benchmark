# Issues to address soon

- Add a method to disable physics engines to the top-level interface - disabling the dynamics should be possible everywhere.
- After setting the world to a state, and before advancing the world, the states should be completely equal even if dynamics is
  disabled - currently the acceleration has to be skipped in the comparison. See also transfer_world_state tutorial.
- Contacts are only added to ContactManager if there is a subscriber. We should be able to enforce contact computation
  via the World interface directly.
- The mirror world is still not great. It is a bit weird with forwarding information such as iterations and time.
  Also, it cannot display the actual contacts, and it cannot be used to manipulate the world via gzclient. Should think about
  a better solution here.
- See the tutorial transfer_world_state: Sometimes the world updates only slowly to the rubble state. This is an issue which should
  be examined with Gazebo soon. The state comparison test in the tutorial code ensures that the states are in fact
  equal, but there must be a delay in communicating this to gzclient.

-
