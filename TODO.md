# Issues to address soon

- Contacts are only added to ContactManager if there is a subscriber. We should be able to enforce contact computation
  via the World interface directly.
- The mirror world is still not great. It's a bit weird with forwarding information such as iterations and time.
  Also, it can't display the actual contacts, and it cannot be used to manipulate the world via gzclient. Should think about
  a better solution here.
- See the tutorial transfer_world_state: Sometimes the world updates only slowly to the rubble state. This is an issue which should
  be examined with Gazebo soon. The state comparison test in the tutorial code ensures that the states are in fact
  equal, but there must be a delay in communicating this to gzclient. Also check why the accelerations are in fact not equal
  if the physics engine is disabled, it should still have been set to the same, if Update() wasn't called!
