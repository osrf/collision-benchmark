# collision-benchmark

Test framework for collision and physics engines.

You can find the documentation in the wiki pages of this repository.

## Short introduction to the collision_benchmark API

The API aims at subsuming several physics engine implementations under one common
interface. This way several physics engines operating under the same interface can be used
together and compared.

The main interface of the API can be found in
[PhysicsWorld.hh](collision_benchmark/PhysicsWorld.hh).
There are a few classes which provide different interfaces, each depending
on other template types. The main class which puts most
of the interfaces together is **PhysicsWorld**. However for accessing a world,
a pointer of only one of the interfaces may be required
(which does not depend on as many template parameters as PhysicsWorld).

There are the following abstract (pure virtual) interfaces which define different sets of
functionalities in relation to simulated worlds, all defined in
[PhysicsWorld.hh](collision_benchmark/PhysicsWorld.hh):

*   **PhysicsWorldBaseInterface** is the most basic interface which does not depend on template types.
    This interface is as independent of the underlying physics engine as possible. It provides the most
    basic functionality.
*   **PhysicsWorldStateInterface** is a very basic interface which only has
    one template type: the world state. So there can be several different
    physics engines operating under this interface -
    the only requirement is that they support the same *world state* type.
    The world state can be defined as a general data type which can be supported by many different engines.
    It is expected to contain all important information about the world.
    The idea is that there may be several different physics engines, all supporting the same world state.
    With this, states of the different worlds can be directly compared.
*   **PhysicsWorldModelInterface** is an interface which defines functions to access
    and load models in the world. The interface only requires to specify the ID of
    models used (e.g. a *std::string* to identify a model via its name),
    but not the actual data type for a model.
    Therefore, this interface is still fairly idependent of the physics engine, the only restriction being
    that all classes operating under this interface must support the same identifier types for models.
*   **PhysicsWorldContactInterface** is an interface which defines functions to access contact points.
*   **PhysicsWorldEngineInterface** adds a few functions which provide low-level access to the
    underlying implementation of the physics engine (e.g. retrieve shared pointers to specific model
    types) . This interface is highly dependent on the physics engine used.

There are two main abstract classes which combine the above interfaces into a common interface:

*   **PhysicsWorld** combines *PhysicsWorldBaseInterface*, *PhysicsWorldStateInterface*,
    *PhysicsWorldModelInterface* and *PhysicsWordlContactInterface*.
    It provides the base for all physics worlds with access to almost the entire functionality provided
    by the interfaces, except *PhysicsWorldEngineInterface* which is highly physics engine dependent.
*   **PhyicsEngineWorld** derives from *PhysicsWorld* and further implements *PhysicsWorldEngineInterface*.
    It is therefore very specific to the physics engine used.
    It requires the types of the classes for the models and the contact points and optionally, of
    the world class and of the class for a physics engine.
    This interface is mainly useful as a common superclass
    for the more specific implementations.
    It can be used to get low-level pointers to the actual world,
    model and contact types.

One full implementation of *PhysicsEngineWorld* is **GazeboPhysicsWorld**.

The main idea is that several implementations of the same template instantiation of *PhysicsWorld*
(which is still quite engine-independent) can be provided for different engines.
For example, all engines supported in Gazebo are automatically available via *GazeboPhysicsWorld*.
We may then add a new "brute-force" implementation of a physics engine which derives from the same *PhysicsWorld*
template instantiation as *GazeboPhysicsWorld* does. We can then directly compare contact points,
model states, and anything else in the *gazebo::physics::WorldState*
between the Gazebo implementation(s) and the brute-force implementation.
This can be useful for testing and debugging of the
physics engines already integrated into Gazebo, or to test new
engines which may be added to Gazebo.

The world interfaces define the access to the physics worlds themselves.
Beyond that, there are a few other classes worth mentioning here
which can be useful for managing several worlds at the same time.

* **MirrorWorld** is a world which can be set to mirror a *PhysicsWorld*.
  Because this is only a mirror to the other *original* world,
  manipulation of the original world is not be possible via this interface.
  It is only possible to view the original world in the mirror.
  The *MirrorWorld* can be useful for scenarios such as visualization of a
  world, where the mirror world is the one used for displaying the original
  world; the mirror can be switched to display a different world.
  An example implementation is the *GazeboTopicForwardingMirror* which redirects
  messages of all eligible topics in the original world to a designated target
  topic. The Gazebo Client *gzclient* may connect to this target topic.
  Behind the scenes it is then easy to change the mirror world to forward
  messages from another world, thereby allowing to easily switch between
  worlds to be displayed in the client.
* **WorldManager** can be used to maintain a number of worlds. The class
  provides functionality to access the same method in all worlds, for
  example to add the same model to all worlds.
  In addition to this, the WorldManager can maintain a MirrorWorld.
  It accepts a certain message type to control switching the world currently
  mirrored by the mirror world.
* **MultipleWorldsServer** provides a server which can be used to run one or
  more worlds with multiple physics engines. This class offers methods to
  make the use of multiple worlds easier, including the maintenance of a
  WorldManager and providing methods for loading, starting and stopping worlds.

## API tutorials

The tutorials are mainly documented in the given source files and only briefly described there. Please refer to the
mentioned source files for more information. All tutorials use the *GazeboPhysicsWorld* to demonstrate the use of
the interfaces with the Gazebo implementation.

To make the tutorials:

``make tutorials``

### Transfering a world state

The tutorial in the file [transfer_world_state.cc](tutorials/transfer_world_state.cc)
shows how to use the basic interface of **PhysicsWorldBase**.
It demonstartes how to load worlds form an SDF file and
set the worlds to a certain state. Two worlds will be loaded, then the state
from one is read, and the other world is set to the same state.

The first world is going to be the one which will be displayed with
*gzclient* (which always displays the first world that was loaded).
This will be the empty world to start with.
The second world will be loaded with the rubble world (*worlds/rubble.world*),
which is a world with quite a bit of movement in it, as you can watch the
rubble collapse.

Then, we want to see how we can use a *world state* to set the world to a
certain state.
To do this, we will first get the state of the rubble world, and set the
empty world to the same state after a number of iterations.
So while you are looking at the first world with gzclient, it should switch to
show the rubble world after a while.

From then on, at each iteration, we will set the first world to the state of
the second, so that the rubble moves just as in
the second world. The two world states should be identical at each loop.
The source code demonstrates how you can compare the
states to do sanity checks as well.

Start the tutorial:

``transfer_world_state``

You will receive a command line prompt when the tutorial is ready to start.
Then, in another terminal, load upgzclient:

``gzclient``

Ideally make sure you can see the both the first terminal and the Gazebo
GUI at the same time.

Then go back to the first terminal and press ``[Enter]`` to start the tutorial.

First, you will see the empty world for 1000 iterations.
Then, the rubble world should suddenly pop up, as the state of the world is
set to the rubble world state (a message like *"Now starting to set the world
to the rubble state"* should also be printed in the terminal)

Please refer to the code in
[transfer_world_state.cc](tutorials/transfer_world_state.cc),
which contains detailed documentation about how this is achieved.

### To come: More tutorials

**TODO: Add simple tutorials using MultipleWorldsServer and the WorldManager**
