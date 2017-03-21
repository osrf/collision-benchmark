# collision-benchmark

Test framework for collision engines.

This package provides a general interface to physics engine's worlds which
can be used for a variety of physics engine implementations.

The main purpose of this framework is to help with testing collision
and physics engines.

One main idea behind this is to compare the output of different physics engine
implementations and find cases in which the engines significantly disagree
on a specific situation (e.g. collision / no collision). Such cases can
be helpful for debugging.

## Dependencies

You will reqiure 

* Gazebo (currently only newest version compiled from source is tested)
  and its dependencies
* Boost libraries
* Assimp
* libvtk

Current requirement is also to use the [dart-6](https://bitbucket.org/JenniferBuehler/gazebo/branch/dart-6)
branch for Gazebo (see [PR #2547](https://bitbucket.org/osrf/gazebo/pull-requests/2547/dart-6/diff)).
Moreover, for some functionality with the contact points it needs to be
the [dart-6-dev](https://bitbucket.org/JenniferBuehler/gazebo/src/6b01e236886123ae0a9d3c585a841a303bb8a3bd/?at=dart-6-dev)
branch which is also merged with

- the PR for [contact_manager_enforcable](https://bitbucket.org/JenniferBuehler/gazebo/branch/contact_manager_enforcable)
(see [PR #2629](https://bitbucket.org/osrf/gazebo/pull-requests/2629/possibility-to-enforce-contact-addition-in/diff)).
- branch [user_cmd_mangager_using_world_name](https://bitbucket.org/JenniferBuehler/gazebo/src/a6a05d9575229ca1d73dcc8f7e40ef1da7a1307e?at=user_cmd_mangager_using_world_name) (see also [issue 2186](https://bitbucket.org/osrf/gazebo/issues/2186/usercmdmanager-does-not-use-world-name-to))
  recommended for full functionality.
- fixes for DART in branch [dart-6-fix-like-PR-2654](https://bitbucket.org/JenniferBuehler/gazebo/src/952623bc3c907c85237b29757790698386fdff0c/gazebo/physics/dart/DARTPhysics.cc?at=dart-6-fix-like-PR-2654&fileviewer=file-view-default#DARTPhysics.cc-195)
  which is what PR 2654 (listed above) does for for bullet.
  This will become a PR once dart-6 is merged with default.
- [PR 2657](https://bitbucket.org/osrf/gazebo/pull-requests/2657)
- [PR 2660](https://bitbucket.org/osrf/gazebo/pull-requests/2660/)
- [PR 2661](https://bitbucket.org/osrf/gazebo/pull-requests/2661)

If you are using dart-6-dev, those changes are merged already.


## Install

If you compiled Gazebo from source, don't forget to source ``<your-gazebo-install-path>/share/setup.sh`` to set up the 
gazebo environment variables. We will in particular need the ``GAZEBO_RESOURCE_PATH``.

Compile:

```
cd build
cmake ..
make
make tests
make install
```

Make sure that:

1. the library ``libcollision_benchmark_gui.so`` compiled by this project (installed to ``<your-install-prefix>/lib``)
is in your GAZEBO_PLUGIN_PATH, and
2. that you add ``<your-install-prefix>/share`` to your GAZEBO_RESOURCE_PATH.
3. You may also want to add the ``<your-install-prefix>/bin`` path to your PATH.


## Simulating multiple parallel worlds

The benchmark tests are based on a framework which allows to run multiple worlds
with their own physics engine each simultaneously.

### One world, different engines

The "multiple worlds server" allows you to specify one world file to load, and several physics engines the world
should be loaded with. This means the world will be loaded multiple times, once with each physics engine,
and then you can switch between the worlds, just like with the previous simple test. The
difference is that you have to specify only one world file. It is also possible to control all worlds (e.g. pausing)
using gzclient.

Print the help of ``multiple_worlds_server`` to learn more about the options:

```
multiple_worlds_server --help
```

You may start the multiple worlds server as follows:

```
multiple_worlds_server <world file> -e <list of phyiscs engines>
```

The ``<list of physics engines>`` can contain *ode, bullet, simbody* and *dart*
(the latter three are only supported if your Gazebo version installed
 supports it).

This will prompt you with a keypress so you get the chance to start gzclient before the worlds are started.
Confirm with ``[Enter]`` immediately if you don't want to wait, otherwise press ``[Enter]`` after gzclient has loaded up.

Start gzclient and you will see the world with the engine with the name which comes first in lexicographical order.
You should load up the gzclient with the GUI plugin which will allow you to control switching between the worlds:

```
gzclient --g libcollision_benchmark_gui.so
```

Make sure *libcollision_benchmark_gui.so* is in the *GAZEBO_PLUGIN_PATH*.

You can switch between the physics engines with the GUI control panel displayed on the top left.
Between the buttons, the name of the currently displayed world is shown, which should contain the physics
engine in the name.

The world is started in paused mode. You may press the "play" button in the client, or hit ``[Enter]``
in the terminal where you started the server, to start the simulation.

You can also pause the simulation and advance it with individual steps only, and you may add models to
the world, or change poses of the existing models - all of which will apply to **all** the worlds
loaded. So you can think of gzclient as the control interface to all the worlds.
This way you can compare how the same world behaves in different physics engines, how the
contact points compare, etc.

Note that not all controls from within the client (or by sending messages to the server)
are supported yet for the ``collision_benchmark`` framework. Always watch the terminal where you
started the server for hints that some functionality has been blocked, if you get no reaction
from the client controls.

**Example:**

``multiple_worlds_server worlds/rubble.world -e bullet ode``

This will load up the rubble world with the bullet and ODE engines. Use the GUI control panel to switch
between the bullet and ode worlds.

Now start the gzclient:

```
gzclient --g libcollision_benchmark_gui.so
```

### Multiple worlds, one or several engines

You may also use the ``multiple_worlds_server`` to load up several worlds
in Gazebo and switch between them.

```
multiple_worlds_server <list of world files>
```

For example:

```
multiple_worlds_server worlds/rubble.world worlds/empty.world
```

As before, you can start up the gzclient with the world switching GUI
plugin to switch between the worlds:

```
gzclient --g libcollision_benchmark_gui.so
```

You may even load up several worlds with several physics engines.
What this will do is load up each world *w* with each physics engine *p*,
creating a total of *w * p* worlds.

```
multiple_worlds_server <list of world files> -e <list of phyiscs engines>
```

Example: 

```
multiple_worlds_server worlds/rubble.world worlds/empty.world -e bullet ode
```


## Physics engine testing

The framework's main purpose is to test physics engines, not to just
run multiple worlds in parallel.
Several tests can be designed and implemented with the help of the API
provided with this framework. All tests that benefit from this framework
have one characteristing in common: They compare the behaviour of several
worlds. Typically, all worlds will therefore contain the same or equivalent
objects. The same world may for example be loaded with different physics
engines running them; or the same world may be loaded multiple times with
the same physics engine, but the representation of the worlds is different,
for example one world uses primitive shapes while the other world replaces
all primitive shapes by equivalent mesh representations.

The directory *test* contains source code for all testing related source code.

To build the tests, you need to do

``make tests``


### The "static tests"

The "static tests" comprise a number of test cases in which the dynamics
in the physics engine(s) are disabled, meaning that the world will not react
to the physics. However contact points between objects are still computed.
The static tests are therefore suitable for testing collision and
contact properties.

The main method of testing implemented so far is the "AABB intersection test"
in which two object's axis-aligned bounding boxes are intersected in many
possible ways, which generally leads to many states in which the objects are
colliding, and also states in which they are not (but in which they are
still close to each other). The world with the two objects is loaded multiple
times, either with different physics engines, or with different representations
of the objects (e.g. primitive *vs.* mesh), or both.   
The test iterates through all AABB intersection states, moves the objects in the
worlds accordingly so all worlds are in the same state, and compares the
output of the worlds. If the worlds disagree about the collision state,
a failure is triggered.

The static tests can be run interactively or automated.
In interactive mode, the test will stop at each failure so you can inspect
it with gzclient. The default is automated mode.

The test can also optionally
be set to save all the failure cases to *.world* files, which can be opened
for inspection at a later point. The default is to not write any files.

To run the test:

```
./static_test [--interactive] [--output <your-output-path>] [<gtest parameters>]
```

For example, to run only the particular test named *SpherePrimMesh*
(for test names please refer to *test/Static_TEST.cc*)
in interactive mode and save files to */home/me/test/*:

```
./static_test --gtest_filter=*SpherePrimMesh* --interactive --output /home/me/test
```

In interactive mode, when the test prompts you to hit [Enter] to continue,
you have time to start gzclient with the world switching GUI interface: 

```
gzclient --g libcollision_benchmark_gui.so
```

Then hit [Enter] in the terminal running the test and watch
the test unfold. If it stops due to a failure, it will prompt you to hit
[Enter] again to continue. Before you continue, you may switch between
the worlds in gzclient and inspect the results. Some information will also
have been printed in the terminal about the test failure details.


## Short introduction to the API

The API aims at subsuming a large number of physics engine implementations under one common
interface. This way several physics engines operating under the same interface can be used
together and compared.

The main interface of the API can be found in *PhysicsWorld.hh*. There are a few classes which provide
different interfaces, each depending on different template types. The main class which puts most
of the interfaces together is **PhysicsWorld**. However for accessing a world, a pointer of only
a partial interface may be required (which does not depend on many template parameters).

There are the following abstract (pure virtual) interfaces which define different sets of 
functionalities in relation to simulated worlds:

*   **PhysicsWorldBaseInterface** is the most basic interface which does not depend on template types.
    This interface is as independent of the underlying physics engine as possible. It provides the most
    basic functionality.
*   **PhysicsWorldStateInterface** is a very basic interface which only has one template type specifying the world
    state. So there can be several different physics engines operating under this interface:
    the only requirement is that engines that are compared via this interface support the same *world state* type.
    The world state can be defined as a general data type which can be supported by many different engines.
    It is expected to contain all information about the world.    
    The idea is that there may be several different physics engines, all supporting the same world state.
    With this, states of the different worlds can be directly compared.
    This can then include engines which are not implemented in the Gazebo interfaces yet.
*   **PhysicsWorldModelInterface** is an interface which defines functions to access
    and load models in the world. The interface only requires to specify the ID of
    models used (e.g. a std::string to identify a model via its name),
    but not the actual data type for a model.
    Therefore, this interface is still fairly idependent of the physics engine, the only restriction being
    that all classes operating under this interface must support the same identifier types for models.
*   **PhysicsWorldContactInterface** is an interface which defines functions to access contact points.
*   **PhysicsWorldEngineInterface** adds a few functions which provide low-level access to the 
    underlying implementation of the physics engine (e.g. retrieve shared pointers to specific model
    types) . This interface is highly dependent on the physics engine used.

There are two main abstract classes which combine the above interfaces into a common interface:

*   **PhysicsWorld** implements *PhysicsWorldBaseInterface*, *PhysicsWorldStateInterface*,
    *PhysicsWorldModelInterface* and *PhysicsWordlContactInterface*.
    It provides the base for all physics worlds with access to almost the entire functionality provided
    by the interfaces, except *PhysicsWorldEngineInterface* which is highly physics engine dependent.
*   **PhyicsEngineWorld** derives from *PhysicsWorld* and further implements *PhysicsWorldEngineInterface*.
    It is therefore very specific to the physics engine used.
    It requires the types of the classes for the models and the contact points and optionally, of 
    the world class and of the class for a physics engine.
    This interface is mainly useful as a common superclass
    for the more specific implementations.
    It can be used to get low-level pointers to the actual model types.

One full implementation of *PhysicsEngineWorld* is **GazeboPhysicsWorld**.

The main idea is that several implementations of the same template instantiation of *PhysicsWorld*
(which is still quite engine-independent) can be provided for different engines.
For example, all engines supported in Gazebo are automatically available via *GazeboPhysicsWorld*.
We may then add a new "brute-force" implementation of a physics engine which derives from the same *PhysicsWorld*
template instantiation as *GazeboPhysicsWorld* does. We can then directly compare contact points,
model states, and anything else in the WorldState, between the Gazebo implementation(s) and
the brute-force implementation. This can be very useful for testing and debugging of the
physics engines already integrated into Gazebo, or to test new engines which may be added to Gazebo.


The world interfaces define the access to the physics worlds themselves.
Beyond that, there are a few other classes worth mentioning here
which can be useful for managing several worlds at the same time.

* **MirrorWorld** is a world which can be set to mirror a *PhysicsWorld*.
  Because this is only a mirror to the other *original* world it mirrors,
  manipulation of the original world is not be possible via this interface.
  It is only possible to view the original world in the mirror.
  The *MirrorWorld* can be useful for scenarios such as visualization of a
  world, where the mirror world is the one used for displaying the original
  world; it can be switched to display a different world.
  An example implementation is the *GazeboTopicForwardingMirror* which redirects
  messages of all eligible topics in the original world to a designated target
  topic. The Gazebo Client may connect to this target topic. Behind the scenes
  it is then easy to change the mirror world to forward messages from another
  world, thereby allowing to easily switch between worlds to be displayed in
  the client.
* **WorldManager** can be used to maintain a number of worlds. The class
  provides functionality to access the same method in all worlds, for
  example to add the same model to all worlds.
  In addition to this, the WorldManager can maintain a MirrorWorld.
  It accepts messages to control switching the world currently
  mirrored by the mirror world.
* **MultipleWorldsServer** provides a server which can be used to run one or
  more world with multiple physics engines.  This interface offers methods to
  make the use of multiple worlds easier, including the maintenance of a
  WorldManager and providing methods for loading, starting and stopping worlds.
 

## API tutorials 

The tutorials are mainly documented in the given source files and only briefly described there. Please refer to the
mentioned source files for more information. All tutorials use the *GazeboPhysicsWorld* to demonstrate the use of
the interfaces with the Gazebo implementation.

To make the tutorials:

``make tutorials``

### TODO: Add simple tutorials using MultipleWorldsServer and the WorldManager


### Transfering a world state

The tutorial in the file [transfer_world_state.cc](https://github.com/JenniferBuehler/collision-benchmark/blob/devel-2/tutorials/transfer_world_state.cc)
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
set to the rubble world state.

Please refer to the code in 
[transfer_world_state.cc](https://github.com/JenniferBuehler/collision-benchmark/blob/devel-2/tutorials/transfer_world_state.cc)
which contains detailed documentation about how this is done.
