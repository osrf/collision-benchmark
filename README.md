# collision-benchmark

Test framework for collision and physics engines.

This provides an API which acts as a general interface to physics engine's worlds.
The API can support a variety of physics engine implementations and is therefore useful
to compare performance and output of different engines.

One main idea behind this API is to compare the output of different physics
engine implementations and find cases in which the engines significantly disagree
on a specific situation (e.g. collision / no collision). Such cases can
be helpful for debugging.

The main implementation of the API provided to date uses Gazebo and therefore
all physics engines coming with Gazebo are supported.

## Table of Contents

1. [Installation](#installation)
1. [Simulating multiple parallel worlds](#simulating-multiple-parallel-worlds)
1. [Physics engine testing](#physics-engine-testing)
    - [Static tests](#the-static-tests)
    - [Dynamic tests](#the-dynamic-tests)
1. [Short introduction to the API](#short-introduction-to-the-api)

## Installation

### Dependencies

You will reqiure 

* Gazebo (currently only newest version compiled from source is tested)
  and its dependencies
* Boost libraries
* Assimp
* libvtk

To fulfill current requirements for some functionality, it is highly
recommended to compile Gazebo from source and use the
the [dart-6-devel](https://bitbucket.org/JenniferBuehler/gazebo/branch/dart-6-devel)
branch, which is the default branch merged with

- [PR 2657](https://bitbucket.org/osrf/gazebo/pull-requests/2657): problems with transport
- [PR 2708](https://bitbucket.org/osrf/gazebo/pull-requests/2708): add method World::SDF()
- [PR 2709](https://bitbucket.org/osrf/gazebo/pull-requests/2709): ODE contact points not constantly displayed
- [PR 2713](https://bitbucket.org/osrf/gazebo/pull-requests/2713): Faulty bounding boxes in bullet and dart
- only minor: [PR 2705](https://bitbucket.org/osrf/gazebo/pull-requests/2705)
- only minor: [PR 2707](https://bitbucket.org/osrf/gazebo/pull-requests/2707)

Please also see [dart PR 881](https://github.com/dartsim/dart/pull/881)
and [fcl PR 213](https://github.com/flexible-collision-library/fcl/pull/213)
which fix some issues with DART/fcl. At the time of writing,
the changes to fix the issues have not been merged yet.

If you compiled Gazebo from source, don't forget to source
``<your-gazebo-install-path>/share/setup.sh`` to set up the 
gazebo environment variables.
We will in particular need the ``GAZEBO_RESOURCE_PATH``.

### Compile and Setup

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

In order to be able to visualize all models you saved with the test
framework in gzclient, you will need to have the directory
``<temp-path>/.gazebo/models``
in your ``GAZEBO_RESOURCE_PATH``. This is required for enerated mesh shapes
which have to be written to file for Gazebo
(because SDF reads meshes from file).
``<temp-path>`` should be your default system path temp folder
(more specifically, the one returned by
 ``gazebo::common::SystemPaths::TmpPath()``):

``export GAZEBO_RESOURCE_PATH=${GAZEBO_RESOURCE_PATH}:/tmp/.gazebo/models``

## Simulating multiple parallel worlds

The framework allows to run multiple worlds with their own physics engine
each simultaneously. This way the output of different engines can be compared.

### One world, different engines

The "multiple worlds server" allows you to specify one world file to load, and several physics engines the world
should be loaded with. This means the world will be loaded multiple times, once with each physics engine,
and then you can switch between the worlds. It is possible to control all
worlds (e.g. pausing) using gzclient.

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

Now you may start gzclient. You should load start it with the GUI plugin,
which will allow you to control switching between the worlds:

```
gzclient --g libcollision_benchmark_gui.so
```

Make sure *libcollision_benchmark_gui.so* is in the *GAZEBO_PLUGIN_PATH*.

You will see the world with the engine with the name which comes first
in lexicographical order.

You can switch between the physics engines with the GUI control panel displayed on the top left.
Between the buttons, the name of the currently displayed world is shown, which should contain the physics
engine in the name.

The world is started in paused mode. You may press the "play" button in gzclient,
or hit ``[Enter]`` in the terminal where you started the server,
to start the simulation.

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

This will load up the rubble world with the bullet and ODE engines.

Now start the gzclient:

```
gzclient --g libcollision_benchmark_gui.so
```

Use the GUI control panels to start the world (press ``<Play>``) and 
switch between the bullet and ode worlds using the panel on the top left.

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

Place a cube or any other new model in the empty world, switch back to the
rubble world to see that the cube is in this world now as well.
Then switch back to the empty world again, to see that your
newly inserted model is still there as well.

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

This will load four worlds, twice the rubble world
and twice the empty world (each once with bullet and once with ODE).

## Physics engine testing

The main purpose of the framework is to test physics engines, not to just
run multiple worlds in parallel.
Several tests can be designed and implemented with the help of the API of 
this testing framework. All tests that benefit from this framework
have one characteristing in common: They compare the behaviour of several
worlds. Typically, all worlds will therefore contain the same or equivalent
objects. The same world may for example be loaded with different physics
engines running them; or the same world may be loaded multiple times with
the same physics engine, but the shape representation varies between the
worlds, for example one world uses primitive shapes while the other world
replaces all primitive shapes by equivalent mesh representations.

The directory *test* contains source code for all testing related source code.

To build the tests, you need to do

``make tests``

To run all tests, type

``make test``

or to run only a specific test, use the test executable directly and
select the test with gtest parameter ``--gtest_filter``:

``<test executable> --gtest_filter=*<pattern in test name>*``

for example, to run WorldInterfaceTest.TransferWorldState:

``<your-build-dir>/world_interface_test --gtest_filter=*TransferWorldState*``

In order to be able to visualize all shapes in gzclient, you
will need to have the directory
``<temp-path>/.gazebo/models``
in your ``GAZEBO_RESOURCE_PATH``, as described in the 
[Installation](#installation) section.
This is required because the tests use generated mesh shapes
which have to be written to file (because SDF reads meshes from file).

### The "static tests"

The "static tests" comprise a number of tests in which the *dynamics
in the physics engine(s) are disabled*, meaning that the world(s) will not react
to the physics. However contact points between objects are still computed.
The static tests are therefore suitable for testing collision and
contact properties.

The main method of testing implemented so far is the "AABB intersection test"
in which two object's axis-aligned bounding boxes are intersected in many
possible ways that they can intersect.
This generally leads to many states in which the objects are
colliding, and also states in which they are not (but in which they are
still close to each other).
Because this test uses two objects, we can refer to the test worlds as the
*"two objects world"*.

A two-objects-world is loaded multiple times: either multiple times with
different physics engines, or multiple times with different representations
of the objects (e.g. primitive *vs.* mesh), or both.

The test iterates through all AABB intersection states and moves the objects
accordingly with the AABB, so all worlds are in the same state. It
then compares the output of the worlds.
If the worlds disagree about the collision state, a failure is triggered.

The static tests can be run interactively or automated.
In interactive mode, the test will stop at each failure so you can inspect
it with gzclient. The default is automated mode, in which the failures are
printed and the test then contiues.

The test can also be set to save all the failure cases to *.world* files, which
can be opened for inspection at a later point.
The default is to not write any files.

To run the test:

```
./static_test [--interactive] [--output <your-output-path>] [<gtest parameters>]
```

If you don't specify an output path, world files won't be written to file.

For example, to run only the particular test named *SpherePrimMesh*
(for other test names please refer to
 [test/Static_TEST.cc](test/Static_TEST.cc)),
run it in interactive mode, and save world files to */home/me/test/*:

```
./static_test --gtest_filter=*SpherePrimMesh* --interactive --output /home/me/test
```

In interactive mode, when the test prompts you to hit ``[Enter]`` to continue,
you have time to start gzclient with the world switching GUI interface: 

```
gzclient --g libcollision_benchmark_gui.so
```

You should see the start state of the test. Before you start the test,
you may want to enable the displaying of contacts
(``View -> Contacts``), because once the test stops due to failure, it will be
paused. And in paused state, enabling the contacts displaying will only show
the contacts once the worlds are advanced again -  so you won't see the contacts
until the next test failure.
It will also be helpful to switch on wireframe rendering (``View -> Wireframe``)
to see the contacts better.

To start the test, hit ``[Enter]`` in the terminal running the test and watch
the test unfold. If it stops due to a failure, it will prompt you to hit
``[Enter]`` again to continue. Before you continue, you may switch between
the worlds in gzclient and inspect the results. Some information will also
have been printed in the terminal about the test failure details.

**Reminder:** If you wish to display the test results which were saved to file
in gazebo later, don't forget to start gazebo in paused mode, as you probably
would like the world to be displayed in the state it was in when the test
failed. The tests also don't use a ground floor, which means the objects will
be falling in free space.
You will also need to add ``<your-output-path>`` to the ``GAZEBO_RESOURCE_PATH``
in order to be able to display models which contain meshes.


### The "dynamic tests"

**WORK IN PROGRESS**

Examples:

``collide_test <list of physics engines> -m mailbox -m <path-to-SDF>``

``collide_test <list of physics engines> -m mailbox -s sphere``


## Short introduction to the API

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
