# collision-benchmark

Benchmark tests for collision checking and contact

## Dependencies

You will reqiure 

* Gazebo and dependencies
* Assimp
* Boost libraries

## Setup

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

Make sure the library ``libcollision_benchmark_gui.so`` compiled by this project (installed to ``<your-install-prefix>/lib``)
is in your GAZEBO_PLUGIN_PATH, and that you add ``<your-install-prefix>/share`` to your GAZEBO_RESOURCE_PATH.
You may also want the ``<your-install-prefix>/bin`` path in your PATH.

## Simulating multiple parallel worlds

The benchmark tests are based on a framework which allows to run multiple worlds
with their own physics engine each simultaneously.

### Simple initial test

There is a test which loads multiple worlds at the same and runs them simultaneously.

A simple

``make test``

will include a test that confirms different engines are loaded.

There is also a test which can be used to visually confirm that different worlds are loaded
and gzclient can display any of these worlds.

``multiple_worlds_server_test_simple <number-of-simulation-steps> <list-of-worlds>``

Every ``<number-of-iterations>`` iterations the next loaded world is considered
the world to be displayed in gzclient. The intention
of this test is to demonstrate how several worlds can be run, and how it is possible to switch
between those worlds for displaying in gzclient.

You need to start gzclient when prompted to do so. Press [Enter] to continue
as soon as gzclient is up and running.

Please note that you cannot use the gzclient controls like Pause and Step with this project
yet. Also, the iterations and simulation time etc. displayed at the bottom
may not reflect the actual simulation time.

**Example 1 without gzclient**

The test can be used to quickly confirm that different engines have in fact
been loaded for different worlds.

```
multiple_worlds_server_test_simple 1 \
    test_worlds/empty_ode.world \
    test_worlds/empty_bullet.world
```

Then just press ``[Enter]`` without loading gzclient to continue the test.
This will update each world once, then switch to the second as the "main" world
and update each world once again. At each update it should print "ode" and "bullet" as collision engines loaded.


**Example 2 with gzclient**

You may also look at the test with gzclient. Use a higher number of iterations for that:

```
multiple_worlds_server_test_simple 800 \
    test_worlds/cube_ode.world \
    test_worlds/sphere_bullet.world
```

Then load gzclient in another terminal:

``gzclient``

and then press ``[Enter]`` in the first terminal to continue with the test.

After 800 iterations, this should switch from the cube to the sphere.

### One world, different engines

It is also possible to specify one world file to load, and several physics engines the world
should be loaded with. This means the world will be loaded multiple times, once with each physics engine,
and then you can switch between the worlds, just like with the previous simple test. The
difference is that you have to specify only one world file.

You may start the Multiple worlds server as follows:

```
multiple_worlds_server <world file> <list of phyiscs engines>
```

The ``<list of physics engines>`` can contain *ode, bullet, simbody* and *dart*.

This will prompt you with a keypress so you get the chance to start gzclient before the worlds are started.
Confirm with ``[Enter]`` immediately if you don't want to wait, otherwise press ``[Enter]`` after gzclient has loaded up.

Start gzclient and you will see the world with the engine with the name which comes first in lexicographical order.
You should load up the gzclient with the GUI plugin which will allow you to control switching between the worlds:

```
gzclient --g libcollision_benchmark_gui.so
```

Make sure *libcollision_benchmark_gui.so* is in the *GAZEBO_PLUGIN_PATH*.

You can swith between the physics engines with the GUI control panel displayed on the top right.
Between the buttons, the name of the currently displayed world is shown, which should contain the physics
engine in the name.

Please note that you cannot use the gzclient controls like Pause and Step with this project
yet. Also, the iterations and simulation time etc displayed at the bottom
may not reflect the actual simulation time.

**Example:**

``multiple_worlds_server worlds/rubble.world bullet ode``

This will load up the rubble world with the bullet and ODE engines. Use the GUI control panel to switch
between the bullet and ode worlds.


## Short introduction to the API

The interface of the API can be found in *PhysicsWorld.hh*. There are a few classes which
inherit its parent and gradually refine the interface to be more specific to physics engines.
They are all template classes which accept certain types to define the interface. The further
down the hierarchy, the more types are added to the interface.

*   **PhysicsWorldBase** is the most basic interface which only depends on a template to specify a world
    state. So there can be several different physics engines operating under this basic interface, as long
    as they all adapt to the same world state type. This interface is as independent of the underlying physics
    engine as possible. The world state is expected to have all information about the world.
*   **PhysicsWorld** derives from *PhysicsWorldBase* and is a bit more specific in that it adds methods to 
    load models to the world and query it for contact points. The interface only requires to specify the ID of
    models (e.g. a std::string to identify a model via its name), but not the actual implementation of a model
    class. Therefore, this interface is still fairly idependent of the physics engine.
*   **PhyicsEngineWorld** derives from *PhysicsWorld* and is even more specific to the physics engine used.
    It requires the types of the classes for the the models and the contact points and optionally, of 
    the world class and of the class for a physics engine. This interface is mainly useful as a common superclass
    for the more specific implementations. It can be used to get low-level pointers to the actual model types.

The main implementation of the main interface is **GazeboPhysicsWorld**. It derives from *PhysicsEngineWorld* and
implements the full interface.

The idea is that there may be several implementations of physics engines, all operating on the same world state.
The most useful but still abstract iterface is probably *PhysicsWorld*. Several implementations of the same
template instantiation of *PhysicsWorld* can be added for different engines. For example, all engines supported in
Gazebo are automatically available via *GazeboPhysicsWorld*. We may then add a new brute-force implementation of a
physics engine which derives from the same *PhysicsWorld* instantiation as *GazeboPhysicsWorld* does. We can
then directly compare contact points, collision states, and anything else in the WorldState, between the Gazebo
implementation and the brute-force implementation. This can be very useful for testing and debugging.


## API tutorials 

The tutorials are mainly documented in the given source files and only briefly described there.
To make the tutorials:

``make tutorials``


### Transfering a world state

The tutorial in the file *transfer_world_state.cc* shows how to use the basic interface of **PhysicsWorldBase** to
load a world form an SDF file. 

Two worlds are loaded: The first world loaded in Gazebo is going to be the one which will be displayed with gzclient.
This will be the empty world to start with.
The second world created will be loaded with the rubble world, which is a world with quite a bit of movement in
it, as the rubble collapses.

Then, we want to see how we can use a *world state* object to set the world to a certain state.
To do this, we will first get the state of the rubble world, and set the empty world to the same state after
a number of iterations. So while you are looking at the first world with gzclient, it should switch to show
the rubble world after a while.


Start the tutorial:

``transfer_world_state``

You will receive a command line prompt when the tutorial is ready to start. Then, in another terminal, load up
gzclient:

``gzclient``

Ideally make sure you can see the both the first terminal and the Gazebo GUI at the same time.

Then go back to the first terminal and press ``[Enter]`` to start the tutorial.

After a while, the rubble world should suddenly pop up, as the state of the world is set to the rubbble world state.

Please refer to the code in *transfer_world_state.cc*  which contains detailed documentation about how this is done.



**TODO**

Sometimes you may notice that the world updates only slowly to the rubble state. This is an issue which should
be examined with Gazebo soon. The state comparison test in the tutorial code ensures that the states are in fact
equal, but there must be a delay in communicating this to gzclient.
