# collision-benchmark

Benchmark tests for collision checking and contact

## Setup

If you compiled Gazebo from source, don't forget to source ``<your-gazebo-install-path>/share/setup.sh`` to set up the 
gazebo environment variables. We will in particular need the ``GAZEBO_RESOURCE_PATH``.

If you installed Gazebo with DART support, you may need to enable the cmake flag ``USE_DART`` to avoid
undefined references to DART:

``cmake -DUSE_DART=true``

Otherwise, compile as usual.

## Simulating multiple parallel worlds

The benchmark tests are based on a framework which allows to run multiple worlds
with their own physics engine each simultaneously.

### Simple initial test

There is a test which loads multiple worlds at the same and runs them simultaneously.

A simple

``make test``

will run a test that confirms different engines are loaded.

There is also a test which can be used to visually confirm that different worlds are loaded
and gzclient can be switched to display any of these worlds.

Every ``<number-of-iterations>`` iterations the next loaded world is considered
the world to be displayed (the "main world") which will be displayed by gzclient. The intention
of this test is to demonstrate how several worlds can be run, and how it is possible to switch
between those worlds for displayingin gzclient.

``multiple_worlds_server_test_simple <number-of-simulation-steps> <list-of-worlds>``

Then you can start gzclient and look at the world currently being displayed. Press [Enter] to continue
as soon as gzclient is up and running.

**Example**

```
multiple_worlds_server_test_simple 800 \
    test_worlds/cube_ode.world \
    test_worlds/sphere_bullet.world
```
Load gzclient and then press [Enter] to continue.

After 800 iterations, this should switch from the cube to the sphere.

**Simple example without gzclient**

This quick test can be used to confirm that different engines have in fact
been loaded for different worlds.

```
multiple_worlds_server_test_simple 1 \
    test_worlds/empty_ode.world \
    test_worlds/empty_bullet.world
```

Then just press [Enter] without loading gzclient to continue the test.
This will update each world once, then switch to the second as the "main" world
and update each world once again. At each update it should print "ode" and "bullet" as collision engines loaded.


