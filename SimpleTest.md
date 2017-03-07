# Simple initial tests

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
cd build
./multiple_worlds_server_test_simple 1 \
    ../test_worlds/empty_ode.world \
    ../test_worlds/empty_bullet.world
```

Then just press ``[Enter]`` without loading gzclient to continue the test.
This will update each world once, then switch to the second as the "main" world
and update each world once again. At each update it should print "ode" and "bullet" as collision engines loaded.


**Example 2 with gzclient**

You may also look at the test with gzclient. Use a higher number of iterations for that:

```
cd build
./multiple_worlds_server_test_simple 2000 \
    ../test_worlds/cube_ode.world \
    ../test_worlds/sphere_bullet.world
```

Then load gzclient in another terminal:

``gzclient``

and then press ``[Enter]`` in the first terminal to continue with the test.

After 2000 iterations, this should switch from the cube to the sphere.
