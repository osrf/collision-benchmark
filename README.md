# collision-benchmark

Benchmark tests for collision checking and contact

## Simulating multiple parallel worlds

The benchmark tests are based on a framework which allows to run multiple worlds
with their own physics engine each simultaneously.

### Simple initial test

Test to load multiple worlds at the same time and print their state at each iteration:

``multiple_worlds_server <number-of-simulation-steps> <list-of-worlds>``

**Example**

```
multiple_worlds_server 1 \
    test_worlds/cube_ode.world \
    test_worlds/cube_dart.world
```
