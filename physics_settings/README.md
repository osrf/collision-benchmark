#physics_settings

For each physics engine, a SDF file with default settings for the physics engine is given. Physics profiles as in [physics presets](https://bitbucket.org/osrf/gazebo_design/src/default/physics_presets/physics_presets.md) could be supported as well in future (see example [presets.world](https://bitbucket.org/osrf/gazebo/src/default/test/worlds/presets.world)).

Even better would be to support several physics engines in one world file, each with different presets, and allow to specify one engine and a preset at *loading time* of the world (switching engines after loading the world would be too involved as the world has to be re-created).

For now, each engine and each setting requires specification in one separate SDF file.
