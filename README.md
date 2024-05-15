# Particle System

This is a plugin for MARS to simulate a static particle system.
Currently the particles are loaded by a MARS scene and this plugin
is only used for generating a `PositionMap` for the vertex shader
allowing to draw the x-closed particles instead of all. For example
always drawing 100 of 1000 distributed particles.

The plugin reads a configuration file from the MARS config folder
which can define multiple particle systems. For each system a
material name (`materialName`) have to be given and the texture
name (`textureName` / not the file name) encoding the particle
positions to render. Optionally a position file (`positionFile`)
can be defined. The file has a comma seperated list of all particle
positions. If no file is given the particles are distributed randomly.
An example configuration is given by:

    particles:
      - materialName: grassParticles
        textureName: particleMap
        positionFile: particle_locations.txt

An example for the `positionFile`:

    -10.0,21.3
    14.374,15.234
    0.0,0.0

The given material (defined in the MARS scene / smurf file) should define
`numInstances` and `instancesWidth`, since they are needed by the
particle system plugin.


Uses Octree implementation from:
git@github.com:mwarning/SimpleOctree.git
