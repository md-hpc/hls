# Usage
`./run` will emulate the full system using the parameters and constants defined in `common.py`. The emulator will report the total number of cycles elapsed during the simulation.
`./cell-mapping` and `./uniform-spread` with both emulate a system with only one of the dimensions of parallelism. `./naive` will have no parallelism.

`./viz` will render the computed positions into an animated matplotlib scatterplot GIF called `md.gif`

Emulated hardware parameters:
* `{FORCE,FILTER}_PIPELINE_STAGES` defines the depth of pipelining for the particle filter and force evaluation pipeline
* `N_PPAR` defines the number of pipelines per compute bank (particle parallelism)
* `N_CPAR` defines the number of compute banks that work in parallel (cell parallelism)
* `N_PIPELINE` defines the level of parallelism for `./cell-mapping` and `./uniform-spread`. Has no affect on `./run`

Simulation parameters:
* `UNIVERSE_SIZE` size of the simulation box in cells (N_CELL = UNIVERSE_SIZE^3)
* `DT` timestep length
* `T` number of timesteps to simulate
* `DENSITY` average number of particles per cell (N_PARTICLE = N_CELL * DENSITY)
* `SEED` random seed for initialization. Set to `None` for random initial conditions

Physics parameters:
* `EPSILON` Constant for LJ computation
* `SIGMA` Constant for LJ computation

# Architecture
The emulated hardware uses both the cell mapping and uniform spread described in the TC 2024 paper. There are N_CPAR compute banks each with N_PPAR compute pipelines. Each compute bank works on a single home cell at a time, within which each compute pipeline works on a single reference cell at a time.

The computed forces are read from the pipelines into per-cell queues that are assumed to be of infinite length.

The forces of a reference particle are accumulated in a register before being sent to the acceleration update queues.

Double buffering is used to simplify particle migration between cells.

Positions, velocities, and accelerations are represented as numpy arrays. While they flow through the logic, they are often packaged into structs (`class Structure` in `common.py`) that contain their cell and address of origin for control flow.

Cell and particle distances modulo the universe dimensions are used to exploit N3L optimizations in the position readers and particle filters.

# Verification
In `verify.py` a set of nested for-loops executes a procedural version of the emulator's algorithm. At the end of each timestep, the emulator compares the memory contents of the position caches with the positions computed by this procedural algorithm.

For debugging purposes, the emulator also computes what it "expects" the particle filters and the force pipelines to actually recieve from the postion reader. This is done using set structures that are computed as the procedural algorithm mentioned above runs. As the filters receive inputs, they compare them with the members of the set to see:
* If the pair has been received already (duplicate)
* If the pair should not have been sent (unexpected)

And at the end of each timestep

* If any pairs in the set were not seen (expected)
