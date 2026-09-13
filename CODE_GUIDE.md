# Code Guide

`src/main.cpp` is the primary robot program. The file is large because it contains hardware configuration, background tasks, driver control, and many competition-specific autonomous routines in one source file.

## Suggested sections to review

### Hardware and control configuration
Near the top of `main.cpp`:
- Motor groups and drivetrain configuration
- IMU, optical, distance, rotation, and vision sensors
- LemLib lateral and angular controller settings
- Odometry/chassis initialization

### Background subsystem tasks
Inside `initialize()`:
- `findRingColor` — classifies rings using optical sensor RGB values
- `colorSort` — rejects the opposing alliance color during autonomous operation
- `clampGoal` / `clampGoalv2` — automates mobile-goal acquisition using distance sensing and drivetrain feedback
- `unstuckIntake` — detects sustained torque and performs automatic intake jam recovery
- `loadLift`, `scoreStake`, `liftToStart` — rotation-feedback lift positioning
- `holdRing` — coordinates intake behavior during autonomous sequences

### Autonomous routines
The middle of the file contains multiple routes developed and tuned throughout the season, including:
- skills autonomous routines
- red/blue goal rush routines
- sweep rush routines
- ring-side routines
- solo AWP routines
- state/worlds variants

The commented alternatives are intentionally retained because they represent competition iterations and quick strategy changes, not accidental dead code.

### Competition entry points
Near the end of the file:
- `autonomous()` selects/runs the active autonomous behavior
- `opcontrol()` contains driver-control and mechanism control logic

## What I would change in a new project

This code was optimized for rapid competition iteration. In a new long-lived codebase, I would split hardware configuration, subsystem controllers, autonomous paths, and driver control into separate modules; store autonomous variants in dedicated files or data structures; and rely on Git history/branches instead of retaining large commented alternatives inline.
