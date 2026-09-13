# 5150E High Stakes Robot Code

Competition software used by **Danbury Mad Hatters Envy (5150E)** during the 2024–25 VEX V5 Robotics Competition: **High Stakes** season.

This repository is a recruiting-focused snapshot of the robot code. It preserves the competition source largely as it was used during the season, including commented alternative autonomous sequences that were kept for rapid iteration at events. This repository is shared for portfolio and code-review purposes. No license is granted for reuse, modification, or redistribution.

## My Role

I served as **Head Programmer & Builder** on 5150E and was the primary developer responsible for the robot-specific software, including autonomous routines, drivetrain and control tuning, sensor integration, subsystem automation, testing, and competition iteration.

This was a **team project**. The repository is shared to demonstrate my robotics programming work and does not imply that every line of the codebase was written solely by me.

## Technologies

- C++
- PROS for VEX V5
- LemLib
- VEX V5 motors, pneumatics, and sensors
- PID-based motion control
- IMU-based odometry/localization
- Concurrent RTOS tasks

## Technical Highlights

### Drivetrain and autonomous control
- Configured a LemLib drivetrain with separate lateral and angular PID controllers.
- Used IMU feedback and chassis odometry for autonomous translation, turning, and pose-based movement.
- Developed multiple autonomous routines for alliance color, starting position, elimination strategy, and skills runs.

### Sensor-driven subsystem automation
- **Optical color sorting:** detected red/blue rings from RGB ratios and automatically rejected opposing-color rings.
- **Automatic intake recovery:** monitored intake torque and reversed the intake when a sustained jam was detected.
- **Mobile-goal acquisition:** used distance sensing to trigger the goal clamp and adjust approach behavior.
- **Lift positioning:** used rotation-sensor feedback for automated loading, scoring, and reset positions.

### Concurrent robot behavior
Several subsystem behaviors run as independent PROS tasks so sensing and mechanism control can continue while the drivetrain is moving. Examples include ring detection, color sorting, goal clamping, intake recovery, lift control, and telemetry display.

## Repository Layout

```text
.
├── README.md
├── include/
│   └── main.h
├── src/
│   └── main.cpp
└── docs/
    └── CODE_GUIDE.md
```

The original PROS/LemLib framework files, generated build artifacts, editor configuration, and vendored libraries are intentionally omitted so the repository stays focused on the team-authored robot logic.

## About the Commented Code

Large commented sections in `src/main.cpp` are intentionally preserved. During competition, autonomous paths and mechanism sequences often needed to be adjusted quickly for field conditions, robot changes, or strategy. Keeping previous versions near the active routine was faster and safer for our event workflow than repeatedly replacing entire functions.

For a production software project I would normally use version control and smaller modules instead. Here, the commented alternatives are part of the historical competition code and show the iteration process used during the season.

## Build Notes

This snapshot is intended primarily for code review rather than as a self-contained build. The original project used **PROS 3.8.0** with **LemLib 0.5.0-rc.5** and included framework-generated headers/libraries that are not committed here.

To recreate a buildable project, create a matching PROS V5 project, install the corresponding LemLib version, and place `src/main.cpp` and `include/main.h` into that project.

## Competition Results

The team earned multiple awards and qualifications during my time with 5150E, including tournament championships, Design and Excellence awards, and VEX Worlds qualification.

## Notes for Reviewers

If you are skimming the code, start with `docs/CODE_GUIDE.md`, which points to the most representative control, sensing, and autonomous sections.
