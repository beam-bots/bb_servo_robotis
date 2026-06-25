<!--
SPDX-FileCopyrightText: 2025 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# Change Log

All notable changes to this project will be documented in this file.
See [Conventional Commits](Https://conventionalcommits.org) for commit guidelines.

<!-- changelog -->

## [v0.3.2](https://github.com/beam-bots/bb_servo_pca9685/compare/v0.3.1...v0.3.2) (2026-06-25)




### Bug Fixes:

* disarm must confirm torque is disabled, not fire-and-forget (#57) (#58) by James Harton

### Improvements:

* support bb 0.20.3 robot_opts/0 child spec (#49) by James Harton

## [v0.3.1](https://github.com/beam-bots/bb_servo_pca9685/compare/v0.3.0...v0.3.1) (2026-05-28)




### Bug Fixes:

* bump bb to `~> 0.20`, use `set_robot_param_default` (#34) by James Harton

## [v0.3.0](https://github.com/beam-bots/bb_servo_pca9685/compare/v0.2.4...v0.3.0) (2026-05-21)




### Features:

* remove `reverse?`, move to motor-space (#31) by James Harton

## [v0.2.4](https://github.com/beam-bots/bb_servo_pca9685/compare/v0.2.3...v0.2.4) (2026-05-17)




## [v0.2.3](https://github.com/beam-bots/bb_servo_pca9685/compare/v0.2.2...v0.2.3) (2026-05-13)




### Improvements:

* add `bb_servo_robotis.install` igniter task (#28) by James Harton

## [v0.2.2](https://github.com/beam-bots/bb_servo_pca9685/compare/v0.2.1...v0.2.2) (2026-03-22)




### Improvements:

* controller: replace message-based writes with ETS-backed fixed-rate control loop (#21) by James Harton

## [v0.2.1](https://github.com/beam-bots/bb_servo_pca9685/compare/v0.2.0...v0.2.1) (2026-01-08)




### Improvements:

* use structured errors in parameter bridge (#6) by James Harton

## [v0.2.0](https://github.com/beam-bots/bb_servo_pca9685/compare/v0.1.0...v0.2.0) (2025-12-24)
### Breaking Changes:

* update to bb 0.8 wrapper GenServer pattern (#3) by James Harton



### Features:

* controller: report hardware errors to safety system by James Harton

### Bug Fixes:

* parameter bridge missing callbacks by James Harton

### Improvements:

* controller: make disarm behaviour configurable by James Harton


