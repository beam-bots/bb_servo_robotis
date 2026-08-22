<!--
SPDX-FileCopyrightText: 2025 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# Change Log

All notable changes to this project will be documented in this file.
See [Conventional Commits](Https://conventionalcommits.org) for commit guidelines.

<!-- changelog -->

## [v0.6.1](https://github.com/beam-bots/bb_servo_pca9685/compare/v0.6.0...v0.6.1) (2026-08-22)




### Improvements:

* declare `:position_feedback` in `capabilities/1` (#106) by James Harton

### Bug Fixes:

* reject the XL320 control table instead of crashing the bus process (#104) by James Harton

## [v0.6.0](https://github.com/beam-bots/bb_servo_pca9685/compare/v0.5.1...v0.6.0) (2026-08-09)
### Breaking Changes:

* drive the servos in the modes they actually implement (#92) by James Harton



## [v0.5.1](https://github.com/beam-bots/bb_servo_pca9685/compare/v0.5.0...v0.5.1) (2026-08-03)




### Bug Fixes:

* test: expect paren-less DSL from the installer (#87) by James Harton

### Improvements:

* drive the servo bus loop from `BB.Loop` (#89) by James Harton

## [v0.5.0](https://github.com/beam-bots/bb_servo_pca9685/compare/v0.4.0...v0.5.0) (2026-08-02)
### Breaking Changes:

* declare the command payloads this driver implements by James Harton



## [v0.4.0](https://github.com/beam-bots/bb_servo_pca9685/compare/v0.3.4...v0.4.0) (2026-08-01)
### Breaking Changes:

* migrate to `BB.Actuator.handle_command/2` (#82) by James Harton



## [v0.3.4](https://github.com/beam-bots/bb_servo_pca9685/compare/v0.3.3...v0.3.4) (2026-07-24)




### Bug Fixes:

* remove phantom Robotis sensor guidance (#76) by James Harton

## [v0.3.3](https://github.com/beam-bots/bb_servo_pca9685/compare/v0.3.2...v0.3.3) (2026-06-29)




### Bug Fixes:

* fetch control table lazily so the bridge survives controller startup ordering (#61) by James Harton

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


