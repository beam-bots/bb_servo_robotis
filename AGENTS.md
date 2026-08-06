<!--
SPDX-FileCopyrightText: 2025 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# AGENTS.md

This file provides guidance to AI coding agents when working with code in this repository.

## Project Overview

BB.Servo.Robotis is a Beam Bots integration library for driving Dynamixel servos via
the Robotis Protocol 2.0 over serial. It provides controller, actuator, and parameter
bridge modules that plug into the BB robotics framework's DSL.

Unlike PWM servos, Dynamixel servos provide closed-loop position feedback, so no
separate sensor module is required.

## Build and Test Commands

```bash
mix check --no-retry    # Run all checks (compile, test, format, credo, dialyzer, reuse)
mix test                # Run tests
mix test path/to/test.exs:42  # Run single test at line
mix format              # Format code
mix credo --strict      # Linting
```

The project uses `ex_check` - always prefer `mix check --no-retry` over running individual tools.

## Architecture

### Component Hierarchy

```
Controller (GenServer)
    |
    v wraps
Robotis (Serial communication)
    ^
    | used by
Actuator (GenServer) --writes goal_position--> ETS table <--reads/clears-- Controller
    |
    v registers with (receives ETS table ref)
Controller --publishes--> JointState (position feedback)
          --publishes--> ServoStatus (status monitoring)

Bridge (GenServer) --reads/writes--> Controller --reads/writes--> Servo registers
```

### Key Modules

- **Controller** (`lib/bb/servo/robotis/controller.ex`) - GenServer wrapping the `Robotis` library.
  Owns a shared ETS table and runs a fixed-rate control loop (default 100Hz) that reads pending
  commands from ETS, writes them to the bus, reads positions via `fast_sync_read`, and publishes
  `JointState` messages. Status polling runs on a counter within the same loop. Implements the
  `BB.Controller` behaviour, including its `disarm/1` safety callback.

  Every write to `torque_enable` goes through the controller, so it caches each servo's torque
  state in the ETS row. That lets an actuator tell whether its goal will be acted on without a
  bus round trip. `{:resume_servo, id, goal}` is the ordered way back under power: the goal is
  written and acknowledged *before* torque comes on, so a servo that has drifted while passive
  doesn't lunge for the goal it was chasing when torque was cut.

- **Actuator** (`lib/bb/servo/robotis/actuator.ex`) - GenServer that receives position commands
  (radians), converts to servo position (0-4095), writes `goal_position` to the controller's
  ETS table, and publishes `BB.Message.Actuator.BeginMotion` messages. Accepts commands sent via:
  - `BB.Actuator.set_position/4` (pubsub)
  - `BB.Actuator.set_position!/4` (direct)
  - `BB.Actuator.set_position_sync/5` (synchronous)

  All three arrive at `handle_command/2`; `BB.Actuator.Server` checks arm state and applies
  the joint's transmission before the driver sees them.

  It also handles `Command.Stop` (cut torque, joint goes passive) and `Command.Hold` (re-apply
  torque where the joint is now resting; a no-op if it never went passive). Any command to a
  joint left passive by a `Stop` resumes on the way past, so callers needn't pair the two.

  `:mode` fixes the servo's operating mode at startup and decides what
  `command_payloads/1` declares — position, velocity, current, or current-based position.
  Anything outside the mode's list is refused by the framework with
  `BB.Error.State.UnsupportedCommand`. The mode is validated against the servo's reported
  `model_number` (see **Model** below) and written once while torque is already off, because
  changing it resets PID gains, profile velocity/acceleration and goal current — so it is never
  changed at runtime.

- **Model** (`lib/bb/servo/robotis/model.ex`) - What each Dynamixel can do, keyed on the
  `model_number` it reports: supported operating modes, and the torque constant (Nm/A) published
  in its specification table, used to turn `Command.Effort` into a current. Neither is readable
  from the bus or derivable from the control table, and one bus can mix models — an XM430 and an
  XL430 answer to the same control table but only one does current control. Unknown models are
  drivable in position mode and refused for anything else.

- **Bridge** (`lib/bb/servo/robotis/bridge.ex`) - Parameter bridge exposing servo control table
  parameters through the BB parameter system. Parameters are identified as `"servo_id:param_name"`.

- **ParamMetadata** (`lib/bb/servo/robotis/bridge/param_metadata.ex`) - Metadata for control table
  parameters. Categorises parameters as info (read-only), config (requires torque off), or control
  (runtime writable).

- **ServoStatus** (`lib/bb/servo/robotis/message/servo_status.ex`) - Message struct for servo status
  information (temperature, voltage, current, hardware errors).

### BB Framework Integration

The library uses BB's:
- `BB.Controller` behaviour for controller lifecycle
- `BB.Actuator` behaviour for actuator lifecycle
- `BB.Bridge` behaviour for parameter bridge
- `BB.Message` for typed message payloads
- `BB.Safety` API to register the controller and report hardware errors
- `BB.publish`/`BB.subscribe` for hierarchical PubSub by path
- `BB.Process.call` to communicate with sibling processes via the robot registry
- ETS for low-latency command passing from actuators to controller
- `Spark.Options` for configuration validation
- Joint limits from robot topology to derive servo parameters

### Command Interface

Send commands using the `BB.Actuator` module:

```elixir
# Arm first — a disarmed robot refuses commands before they reach the driver
{:ok, cmd} = MyRobot.arm()
{:ok, :armed, _} = BB.Command.await(cmd)

# Pubsub delivery (for orchestration/logging). Takes a name or a full path.
BB.Actuator.set_position(MyRobot, :servo, 0.5)
BB.Actuator.set_position(MyRobot, [:base, :shoulder, :servo], 0.5)

# Direct delivery (fire-and-forget, lower latency)
BB.Actuator.set_position!(MyRobot, :servo, 0.5)

# Synchronous delivery (with acknowledgement)
{:ok, :accepted} = BB.Actuator.set_position_sync(MyRobot, :servo, 0.5)

# Go passive — the joint can be backdriven by hand, and will sag under load
BB.Actuator.stop(MyRobot, :servo)

# Back under power, holding wherever it came to rest
BB.Actuator.hold(MyRobot, :servo)

# Only in an actuator configured `mode: :velocity` / `:current`
BB.Actuator.set_velocity(MyRobot, :wheel, 2.0, duration: 500)
BB.Actuator.set_effort(MyRobot, :gripper, 0.4)
```

`stop/3` is not the safety path: it leaves the robot armed and commandable. Making the
hardware safe is `MyRobot.disarm()`, which is robot-wide.

### Integration Pattern

```elixir
defmodule MyRobot do
  use BB

  controllers do
    controller :dynamixel, {BB.Servo.Robotis.Controller,
      port: "/dev/ttyUSB0",
      baud_rate: 1_000_000,
      control_table: Robotis.ControlTable.XM430
    }
  end

  parameters do
    bridge :robotis, {BB.Servo.Robotis.Bridge, controller: :dynamixel}
  end

  topology do
    link :base do
      joint :shoulder do
        type :revolute

        limit lower: ~u(-90 degree), upper: ~u(90 degree),
              velocity: ~u(60 degree_per_second), effort: ~u(1 newton_meter)

        actuator :servo, {BB.Servo.Robotis.Actuator,
          servo_id: 1,
          controller: :dynamixel
        }

        link :upper_arm do
        end
      end
    end
  end
end
```

### Testing

Tests use Mimic to mock `BB`, `BB.Process`, `BB.Robot`, and `Robotis`. Test support modules are
in `test/support/`.

## Dependencies

- `bb` - The Beam Bots robotics framework
- `robotis` - Low-level Robotis/Dynamixel Protocol 2.0 driver

### Message Flow

```
BB.Actuator.set_position()
    |
    v
Actuator receives Command.Position
    |
    v
Actuator writes goal_position to ETS table
    |
    v
Actuator publishes BeginMotion

Controller tick loop (unified, default 100Hz):
    |
    v
1. Read pending commands from ETS → write_raw to servos → clear ETS commands
    |
    v
2. Read present_position via fast_sync_read → update ETS → publish JointState
    |
    v
3. Every N ticks: read temperature/voltage/current/errors → publish ServoStatus
    |
    v
4. Report hardware errors to BB.Safety
```

### Supported Control Tables

- `Robotis.ControlTable.XM430` - XM430 series (W210, W350)
- `Robotis.ControlTable.XL330` - XL330-M288
- `Robotis.ControlTable.XL320` - XL320 (has different parameters)


## Licensing headers

Every source file must carry an SPDX header — a `#`-style comment for code, an
HTML comment for Markdown, or a `<file>.license` sidecar for files that can't
hold comments (binaries, JSON, lockfiles). `mix check` runs `reuse lint` and
fails the build if one is missing.

When you create a new file, its `SPDX-FileCopyrightText` line must credit **the
user you are working for** — not you (the agent), and not this repo's original
author. Take their name from `git config user.name` (add their `user.email` if
you include one) and use the current year. Match the neighbouring files'
`SPDX-License-Identifier` (usually `Apache-2.0`):

```
SPDX-FileCopyrightText: <current year> <your user's name>

SPDX-License-Identifier: Apache-2.0
```

Never copy an existing file's copyright line onto a new file — that credits the
wrong person. When you only edit an existing file, leave its headers unchanged.
