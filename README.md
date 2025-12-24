<!--
SPDX-FileCopyrightText: 2025 James Harton

SPDX-License-Identifier: Apache-2.0
-->

<img src="https://github.com/beam-bots/bb/blob/main/logos/beam_bots_logo.png?raw=true" alt="Beam Bots Logo" width="250" />

# Beam Bots Robotis/Dynamixel servo control

[![CI](https://github.com/beam-bots/bb_servo_robotis/actions/workflows/ci.yml/badge.svg)](https://github.com/beam-bots/bb_servo_robotis/actions/workflows/ci.yml)
[![License: Apache 2.0](https://img.shields.io/badge/License-Apache--2.0-green.svg)](https://opensource.org/licenses/Apache-2.0)
[![Hex version badge](https://img.shields.io/hexpm/v/bb_servo_robotis.svg)](https://hex.pm/packages/bb_servo_robotis)
[![REUSE status](https://api.reuse.software/badge/github.com/beam-bots/bb_servo_robotis)](https://api.reuse.software/info/github.com/beam-bots/bb_servo_robotis)

# BB.Servo.Robotis

BB integration for driving Dynamixel servos via the Robotis Protocol 2.0 over serial.

This library provides controller, actuator, and parameter bridge modules for
integrating Dynamixel servos with the Beam Bots robotics framework. Unlike
PWM servos, Dynamixel servos provide closed-loop position feedback.

## Features

- **Closed-loop position feedback** - Servos report their actual position
- **Multiple servos on one bus** - All servos share a single serial connection
- **Safety integration** - Torque is automatically disabled when the robot is disarmed or crashes
- **Status monitoring** - Temperature, voltage, current, and hardware error reporting
- **Parameter access** - Read and write servo configuration via the BB parameter system

## Supported Servos

Currently supports the following servo families via the U2D2 USB adapter:

- XM430 (W210, W350)
- XL330-M288
- XL320

## Installation

Add `bb_servo_robotis` to your list of dependencies in `mix.exs`:

```elixir
def deps do
  [
    {:bb_servo_robotis, "~> 0.2.0"}
  ]
end
```

## Requirements

- U2D2 USB adapter or compatible serial interface
- Dynamixel servos using Protocol 2.0
- BB framework (`~> 0.7`)

## Usage

Define a controller and joints with servo actuators in your robot DSL:

```elixir
defmodule MyRobot do
  use BB

  controller :dynamixel, {BB.Servo.Robotis.Controller,
    port: "/dev/ttyUSB0",
    baud_rate: 1_000_000,
    control_table: :xm430
  }

  topology do
    link :base do
      joint :shoulder, type: :revolute do
        limit lower: ~u(-90 degree), upper: ~u(90 degree), velocity: ~u(60 degree_per_second)

        actuator :servo, {BB.Servo.Robotis.Actuator,
          servo_id: 1,
          controller: :dynamixel
        }

        link :upper_arm do
          joint :elbow, type: :revolute do
            limit lower: ~u(-90 degree), upper: ~u(90 degree), velocity: ~u(60 degree_per_second)

            actuator :servo, {BB.Servo.Robotis.Actuator,
              servo_id: 2,
              controller: :dynamixel
            }

            link :forearm do
            end
          end
        end
      end
    end
  end
end
```

The actuator automatically derives its configuration from the joint limits - no
need to specify servo rotation range or speed separately. Position feedback is
handled by the controller; no separate sensor is needed.

## Sending Commands

Use the `BB.Actuator` module to send commands to servos. Three delivery methods
are available:

### Pubsub Delivery (for orchestration)

Commands are published via pubsub, enabling logging, replay, and multi-subscriber
patterns:

```elixir
BB.Actuator.set_position(MyRobot, [:base, :shoulder, :servo], 0.5)

BB.Actuator.set_position(MyRobot, [:base, :shoulder, :servo], 0.5,
  command_id: make_ref()
)
```

### Direct Delivery (for time-critical control)

Commands bypass pubsub for lower latency:

```elixir
BB.Actuator.set_position!(MyRobot, :servo, 0.5)
```

### Synchronous Delivery (with acknowledgement)

Wait for the actuator to acknowledge the command:

```elixir
case BB.Actuator.set_position_sync(MyRobot, :servo, 0.5) do
  {:ok, :accepted} -> :ok
  {:error, reason} -> handle_error(reason)
end
```

## Components

### Controller

`BB.Servo.Robotis.Controller` manages the serial connection to the Dynamixel
bus. Define one controller per U2D2 adapter. The controller handles:

- Serial communication with servos
- Position feedback polling for all registered servos
- Status monitoring (temperature, voltage, current, errors)
- Torque enable/disable on arm/disarm

**Options:**

| Option | Type | Default | Description |
|--------|------|---------|-------------|
| `port` | string | required | Serial port path (e.g., "/dev/ttyUSB0") |
| `baud_rate` | integer | 57600 | Baud rate in bps |
| `control_table` | atom | `:xm430` | Servo control table (`:xm430`, `:xl330_m288`, `:xl320`) |
| `poll_interval_ms` | integer | 50 | Position feedback polling interval (20Hz default) |
| `status_poll_interval_ms` | integer | 1000 | Status polling interval (0 to disable) |
| `disarm_action` | atom | `:disable_torque` | Action on disarm (`:disable_torque` or `:hold`) |

### Actuator

`BB.Servo.Robotis.Actuator` controls a single servo on the bus.

**Options:**

| Option | Type | Default | Description |
|--------|------|---------|-------------|
| `servo_id` | 1-253 | required | Dynamixel servo ID |
| `controller` | atom | required | Name of the controller in robot registry |
| `reverse?` | boolean | false | Reverse rotation direction |
| `position_deadband` | integer | 2 | Minimum position change (raw units) to publish feedback |

**Behaviour:**

- Maps joint position limits to servo position (0-360 degrees)
- Clamps commanded positions to joint limits
- Registers with controller for position feedback
- Publishes `BB.Message.Actuator.BeginMotion` after each command
- Torque management is delegated to the controller

### Position Feedback

Unlike PWM servos, Dynamixel servos report their actual position. The controller
polls all registered servos and publishes `BB.Message.Sensor.JointState` messages.
No separate sensor is needed in the robot definition.

Subscribe to position updates:

```elixir
BB.subscribe(MyRobot, [:sensor, :dynamixel, :shoulder])
```

### Status Monitoring

The controller periodically reads status registers and publishes
`BB.Servo.Robotis.Message.ServoStatus` messages containing:

- `temperature` - Internal temperature in Celsius
- `voltage` - Input voltage in Volts
- `current` - Current draw in Amps
- `hardware_error` - Hardware error flags (nil if no errors)

Hardware errors are also reported to the safety system.

Subscribe to status updates:

```elixir
BB.subscribe(MyRobot, [:sensor, :dynamixel, :servo_status])
```

### Parameter Bridge

`BB.Servo.Robotis.Bridge` exposes servo configuration through the BB parameter
system. Parameters are identified by strings in the format `"servo_id:param_name"`.

```elixir
defmodule MyRobot do
  use BB

  parameters do
    bridge :robotis, {BB.Servo.Robotis.Bridge, controller: :dynamixel}
  end
end

# Read parameter
{:ok, 800} = BB.Parameter.get_remote(MyRobot, :robotis, "1:position_p_gain")

# Write parameter (control params)
:ok = BB.Parameter.set_remote(MyRobot, :robotis, "1:position_p_gain", 1000)

# List all parameters
{:ok, params} = BB.Parameter.list_remote(MyRobot, :robotis)
```

Parameter categories:

- **info** - Read-only identification (model_number, firmware_version)
- **config** - EEPROM settings, require torque off to write (limits, operating_mode)
- **control** - RAM settings, writable at runtime (PID gains, profiles)

## How It Works

### Architecture

```
Controller (GenServer)
    |
    v wraps
Robotis (Serial communication)
    ^
    | used by
Actuator (GenServer) --publishes--> BeginMotion
    |
    v registers with
Controller --publishes--> JointState (position feedback)
          --publishes--> ServoStatus (status monitoring)
```

Multiple actuators share a single controller. Each actuator controls a servo
with a unique ID (1-253) on the bus.

### Position Mapping

The actuator maps the joint's position limits to the servo's position range:

```
Joint centre       ->  180 degrees (servo centre)
Joint lower limit  ->  180 - (range/2) degrees
Joint upper limit  ->  180 + (range/2) degrees
```

For a joint with limits `-90 degrees` to `+90 degrees`:
- `-90 degrees` maps to 90 degrees on the servo
- `0 degrees` maps to 180 degrees on the servo
- `+90 degrees` maps to 270 degrees on the servo

### Safety

The controller implements `BB.Safety`:

- **On arm** - Torque is enabled on all registered servos
- **On disarm** - Torque is disabled (or held, if configured)
- **On crash** - Torque is disabled without requiring GenServer state

Hardware errors detected during status polling are reported to the safety
system and may trigger disarm depending on your safety configuration.

### Motion Lifecycle

When a position command is processed:

1. Actuator clamps position to joint limits
2. Converts angle to servo position (0-4095 units)
3. Sends goal_position to controller
4. Controller writes to servo via Robotis library
5. Publishes `BB.Message.Actuator.BeginMotion` with:
   - `initial_position` - where the servo was
   - `target_position` - where it's going
   - `expected_arrival` - when it should arrive (based on velocity limit)
   - `command_id` - correlation ID (if provided)
   - `command_type` - `:position`

Position feedback is handled separately by the controller's polling loop.

## Documentation

Full documentation is available at [HexDocs](https://hexdocs.pm/bb_servo_robotis).
