<!--
SPDX-FileCopyrightText: 2026 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# BB.Servo.Robotis Usage Rules

`bb_servo_robotis` drives Robotis/Dynamixel servos (Protocol 2.0, over a serial
U2D2 adapter) for [Beam Bots](https://hexdocs.pm/bb). It supplies three
components: `BB.Servo.Robotis.Controller` (a `BB.Controller` bus manager),
`BB.Servo.Robotis.Actuator` (a `BB.Actuator`, one per servo), and
`BB.Servo.Robotis.Bridge` (a `BB.Bridge` exposing the servo control table). For
BB framework basics, see `bb`'s rules (`mix usage_rules.sync <file> bb:all`);
this file covers only what's Robotis-specific.

## Core principles

1. **One controller per serial bus, many actuators.** The controller owns the
   serial connection and a shared ETS table; each actuator writes its
   `goal_position` there, and the controller's tick loop batches writes and
   bulk-reads positions (`fast_sync_read`). Every actuator names its controller
   and carries a unique `servo_id` (1–253) on that bus.
2. **Torque and disarm live on the controller, not the actuator.** The
   controller registers with `BB.Safety`, enables torque on arm, and disables
   it on disarm/crash. The actuator's `disarm/1` is a deliberate no-op.
3. **No separate position sensor.** Dynamixels report their own position; the
   controller polls and publishes `BB.Message.Sensor.JointState`. Do not add a
   sensor for feedback — there is no `BB.Servo.Robotis.Sensor`.

## Wiring it in

Declare the controller (and, optionally, the parameter bridge), then attach an
actuator to each joint:

```elixir
defmodule MyRobot.Robot do
  use BB

  controllers do
    controller :dynamixel,
      {BB.Servo.Robotis.Controller,
       port: "/dev/ttyUSB0",
       baud_rate: 1_000_000,
       control_table: Robotis.ControlTable.XM430}
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

        actuator :servo,
          {BB.Servo.Robotis.Actuator, servo_id: 1, controller: :dynamixel}

        link :upper_arm do
        end
      end
    end
  end
end
```

The actuator derives its motion range from the joint's `limit` — you never set
servo range or speed on the actuator itself.

## Options

**Controller** (`BB.Servo.Robotis.Controller`):

| Option | Default | Meaning |
|---|---|---|
| `:port` | required | Serial port path, e.g. `"/dev/ttyUSB0"` |
| `:baud_rate` | `1_000_000` | Bus baud rate |
| `:control_table` | `Robotis.ControlTable.XM430` | Must match the servo family (XM430 / XL330 / XL320) |
| `:loop_interval_ms` | `10` | Control-loop period (≈100 Hz) |
| `:status_poll_interval_ms` | `1000` | Temp/voltage/current/error poll; `0` disables |
| `:disarm_action` | `:disable_torque` | `:disable_torque`, or `:hold` to keep position |

**Actuator** (`BB.Servo.Robotis.Actuator`): `:servo_id` (1–253, required),
`:controller` (the controller's DSL name, required), `:position_deadband`
(default `2`, raw units — filters feedback noise).

**Bridge** (`BB.Servo.Robotis.Bridge`): `:controller` (required). Parameters are
addressed as `"servo_id:param_name"`, e.g.
`BB.Parameter.get_remote(MyRobot.Robot, :robotis, "1:position_p_gain")`.

## Commanding motion

Commands are issued in joint-space; BB applies the joint `transmission` so the
actuator receives motor-space values. Arm the robot first — a disarmed robot
ignores motion commands:

```elixir
# Either the actuator's unique name or its full path — `[:base, :shoulder, :servo]`
# here. A partial path matches no subscriber and the command goes nowhere.
BB.Actuator.set_position(MyRobot.Robot, :servo, 0.5)
```

## Anti-patterns

- **Don't add a `sensor` for position feedback.** The controller publishes
  `JointState` under `[:sensor, <controller_name>, <joint>]`; there is no
  `BB.Servo.Robotis.Sensor` module.
- **Don't assume it runs under simulation.** Both the controller and the bridge
  default to `simulation: :omit`, so neither starts in simulation. Set
  `simulation: :mock` (or `:start`) if you need them there.
- **Don't manage torque per-actuator.** Torque is centralised on the controller
  via `:disarm_action`; the actuator's `disarm/1` is a no-op by design.
- **Don't share a `servo_id` or a serial `port`.** One controller per U2D2
  adapter, and each servo on the bus needs a distinct ID.

## Further reading

- [bb_servo_robotis docs](https://hexdocs.pm/bb_servo_robotis)
- `bb`'s actuator and safety rules (`bb:actuators`, `bb:safety-and-commands`)
  and [Writing an Actuator](https://hexdocs.pm/bb/12-writing-an-actuator.html)
