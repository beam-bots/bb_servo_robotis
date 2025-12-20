# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Servo.Robotis do
  @moduledoc """
  Beam Bots integration for Robotis/Dynamixel servos.

  This library provides controller, actuator, and sensor modules for integrating
  Dynamixel servos with the Beam Bots robotics framework. It uses the `robotis`
  hex package for low-level serial communication.

  ## Features

  - **Closed-loop position feedback**: Unlike PWM servos, Dynamixel servos report
    their actual position, enabling true closed-loop control.
  - **Multiple servos on one bus**: All servos share a single serial connection
    through the controller.
  - **Safety integration**: Torque is automatically disabled when the robot is
    disarmed or crashes.

  ## Supported Servos

  Currently supports XM430 series servos (W210, W350) via the U2D2 USB adapter.
  The library uses Protocol 2.0.

  ## Example Usage

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

              sensor :position, {BB.Servo.Robotis.Sensor,
                servo_id: 1,
                controller: :dynamixel,
                poll_interval_ms: 20
              }
            end
          end
        end
      end

  ## Modules

  - `BB.Servo.Robotis.Controller` - Manages the serial connection to the U2D2
  - `BB.Servo.Robotis.Actuator` - Sends position commands to servos
  - `BB.Servo.Robotis.Sensor` - Reads actual position from servos
  """
end
