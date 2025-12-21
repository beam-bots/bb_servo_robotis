# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Servo.Robotis.Message.ServoStatus do
  @moduledoc """
  Status information for a Robotis/Dynamixel servo.

  Published periodically by the controller when status polling is enabled.
  Subscribe to `[:sensor, :controller_name, :servo_status]` to receive updates.

  ## Fields

  - `servo_id` - The servo ID (1-253)
  - `temperature` - Internal temperature in Celsius
  - `voltage` - Input voltage in Volts
  - `current` - Present current draw in Amps
  - `hardware_error` - Hardware error flags (nil if no errors)

  ## Hardware Error Flags

  The `hardware_error` field contains bit flags indicating error conditions:

  - Bit 0: Input voltage error
  - Bit 2: Overheating error
  - Bit 3: Motor encoder error
  - Bit 4: Electrical shock error
  - Bit 5: Overload error

  ## Examples

      alias BB.Servo.Robotis.Message.ServoStatus

      {:ok, msg} = ServoStatus.new(:dynamixel,
        servo_id: 1,
        temperature: 45.0,
        voltage: 12.1,
        current: 0.5,
        hardware_error: nil
      )
  """

  defstruct [
    :servo_id,
    :temperature,
    :voltage,
    :current,
    :hardware_error
  ]

  use BB.Message,
    schema: [
      servo_id: [type: :pos_integer, required: true, doc: "Servo ID (1-253)"],
      temperature: [type: :number, required: true, doc: "Temperature in Celsius"],
      voltage: [type: :number, required: true, doc: "Input voltage in Volts"],
      current: [type: :number, required: true, doc: "Current draw in Amps"],
      hardware_error: [
        type: {:or, [:non_neg_integer, {:literal, nil}]},
        default: nil,
        doc: "Hardware error flags (nil if no errors)"
      ]
    ]

  @type t :: %__MODULE__{
          servo_id: pos_integer(),
          temperature: number(),
          voltage: number(),
          current: number(),
          hardware_error: non_neg_integer() | nil
        }
end
