# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Error.Protocol.Robotis.HardwareAlert do
  @moduledoc """
  Robotis/Dynamixel servo hardware alert error.

  Raised when a Dynamixel servo reports hardware error status flags.
  The `alerts` field contains a list of active alert atoms.

  ## Alert Types

  - `:input_voltage` - Input voltage out of range
  - `:overheating` - Motor temperature too high
  - `:motor_encoder` - Motor encoder malfunction
  - `:electrical_shock` - Electrical shock detected
  - `:overload` - Motor overload detected
  """
  use BB.Error,
    class: :protocol,
    fields: [:servo_id, :alerts, :raw_value]

  @type alert ::
          :input_voltage | :overheating | :motor_encoder | :electrical_shock | :overload

  @type t :: %__MODULE__{
          servo_id: non_neg_integer(),
          alerts: [alert()],
          raw_value: non_neg_integer()
        }

  defimpl BB.Error.Severity do
    def severity(_), do: :critical
  end

  @alert_bits [
    {0, :input_voltage},
    {2, :overheating},
    {3, :motor_encoder},
    {4, :electrical_shock},
    {5, :overload}
  ]

  @doc """
  Creates a HardwareAlert from raw hardware error bits.
  """
  @spec from_bits(non_neg_integer(), non_neg_integer()) :: t()
  def from_bits(servo_id, bits) when is_integer(bits) and bits > 0 do
    alerts =
      @alert_bits
      |> Enum.filter(fn {bit, _name} -> Bitwise.band(bits, Bitwise.bsl(1, bit)) != 0 end)
      |> Enum.map(fn {_bit, name} -> name end)

    %__MODULE__{servo_id: servo_id, alerts: alerts, raw_value: bits}
  end

  def message(%{servo_id: servo_id, alerts: alerts}) do
    alert_str = Enum.join(alerts, ", ")
    "Servo #{servo_id} hardware alert: #{alert_str}"
  end
end
