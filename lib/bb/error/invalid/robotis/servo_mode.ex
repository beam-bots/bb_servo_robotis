# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Error.Invalid.Robotis.ServoMode do
  @moduledoc """
  A servo can't be driven in the operating mode it was configured with.

  Raised at startup, once the servo on the bus has identified itself. The
  `reason` says which way it failed:

  - `:unknown_model` - the model number isn't in `BB.Servo.Robotis.Model`, so
    only `:position` can be assumed to work
  - `:unsupported_mode` - the model doesn't implement the mode at all; an XL430
    has no current control
  - `:no_torque_constant` - the mode needs to turn newton metres into amps, and
    there's no published torque constant for that model

  The actuator refuses to start rather than falling back to something the robot
  author didn't ask for.
  """
  use BB.Error,
    class: :invalid,
    fields: [:servo_id, :model, :model_number, :mode, :reason, :supported]

  @type reason :: :no_torque_constant | :unknown_model | :unsupported_mode

  @type t :: %__MODULE__{
          servo_id: non_neg_integer(),
          model: String.t() | nil,
          model_number: integer(),
          mode: atom(),
          reason: reason(),
          supported: [atom()]
        }

  defimpl BB.Error.Severity do
    def severity(_), do: :error
  end

  def message(%{reason: :unknown_model} = error) do
    "Servo #{error.servo_id} reports an unrecognised model number " <>
      "(#{error.model_number}), so it can only be driven in :position mode, not " <>
      "#{inspect(error.mode)}. See BB.Servo.Robotis.Model."
  end

  def message(%{reason: :no_torque_constant} = error) do
    "Servo #{error.servo_id} is a #{error.model}, which has no published torque constant, " <>
      "so effort in newton metres can't be converted to current for " <>
      "#{inspect(error.mode)} mode."
  end

  def message(%{reason: :unsupported_mode} = error) do
    "Servo #{error.servo_id} is a #{error.model}, which doesn't support " <>
      "#{inspect(error.mode)} mode. It supports: #{inspect(error.supported)}."
  end
end
