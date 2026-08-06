# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Servo.Robotis.Model do
  @moduledoc """
  What each Dynamixel model can do, keyed on the `model_number` it reports.

  Two things can't be read off the bus and can't be inferred from the control
  table: which operating modes a servo implements, and how much torque an amp of
  current buys. Both are per-model, and a single bus can mix models — an XM430
  and an XL430 answer to the same control table but only one of them can do
  current control.

  ## Torque constants

  Robotis publishes a torque constant in the specification table for each model,
  at each of the three supply voltages. The figure here is the one at the
  recommended voltage. It moves with supply: a few percent across the XM430
  range, closer to a fifth across the XL330 range, so treat effort in newton
  metres as approximate rather than calibrated.

  ## Coverage

  Only the models below are known. An unrecognised servo can still be driven in
  position mode — that needs nothing from this table — but anything else is
  refused rather than guessed at. If you have a model that isn't here, its
  numbers are in the specification table on
  [Robotis' e-manual](https://emanual.robotis.com/docs/en/dxl/x/).
  """

  @type t :: %{
          name: String.t(),
          modes: [mode()],
          torque_constant: float() | nil
        }

  @type mode :: :current | :current_position | :extended_position | :position | :velocity

  @x_series_modes [:current, :current_position, :extended_position, :position, :velocity]

  @models %{
    1020 => %{name: "XM430-W350", modes: @x_series_modes, torque_constant: 1.783},
    1030 => %{name: "XM430-W210", modes: @x_series_modes, torque_constant: 1.304},
    1060 => %{
      name: "XL430-W250",
      modes: [:extended_position, :position, :velocity],
      torque_constant: nil
    },
    1190 => %{name: "XL330-M077", modes: @x_series_modes, torque_constant: 0.146},
    1200 => %{name: "XL330-M288", modes: @x_series_modes, torque_constant: 0.354}
  }

  @doc """
  Look up a servo by the `model_number` it reports.

  Returns `:error` for a model this table doesn't know about.
  """
  @spec fetch(integer()) :: {:ok, t()} | :error
  def fetch(model_number), do: Map.fetch(@models, model_number)

  @doc """
  Every operating mode this driver can configure, across all known models.
  """
  @spec modes() :: [mode()]
  def modes, do: @x_series_modes

  @doc """
  The `operating_mode` register value for a mode.

  These are the values in the X-series control table at address 11.
  """
  @spec operating_mode(mode()) :: atom()
  def operating_mode(:current), do: :current_control
  def operating_mode(:current_position), do: :current_based_position_control
  def operating_mode(:extended_position), do: :extended_position_control
  def operating_mode(:position), do: :position_control
  def operating_mode(:velocity), do: :velocity_control
end
