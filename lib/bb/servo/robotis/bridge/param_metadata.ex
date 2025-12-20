# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Servo.Robotis.Bridge.ParamMetadata do
  @moduledoc """
  Metadata for Robotis servo control table parameters.

  Categorises parameters and determines access permissions for the parameter bridge.
  Status parameters (temperature, voltage, current, etc.) are excluded as they are
  exposed via sensor messages instead.

  ## Categories

  - `:info` - Read-only identification (model_number, firmware_version)
  - `:config` - EEPROM settings, require torque off to write (limits, operating_mode)
  - `:control` - RAM settings, writable at runtime (PID gains, profiles)
  """

  @type category :: :info | :config | :control
  @type control_table :: :xm430 | :xl330_m288 | :xl320

  @type param_info :: %{
          category: category(),
          writable: boolean(),
          requires_torque_off: boolean(),
          doc: String.t()
        }

  # Info parameters - read-only identification
  @info_params %{
    model_number: "Servo model number",
    model_information: "Extended model information",
    firmware_version: "Firmware version"
  }

  # Config parameters - EEPROM, require torque off to write
  @config_params %{
    id: "Servo ID (1-253)",
    baud_rate: "Communication baud rate",
    return_delay_time: "Response delay time",
    drive_mode: "Drive mode (velocity/time based, direction)",
    operating_mode: "Operating mode (position/velocity/current/PWM)",
    secondary_id: "Secondary ID for group control",
    protocol_type: "Communication protocol",
    homing_offset: "Position offset from home",
    moving_threshold: "Threshold for moving status",
    temperature_limit: "Maximum temperature limit",
    max_voltage_limit: "Maximum input voltage limit",
    min_voltage_limit: "Minimum input voltage limit",
    pwm_limit: "Maximum PWM limit",
    current_limit: "Maximum current limit",
    velocity_limit: "Maximum velocity limit",
    max_position_limit: "Maximum position limit",
    min_position_limit: "Minimum position limit",
    max_angle_limit: "Maximum angle limit (alias)",
    min_angle_limit: "Minimum angle limit (alias)",
    startup_configuration: "Startup configuration flags",
    pwm_slope: "PWM slope for smooth control",
    shutdown: "Shutdown error conditions"
  }

  # XL320-specific config params
  @xl320_config_params %{
    cw_angle_limit: "Clockwise angle limit",
    ccw_angle_limit: "Counter-clockwise angle limit",
    control_mode: "Control mode (wheel/joint)",
    max_torque: "Maximum torque"
  }

  # Control parameters - RAM, writable at runtime
  @control_params %{
    torque_enable: "Enable/disable torque",
    led: "LED on/off",
    status_return_level: "Status packet return level",
    velocity_i_gain: "Velocity I gain",
    velocity_p_gain: "Velocity P gain",
    position_d_gain: "Position D gain",
    position_i_gain: "Position I gain",
    position_p_gain: "Position P gain",
    feedforward_2nd_gain: "Feedforward 2nd gain",
    feedforward_1st_gain: "Feedforward 1st gain",
    bus_watchdog: "Bus watchdog timeout",
    goal_pwm: "Goal PWM",
    goal_current: "Goal current",
    goal_velocity: "Goal velocity",
    profile_acceleration: "Profile acceleration",
    profile_velocity: "Profile velocity",
    goal_position: "Goal position"
  }

  # XL320-specific control params
  @xl320_control_params %{
    d_gain: "D gain",
    i_gain: "I gain",
    p_gain: "P gain",
    moving_speed: "Moving speed",
    torque_limit: "Torque limit",
    punch: "Minimum current threshold"
  }

  # Status parameters - excluded from bridge, exposed via sensor messages
  @status_params [
    :registered_instruction,
    :hardware_error_status,
    :realtime_tick,
    :moving,
    :moving_status,
    :present_pwm,
    :present_current,
    :present_velocity,
    :present_position,
    :velocity_trajectory,
    :position_trajectory,
    :present_input_voltage,
    :present_temperature,
    :backup_ready,
    :present_voltage,
    :present_speed,
    :present_load
  ]

  # Indirect addressing - excluded (advanced feature)
  @indirect_prefix "indirect_"

  @doc """
  List all parameter names for a control table, excluding status and indirect params.
  """
  @spec list_params(control_table()) :: [atom()]
  def list_params(control_table) do
    all_params =
      Map.keys(@info_params) ++ config_params(control_table) ++ control_params(control_table)

    Enum.sort(all_params)
  end

  @doc """
  Get metadata for a specific parameter.
  """
  @spec param_info(control_table(), atom()) :: {:ok, param_info()} | {:error, :unknown_param}
  def param_info(control_table, param_name) do
    cond do
      Map.has_key?(@info_params, param_name) ->
        {:ok,
         %{
           category: :info,
           writable: false,
           requires_torque_off: false,
           doc: @info_params[param_name]
         }}

      param_name in config_params(control_table) ->
        doc = config_doc(control_table, param_name)

        {:ok,
         %{
           category: :config,
           writable: true,
           requires_torque_off: true,
           doc: doc
         }}

      param_name in control_params(control_table) ->
        doc = control_doc(control_table, param_name)

        {:ok,
         %{
           category: :control,
           writable: true,
           requires_torque_off: false,
           doc: doc
         }}

      true ->
        {:error, :unknown_param}
    end
  end

  @doc """
  Check if a parameter is writable.
  """
  @spec writable?(control_table(), atom()) :: boolean()
  def writable?(control_table, param_name) do
    case param_info(control_table, param_name) do
      {:ok, %{writable: writable}} -> writable
      {:error, _} -> false
    end
  end

  @doc """
  Check if a parameter requires torque to be disabled before writing.
  """
  @spec requires_torque_off?(control_table(), atom()) :: boolean()
  def requires_torque_off?(control_table, param_name) do
    case param_info(control_table, param_name) do
      {:ok, %{requires_torque_off: requires}} -> requires
      {:error, _} -> false
    end
  end

  @doc """
  Check if a parameter is a status parameter (excluded from bridge).
  """
  @spec status_param?(atom()) :: boolean()
  def status_param?(param_name) do
    param_name in @status_params or
      String.starts_with?(Atom.to_string(param_name), @indirect_prefix)
  end

  # Private helpers

  defp config_params(:xl320), do: Map.keys(@config_params) ++ Map.keys(@xl320_config_params)
  defp config_params(_), do: Map.keys(@config_params)

  defp control_params(:xl320), do: Map.keys(@control_params) ++ Map.keys(@xl320_control_params)
  defp control_params(_), do: Map.keys(@control_params)

  defp config_doc(:xl320, param_name) do
    Map.get(@xl320_config_params, param_name) || Map.get(@config_params, param_name, "")
  end

  defp config_doc(_, param_name), do: Map.get(@config_params, param_name, "")

  defp control_doc(:xl320, param_name) do
    Map.get(@xl320_control_params, param_name) || Map.get(@control_params, param_name, "")
  end

  defp control_doc(_, param_name), do: Map.get(@control_params, param_name, "")
end
