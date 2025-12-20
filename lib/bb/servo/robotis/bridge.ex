# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Servo.Robotis.Bridge do
  @moduledoc """
  Parameter bridge for Robotis/Dynamixel servo control table parameters.

  This bridge exposes servo configuration and control parameters through the
  BB parameter system. Status parameters (temperature, voltage, current, errors)
  are exposed via sensor messages instead - see `BB.Servo.Robotis.Message.ServoStatus`.

  ## Parameter ID Format

  Parameters are identified by strings in the format `"servo_id:param_name"`:

  - `"1:position_p_gain"` - Position P gain for servo ID 1
  - `"2:velocity_limit"` - Velocity limit for servo ID 2

  ## Parameter Categories

  - **info** - Read-only identification (model_number, firmware_version)
  - **config** - EEPROM settings, require torque off to write (limits, operating_mode)
  - **control** - RAM settings, writable at runtime (PID gains, profiles)

  ## Usage

      # In robot DSL
      parameters do
        bridge :robotis, {BB.Servo.Robotis.Bridge, controller: :dynamixel}
      end

      # Read parameter
      {:ok, 800} = BB.Parameter.get_remote(MyRobot, :robotis, "1:position_p_gain")

      # Write parameter (control params)
      :ok = BB.Parameter.set_remote(MyRobot, :robotis, "1:position_p_gain", 1000)

      # List all parameters
      {:ok, params} = BB.Parameter.list_remote(MyRobot, :robotis)
  """

  use BB.Bridge,
    options_schema: [
      controller: [
        type: :atom,
        required: true,
        doc: "Name of the Robotis controller to use"
      ]
    ]

  alias BB.Servo.Robotis.Bridge.ParamMetadata

  @impl GenServer
  def init(opts) do
    bb = Keyword.fetch!(opts, :bb)
    controller = Keyword.fetch!(opts, :controller)

    # Query controller for control table
    {:ok, control_table} = BB.Process.call(bb.robot, controller, :get_control_table)

    state = %{
      robot: bb.robot,
      controller: controller,
      control_table: control_table
    }

    {:ok, state}
  end

  @impl BB.Bridge
  def handle_change(_robot, _changed, state) do
    # This bridge is primarily inbound (reading from servos)
    # Outbound changes are not propagated to servos
    {:ok, state}
  end

  @impl BB.Bridge
  def list_remote(state) do
    {:ok, servo_ids} = BB.Process.call(state.robot, state.controller, :list_servos)

    params =
      for servo_id <- servo_ids,
          param_name <- ParamMetadata.list_params(state.control_table),
          {:ok, info} = ParamMetadata.param_info(state.control_table, param_name) do
        %{
          id: "#{servo_id}:#{param_name}",
          value: nil,
          type: nil,
          doc: info.doc,
          path: [:robotis, String.to_atom("servo_#{servo_id}"), param_name],
          writable: info.writable,
          category: info.category,
          requires_torque_off: info.requires_torque_off
        }
      end

    {:ok, params, state}
  end

  @impl BB.Bridge
  def get_remote(param_id, state) do
    with {:ok, servo_id, param_name} <- parse_param_id(param_id),
         :ok <- validate_known_param(state.control_table, param_name),
         {:ok, value} <- read_param(state, servo_id, param_name) do
      {:ok, value, state}
    else
      {:error, reason} -> {:error, reason, state}
    end
  end

  @impl BB.Bridge
  def set_remote(param_id, value, state) do
    with {:ok, servo_id, param_name} <- parse_param_id(param_id),
         :ok <- validate_known_param(state.control_table, param_name),
         :ok <- validate_writable(state.control_table, param_name),
         :ok <- validate_torque_requirement(state, servo_id, param_name),
         result <- write_param(state, servo_id, param_name, value) do
      case result do
        :ok -> {:ok, state}
        {:ok, _} -> {:ok, state}
        {:error, reason} -> {:error, reason, state}
      end
    else
      {:error, reason} -> {:error, reason, state}
    end
  end

  # Parsing

  defp parse_param_id(param_id) when is_binary(param_id) do
    case String.split(param_id, ":", parts: 2) do
      [id_str, param_str] ->
        case Integer.parse(id_str) do
          {servo_id, ""} when servo_id > 0 and servo_id < 254 ->
            {:ok, servo_id, String.to_existing_atom(param_str)}

          _ ->
            {:error, {:invalid_param_id, param_id}}
        end

      _ ->
        {:error, {:invalid_param_id, param_id}}
    end
  rescue
    ArgumentError -> {:error, {:invalid_param_id, param_id}}
  end

  defp parse_param_id(param_id), do: {:error, {:invalid_param_id, param_id}}

  # Validation

  defp validate_known_param(control_table, param_name) do
    case ParamMetadata.param_info(control_table, param_name) do
      {:ok, _} -> :ok
      {:error, :unknown_param} -> {:error, {:unknown_param, param_name}}
    end
  end

  defp validate_writable(control_table, param_name) do
    if ParamMetadata.writable?(control_table, param_name) do
      :ok
    else
      {:error, {:read_only, param_name}}
    end
  end

  defp validate_torque_requirement(state, servo_id, param_name) do
    if ParamMetadata.requires_torque_off?(state.control_table, param_name) do
      case read_param(state, servo_id, :torque_enable) do
        {:ok, false} -> :ok
        {:ok, true} -> {:error, {:torque_must_be_disabled, param_name}}
        {:error, _} -> :ok
      end
    else
      :ok
    end
  end

  # Controller communication

  defp read_param(state, servo_id, param_name) do
    BB.Process.call(state.robot, state.controller, {:read, servo_id, param_name})
  end

  defp write_param(state, servo_id, param_name, value) do
    BB.Process.call(state.robot, state.controller, {:write, servo_id, param_name, value})
  end
end
