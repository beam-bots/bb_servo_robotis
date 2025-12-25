# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Servo.Robotis.Controller do
  @moduledoc """
  A controller that manages a Robotis/Dynamixel servo bus.

  This controller wraps the `Robotis` GenServer and provides an interface
  for actuators to communicate with servos via the serial bus. Multiple
  actuators can share a single controller, with each actuator controlling
  a different servo ID (1-253).

  ## Position Feedback

  The controller handles position feedback for all registered servos using
  efficient bulk reads (`fast_sync_read`). When actuators register with the
  controller, they provide their joint mapping information. The controller
  then polls all registered servos at a configurable interval and publishes
  `JointState` messages.

  This eliminates the need for separate sensor GenServers and prevents
  bus contention from multiple concurrent read requests.

  ## Configuration

  The controller is typically defined in the robot DSL:

      controller :dynamixel, {BB.Servo.Robotis.Controller,
        port: "/dev/ttyUSB0",
        baud_rate: 1_000_000,
        control_table: Robotis.ControlTable.XM430,
        poll_interval_ms: 50
      }

  ## Options

  - `:port` - (required) The serial port path, e.g., `"/dev/ttyUSB0"`
  - `:baud_rate` - Baud rate in bps (default: 57600)
  - `:control_table` - The servo control table to use (default: `Robotis.ControlTable.XM430`)
  - `:poll_interval_ms` - Position feedback interval in ms (default: 50, i.e. 20Hz)
  - `:status_poll_interval_ms` - Status polling interval in ms (default: 1000, set to 0 to disable)
  - `:disarm_action` - Action to take when robot is disarmed (default: `:disable_torque`)
    - `:disable_torque` - Disable torque on all servos (safe default)
    - `:hold` - Keep torque enabled (servos hold position)

  ## Status Polling

  When enabled, the controller periodically reads status registers (temperature, voltage,
  current, hardware errors) from all registered servos and publishes `ServoStatus` messages
  to the sensor path.

  ## Safety

  This controller implements the `BB.Safety` behaviour. When the robot is disarmed
  or crashes, torque is disabled on all known servo IDs using sync_write for speed.
  """
  use BB.Controller,
    options_schema: [
      port: [
        type: :string,
        doc: "The serial port path (e.g., \"/dev/ttyUSB0\")",
        required: true
      ],
      baud_rate: [
        type: :pos_integer,
        doc: "Baud rate in bps",
        default: 1_000_000
      ],
      control_table: [
        type: {:behaviour, Robotis.ControlTable},
        doc: "The servo control table to use",
        default: Robotis.ControlTable.XM430
      ],
      poll_interval_ms: [
        type: :pos_integer,
        doc: "Position feedback polling interval in milliseconds",
        default: 50
      ],
      status_poll_interval_ms: [
        type: :non_neg_integer,
        doc: "Status polling interval in milliseconds (0 to disable)",
        default: 1000
      ],
      disarm_action: [
        type: {:in, [:disable_torque, :hold]},
        doc: "Action to take when robot is disarmed",
        default: :disable_torque
      ]
    ]

  require Logger

  alias BB.Message
  alias BB.Message.Sensor.JointState
  alias BB.Servo.Robotis.Message.ServoStatus
  alias BB.StateMachine.Transition

  @position_resolution 4096

  @doc """
  Handle disarm based on the configured `disarm_action`.

  Called by `BB.Safety.Controller` when the robot is disarmed or crashes.
  By default, disables torque on all registered servo IDs.
  """
  @impl BB.Controller
  def disarm(opts) do
    disarm_action = Keyword.get(opts, :disarm_action, :disable_torque)
    do_disarm(disarm_action, opts)
  end

  defp do_disarm(:hold, _opts), do: :ok

  defp do_disarm(:disable_torque, opts) do
    robotis = Keyword.fetch!(opts, :robotis)
    servo_ids = Keyword.get(opts, :servo_ids, [])

    try do
      if servo_ids != [] do
        values = Enum.map(servo_ids, fn id -> {id, false} end)
        Robotis.sync_write(robotis, :torque_enable, values)
      end

      :ok
    catch
      :exit, _ -> :ok
    end
  end

  @impl BB.Controller
  def init(opts) do
    bb = Keyword.fetch!(opts, :bb)
    control_table = Keyword.get(opts, :control_table, Robotis.ControlTable.XM430)
    poll_interval_ms = Keyword.get(opts, :poll_interval_ms, 50)
    status_poll_interval_ms = Keyword.get(opts, :status_poll_interval_ms, 1000)
    disarm_action = Keyword.get(opts, :disarm_action, :disable_torque)

    case start_robotis(opts) do
      {:ok, robotis} ->
        state = %{
          bb: bb,
          robotis: robotis,
          control_table: control_table,
          name: List.last(bb.path),
          poll_interval_ms: poll_interval_ms,
          status_poll_interval_ms: status_poll_interval_ms,
          disarm_action: disarm_action,
          # servo_id -> %{joint_name: atom, center_angle: float, last_position_raw: integer | nil}
          servo_registry: %{},
          # servo_id -> %{temperature: float, voltage: float, current: float, hardware_error: integer | nil}
          last_status: %{}
        }

        BB.Safety.register(__MODULE__,
          robot: state.bb.robot,
          path: state.bb.path,
          opts: [robotis: state.robotis, servo_ids: [], disarm_action: state.disarm_action]
        )

        # Subscribe to state machine transitions for arm/disarm
        BB.subscribe(state.bb.robot, [:state_machine])

        # Start polling after a short delay to let actuators register
        Process.send_after(self(), :start_polling, 100)

        {:ok, state}

      {:error, reason} ->
        {:stop, reason}
    end
  end

  defp start_robotis(opts) do
    Robotis.start_link(
      uart_port: Keyword.fetch!(opts, :port),
      baud: Keyword.get(opts, :baud_rate, 1_000_000),
      control_table: Keyword.get(opts, :control_table, Robotis.ControlTable.XM430)
    )
  end

  @impl BB.Controller
  def handle_call(
        {:register_servo, servo_id, joint_name, center_angle, position_deadband, reverse?},
        _from,
        state
      ) do
    new_registry =
      Map.put(state.servo_registry, servo_id, %{
        joint_name: joint_name,
        center_angle: center_angle,
        position_deadband: position_deadband,
        reverse?: reverse?,
        last_position_raw: nil
      })

    # Update safety registration with new servo list
    BB.Safety.register(__MODULE__,
      robot: state.bb.robot,
      path: state.bb.path,
      opts: [
        robotis: state.robotis,
        servo_ids: Map.keys(new_registry),
        disarm_action: state.disarm_action
      ]
    )

    {:reply, :ok, %{state | servo_registry: new_registry}}
  end

  def handle_call({:read, servo_id, param}, _from, state) do
    result = Robotis.read(state.robotis, servo_id, param)
    {:reply, result, state}
  end

  def handle_call({:read_raw, servo_id, param}, _from, state) do
    result = Robotis.read_raw(state.robotis, servo_id, param)
    {:reply, result, state}
  end

  def handle_call({:write, servo_id, param, value}, _from, state) do
    result = Robotis.write(state.robotis, servo_id, param, value, true)
    {:reply, result, state}
  end

  def handle_call({:write_raw, servo_id, param, value}, _from, state) do
    result = Robotis.write_raw(state.robotis, servo_id, param, value, true)
    {:reply, result, state}
  end

  def handle_call({:ping, servo_id}, _from, state) do
    result = Robotis.ping(state.robotis, servo_id)
    {:reply, result, state}
  end

  def handle_call(:ping_all, _from, state) do
    result = Robotis.ping(state.robotis)
    {:reply, result, state}
  end

  def handle_call({:fast_sync_read, servo_ids, param}, _from, state) do
    result = Robotis.fast_sync_read(state.robotis, servo_ids, param)
    {:reply, result, state}
  end

  def handle_call(:list_servos, _from, state) do
    {:reply, {:ok, Map.keys(state.servo_registry)}, state}
  end

  def handle_call(:get_control_table, _from, state) do
    {:reply, {:ok, state.control_table}, state}
  end

  @impl BB.Controller
  def handle_cast({:write, servo_id, param, value}, state) do
    Robotis.write(state.robotis, servo_id, param, value, false)
    {:noreply, state}
  end

  def handle_cast({:write_raw, servo_id, param, value}, state) do
    Robotis.write_raw(state.robotis, servo_id, param, value, false)
    {:noreply, state}
  end

  def handle_cast({:sync_write, param, values}, state) do
    Robotis.sync_write(state.robotis, param, values)
    {:noreply, state}
  end

  @impl BB.Controller
  def handle_info(:start_polling, state) do
    schedule_poll(state.poll_interval_ms)
    schedule_status_poll(state.status_poll_interval_ms)
    {:noreply, state}
  end

  def handle_info(:poll, state) do
    state = poll_positions(state)
    schedule_poll(state.poll_interval_ms)
    {:noreply, state}
  end

  def handle_info(:poll_status, state) do
    state = poll_status(state)
    schedule_status_poll(state.status_poll_interval_ms)
    {:noreply, state}
  end

  def handle_info({:bb, [:state_machine], %Message{payload: %Transition{to: :armed}}}, state) do
    enable_all_torque(state)
    {:noreply, state}
  end

  def handle_info({:bb, [:state_machine], %Message{payload: %Transition{}}}, state) do
    {:noreply, state}
  end

  defp schedule_poll(interval_ms) do
    Process.send_after(self(), :poll, interval_ms)
  end

  defp schedule_status_poll(0), do: :ok

  defp schedule_status_poll(interval_ms) do
    Process.send_after(self(), :poll_status, interval_ms)
  end

  defp enable_all_torque(%{servo_registry: registry}) when map_size(registry) == 0 do
    :ok
  end

  defp enable_all_torque(state) do
    servo_ids = Map.keys(state.servo_registry)
    values = Enum.map(servo_ids, fn id -> {id, true} end)
    Robotis.sync_write(state.robotis, :torque_enable, values)
  end

  defp poll_positions(%{servo_registry: registry} = state) when map_size(registry) == 0 do
    state
  end

  defp poll_positions(state) do
    servo_ids = Map.keys(state.servo_registry)
    results = Robotis.fast_sync_read(state.robotis, servo_ids, :present_position)
    new_registry = publish_changed_positions(results, state)
    %{state | servo_registry: new_registry}
  end

  defp publish_changed_positions(results, state) do
    Enum.reduce(results, state.servo_registry, fn
      {servo_id, {:ok, position_degrees}}, registry ->
        maybe_publish_position(state, registry, servo_id, position_degrees)

      {servo_id, {:error, reason}}, registry ->
        Logger.warning("Failed to read position for servo #{servo_id}: #{inspect(reason)}")
        registry

      other, registry ->
        Logger.warning("Unexpected fast_sync_read result: #{inspect(other)}")
        registry
    end)
  end

  defp maybe_publish_position(_state, registry, servo_id, _position_degrees)
       when not is_map_key(registry, servo_id) do
    registry
  end

  defp maybe_publish_position(state, registry, servo_id, position_degrees) do
    entry = Map.fetch!(registry, servo_id)
    maybe_publish_position_for_entry(state, registry, servo_id, position_degrees, entry)
  end

  defp maybe_publish_position_for_entry(
         state,
         registry,
         servo_id,
         position_degrees,
         %{last_position_raw: nil} = entry
       ) do
    publish_and_update(state, registry, servo_id, position_degrees, entry)
  end

  defp maybe_publish_position_for_entry(
         state,
         registry,
         servo_id,
         position_degrees,
         %{last_position_raw: last, position_deadband: deadband} = entry
       ) do
    if position_exceeds_deadband?(position_degrees, last, deadband) do
      publish_and_update(state, registry, servo_id, position_degrees, entry)
    else
      registry
    end
  end

  defp position_exceeds_deadband?(position_degrees, last, deadband) do
    deadband_degrees = deadband * 360.0 / @position_resolution
    abs(position_degrees - last) >= deadband_degrees
  end

  defp publish_and_update(state, registry, servo_id, position_degrees, entry) do
    position_rad = degrees_to_joint_angle(position_degrees, entry.center_angle, entry.reverse?)
    publish_joint_state(state, entry.joint_name, position_rad)
    Map.put(registry, servo_id, %{entry | last_position_raw: position_degrees})
  end

  # Robotis library returns position in degrees (0-360), not raw units (0-4095)
  # Servo center (180°) corresponds to joint center_angle
  defp degrees_to_joint_angle(position_degrees, center_angle, reverse?) do
    servo_offset_deg = position_degrees - 180.0

    servo_offset_rad =
      if reverse? do
        -servo_offset_deg * :math.pi() / 180.0
      else
        servo_offset_deg * :math.pi() / 180.0
      end

    center_angle + servo_offset_rad
  end

  defp publish_joint_state(state, joint_name, position_rad) do
    {:ok, msg} =
      Message.new(JointState, joint_name,
        names: [joint_name],
        positions: [position_rad]
      )

    # Publish to sensor path so existing subscribers receive it
    BB.publish(state.bb.robot, [:sensor, state.name, joint_name], msg)
  end

  # Status polling - reads temperature, voltage, current, and error status
  defp poll_status(%{servo_registry: registry} = state) when map_size(registry) == 0 do
    state
  end

  defp poll_status(state) do
    servo_ids = Map.keys(state.servo_registry)

    # Read each status parameter from all servos
    temp_results = Robotis.fast_sync_read(state.robotis, servo_ids, :present_temperature)
    voltage_results = Robotis.fast_sync_read(state.robotis, servo_ids, :present_input_voltage)
    current_results = Robotis.fast_sync_read(state.robotis, servo_ids, :present_current)
    error_results = Robotis.fast_sync_read(state.robotis, servo_ids, :hardware_error_status)

    # Combine results per servo and publish if changed
    new_last_status =
      Enum.reduce(servo_ids, state.last_status, fn servo_id, acc ->
        status =
          build_status(servo_id, temp_results, voltage_results, current_results, error_results)

        last = Map.get(acc, servo_id)

        if status_changed?(status, last) do
          publish_servo_status(state, servo_id, status)
          maybe_report_hardware_error(state, servo_id, status, last)
          Map.put(acc, servo_id, status)
        else
          acc
        end
      end)

    %{state | last_status: new_last_status}
  end

  defp build_status(servo_id, temp_results, voltage_results, current_results, error_results) do
    %{
      temperature: get_status_value(temp_results, servo_id),
      voltage: get_status_value(voltage_results, servo_id),
      current: get_status_value(current_results, servo_id),
      hardware_error: get_status_value(error_results, servo_id)
    }
  end

  defp get_status_value(results, servo_id) do
    case Enum.find(results, fn {id, _} -> id == servo_id end) do
      {_, {:ok, value}} -> value
      _ -> nil
    end
  end

  # Deadbands for status values to avoid publishing on noise
  @temp_deadband 1.0
  @voltage_deadband 0.1
  @current_deadband 0.01

  defp status_changed?(_status, nil), do: true

  defp status_changed?(status, last) do
    temp_changed?(status.temperature, last.temperature) or
      voltage_changed?(status.voltage, last.voltage) or
      current_changed?(status.current, last.current) or
      status.hardware_error != last.hardware_error
  end

  defp temp_changed?(nil, _), do: false
  defp temp_changed?(_, nil), do: true
  defp temp_changed?(new, old), do: abs(new - old) >= @temp_deadband

  defp voltage_changed?(nil, _), do: false
  defp voltage_changed?(_, nil), do: true
  defp voltage_changed?(new, old), do: abs(new - old) >= @voltage_deadband

  defp current_changed?(nil, _), do: false
  defp current_changed?(_, nil), do: true
  defp current_changed?(new, old), do: abs(new - old) >= @current_deadband

  defp publish_servo_status(state, servo_id, status) do
    case ServoStatus.new(state.name,
           servo_id: servo_id,
           temperature: status.temperature || 0.0,
           voltage: status.voltage || 0.0,
           current: status.current || 0.0,
           hardware_error: status.hardware_error
         ) do
      {:ok, msg} ->
        BB.publish(state.bb.robot, [:sensor, state.name, :servo_status], msg)

      {:error, reason} ->
        Logger.warning("Failed to create ServoStatus message: #{inspect(reason)}")
    end
  end

  # Report hardware error to safety system when a new error is detected
  defp maybe_report_hardware_error(_state, _servo_id, %{hardware_error: nil}, _last), do: :ok
  defp maybe_report_hardware_error(_state, _servo_id, %{hardware_error: 0}, _last), do: :ok

  defp maybe_report_hardware_error(state, servo_id, %{hardware_error: error}, nil) do
    report_hardware_error(state, servo_id, error)
  end

  defp maybe_report_hardware_error(state, servo_id, %{hardware_error: error}, %{
         hardware_error: last_error
       })
       when error != last_error do
    report_hardware_error(state, servo_id, error)
  end

  defp maybe_report_hardware_error(_state, _servo_id, _status, _last), do: :ok

  defp report_hardware_error(state, servo_id, error) do
    joint_name =
      case Map.get(state.servo_registry, servo_id) do
        %{joint_name: name} -> name
        nil -> String.to_atom("servo_#{servo_id}")
      end

    path = state.bb.path ++ [joint_name]
    BB.Safety.report_error(state.bb.robot, path, {:hardware_error, error})
  end

  @impl BB.Controller
  def terminate(_reason, state) do
    if Process.alive?(state.robotis) do
      GenServer.stop(state.robotis)
    end

    :ok
  end
end
