# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Servo.Robotis.Controller do
  @moduledoc """
  A controller that manages a Robotis/Dynamixel servo bus.

  This controller wraps the `Robotis` GenServer and provides a shared ETS table
  for actuators to write commands to. The controller runs a fixed-rate control
  loop that batches all pending commands into writes and reads positions via
  bulk reads (`fast_sync_read`).

  On each loop tick, the controller:

  1. Reads all pending commands from the ETS table
  2. Writes them to the serial bus
  3. Clears the command fields
  4. Reads all servo positions via `fast_sync_read`
  5. Updates the ETS table with current positions
  6. Publishes `JointState` messages for positions that exceed deadband

  ## Configuration

  The controller is typically defined in the robot DSL:

      controllers do
        controller :dynamixel, {BB.Servo.Robotis.Controller,
          port: "/dev/ttyUSB0",
          baud_rate: 1_000_000,
          control_table: Robotis.ControlTable.XM430,
          loop_interval_ms: 10
        }
      end

  ## Options

  - `:port` - (required) The serial port path, e.g., `"/dev/ttyUSB0"`
  - `:baud_rate` - Baud rate in bps (default: 1_000_000)
  - `:control_table` - The servo control table to use (default: `Robotis.ControlTable.XM430`)
  - `:loop_interval_ms` - Control loop interval in ms (default: 10, i.e. 100Hz)
  - `:status_poll_interval_ms` - Status polling interval in ms (default: 1000, set to 0 to disable)
  - `:disarm_action` - Action to take when robot is disarmed (default: `:disable_torque`)
    - `:disable_torque` - Disable torque on all servos (safe default)
    - `:hold` - Keep torque enabled (servos hold position)

  ## ETS Table Structure

  Each registered servo has a row in the ETS table:

      {servo_id, actuator_path, position_deadband, last_position_raw, present_position,
       present_temperature, present_voltage, present_current, hardware_error,
       goal_position}

  Actuators write `goal_position` (raw units) via `:ets.update_element/3`.
  The controller reads and clears them each tick.

  ## Safety

  This controller implements the `BB.Controller` behaviour's `disarm/1` safety
  callback. When the robot is disarmed or crashes, torque is disabled on all known
  servo IDs using acknowledged per-servo writes, so `disarm/1` only reports `:ok`
  once the bus has confirmed every servo is safe; any failed or undeliverable
  write returns `{:error, reason}` and drives the robot into the `:error` state.
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
      loop_interval_ms: [
        type: :pos_integer,
        doc: "Control loop interval in milliseconds (default: 10, i.e. 100Hz)",
        default: 10
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

  alias BB.Diagnostic
  alias BB.Error.Protocol.Robotis.HardwareAlert
  alias BB.Message
  alias BB.Message.Sensor.JointState
  alias BB.Servo.Robotis.Message.ServoStatus
  alias BB.StateMachine.Transition

  @position_resolution 4096

  # ETS tuple field indices (positions 2-3 are config, matched via pattern)
  @idx_last_position_raw 4
  @idx_present_position 5
  @idx_present_temperature 6
  @idx_present_voltage 7
  @idx_present_current 8
  @idx_hardware_error 9
  @idx_goal_position 10

  # Diagnostic thresholds
  @temp_warning_threshold 55.0
  @temp_error_threshold 70.0
  @voltage_low_threshold 10.0
  @voltage_high_threshold 14.0

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
      disable_torque(robotis, servo_ids)
    catch
      :exit, reason -> {:error, {:exit, reason}}
    end
  end

  defp disable_torque(robotis, servo_ids) do
    Enum.reduce_while(servo_ids, :ok, fn id, :ok ->
      case Robotis.write(robotis, id, :torque_enable, false, true) do
        :ok -> {:cont, :ok}
        {:error, reason} -> {:halt, {:error, {:servo, id, :torque_enable, reason}}}
      end
    end)
  end

  @impl BB.Controller
  def init(opts) do
    bb = Keyword.fetch!(opts, :bb)
    control_table = Keyword.get(opts, :control_table, Robotis.ControlTable.XM430)
    loop_interval_ms = Keyword.get(opts, :loop_interval_ms, 10)
    status_poll_interval_ms = Keyword.get(opts, :status_poll_interval_ms, 1000)
    disarm_action = Keyword.get(opts, :disarm_action, :disable_torque)

    status_every_n_ticks =
      if status_poll_interval_ms > 0 do
        max(1, div(status_poll_interval_ms, loop_interval_ms))
      else
        0
      end

    case start_robotis(opts) do
      {:ok, robotis} ->
        servo_table = :ets.new(:servo_state, [:set, :public])

        state = %{
          bb: bb,
          robotis: robotis,
          control_table: control_table,
          name: List.last(bb.path),
          loop: BB.Loop.new(bb, clock: {:rate, 1000 / loop_interval_ms}),
          status_poll_interval_ms: status_poll_interval_ms,
          status_every_n_ticks: status_every_n_ticks,
          status_tick_counter: 0,
          disarm_action: disarm_action,
          servo_table: servo_table,
          servo_ids: [],
          last_status: %{}
        }

        BB.Safety.register(__MODULE__,
          robot: state.bb.robot,
          path: state.bb.path,
          opts: [robotis: state.robotis, servo_ids: [], disarm_action: state.disarm_action]
        )

        BB.subscribe(state.bb.robot, [:state_machine])

        Process.send_after(self(), :start_loop, 100)

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

  # --- Handle calls ---

  @impl BB.Controller
  def handle_call(
        {:register_servo, servo_id, actuator_path, position_deadband},
        _from,
        state
      ) do
    :ets.insert(state.servo_table, {
      servo_id,
      actuator_path,
      position_deadband,
      _last_position_raw = nil,
      _present_position = nil,
      _present_temperature = nil,
      _present_voltage = nil,
      _present_current = nil,
      _hardware_error = nil,
      _goal_position = nil
    })

    servo_ids = [servo_id | state.servo_ids] |> Enum.sort() |> Enum.uniq()

    BB.Safety.register(__MODULE__,
      robot: state.bb.robot,
      path: state.bb.path,
      opts: [
        robotis: state.robotis,
        servo_ids: servo_ids,
        disarm_action: state.disarm_action
      ]
    )

    {:reply, {:ok, state.servo_table}, %{state | servo_ids: servo_ids}}
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
    {:reply, {:ok, state.servo_ids}, state}
  end

  def handle_call(:get_control_table, _from, state) do
    {:reply, {:ok, state.control_table}, state}
  end

  # --- Handle info ---

  @impl BB.Controller
  def handle_info(:start_loop, state) do
    {:noreply, %{state | loop: BB.Loop.arm(state.loop)}}
  end

  def handle_info(:tick, state) do
    {_dt, _skipped, loop} = BB.Loop.tick(state.loop)

    state =
      %{state | loop: loop}
      |> process_commands()
      |> read_positions()
      |> maybe_poll_status()

    {:noreply, state}
  end

  def handle_info({:bb, [:state_machine], %Message{payload: %Transition{to: :armed}}}, state) do
    enable_all_torque(state)
    {:noreply, state}
  end

  def handle_info({:bb, [:state_machine], %Message{payload: %Transition{}}}, state) do
    {:noreply, state}
  end

  # --- Control loop ---

  defp process_commands(%{servo_ids: []} = state), do: state

  defp process_commands(state) do
    entries = :ets.tab2list(state.servo_table)

    commands =
      for {id, _, _, _, _, _, _, _, _, goal_pos} <- entries,
          goal_pos != nil,
          do: {id, goal_pos}

    if commands != [] do
      Enum.each(commands, fn {id, goal_pos} ->
        Robotis.write_raw(state.robotis, id, :goal_position, goal_pos, false)
      end)

      for {id, _} <- commands do
        :ets.update_element(state.servo_table, id, [{@idx_goal_position, nil}])
      end
    end

    state
  end

  defp read_positions(%{servo_ids: []} = state), do: state

  defp read_positions(state) do
    results = Robotis.fast_sync_read(state.robotis, state.servo_ids, :present_position)

    Enum.each(results, fn
      {servo_id, {:ok, position_degrees}} ->
        :ets.update_element(
          state.servo_table,
          servo_id,
          [{@idx_present_position, position_degrees}]
        )

        maybe_publish_position(state, servo_id, position_degrees)

      {servo_id, {:error, reason}} ->
        Logger.warning("Failed to read position for servo #{servo_id}: #{inspect(reason)}")
        emit_comm_error_diagnostic(state, servo_id, reason)

      other ->
        Logger.warning("Unexpected fast_sync_read result: #{inspect(other)}")
    end)

    state
  end

  defp maybe_poll_status(%{status_every_n_ticks: 0} = state), do: state

  defp maybe_poll_status(state) do
    counter = state.status_tick_counter + 1

    if counter >= state.status_every_n_ticks do
      poll_status(%{state | status_tick_counter: 0})
    else
      %{state | status_tick_counter: counter}
    end
  end

  # --- Position publishing ---

  defp maybe_publish_position(state, servo_id, position_degrees) do
    case :ets.lookup(state.servo_table, servo_id) do
      [
        {^servo_id, actuator_path, position_deadband, last_position_raw, _, _, _, _, _, _}
      ] ->
        if should_publish_position?(position_degrees, last_position_raw, position_deadband) do
          # Robotis returns position in degrees (0-360); centre at 180° corresponds
          # to motor zero. Convert to motor-space radians and let the transmission
          # handle the rest on the way out.
          motor_rad = (position_degrees - 180.0) * :math.pi() / 180.0
          publish_joint_state(state, actuator_path, motor_rad)

          :ets.update_element(state.servo_table, servo_id, [
            {@idx_last_position_raw, position_degrees}
          ])
        end

      [] ->
        :ok
    end
  end

  defp should_publish_position?(_position_degrees, nil, _deadband), do: true

  defp should_publish_position?(position_degrees, last, deadband) do
    position_exceeds_deadband?(position_degrees, last, deadband)
  end

  defp emit_comm_error_diagnostic(state, servo_id, reason) do
    joint_name = get_joint_name(state, servo_id)
    component = [state.bb.robot | state.bb.path] ++ [joint_name]

    Diagnostic.error(component, "Communication error reading position",
      values: %{servo_id: servo_id, reason: inspect(reason)}
    )
  end

  defp position_exceeds_deadband?(position_degrees, last, deadband) do
    deadband_degrees = deadband * 360.0 / @position_resolution
    abs(position_degrees - last) >= deadband_degrees
  end

  defp publish_joint_state(state, actuator_path, motor_position) do
    joint_name = joint_name_from_path(actuator_path)

    {:ok, motor_msg} =
      Message.new(JointState, joint_name,
        names: [joint_name],
        positions: [motor_position]
      )

    joint_msg = BB.Actuator.to_joint_space(state.bb.robot, actuator_path, motor_msg)
    BB.publish(state.bb.robot, [:sensor, state.name, joint_name], joint_msg)
  end

  defp joint_name_from_path(actuator_path) do
    actuator_path |> Enum.reverse() |> Enum.at(1)
  end

  # --- Arming ---

  defp enable_all_torque(%{servo_ids: []}), do: :ok

  defp enable_all_torque(state) do
    servo_ids = state.servo_ids

    # Read current positions so we can set goals before enabling torque
    results = Robotis.fast_sync_read(state.robotis, servo_ids, :present_position)

    Enum.each(results, fn
      {id, {:ok, _position}} ->
        case Robotis.read_raw(state.robotis, id, :present_position) do
          {:ok, raw_pos} ->
            Robotis.write_raw(state.robotis, id, :goal_position, raw_pos, false)

          {:error, reason} ->
            Logger.warning("Servo #{id} read position failed before arming: #{inspect(reason)}")
        end

      {id, {:error, reason}} ->
        Logger.warning("Servo #{id} sync_read failed before arming: #{inspect(reason)}")
    end)

    values = Enum.map(servo_ids, fn id -> {id, true} end)
    Robotis.sync_write(state.robotis, :torque_enable, values)
  end

  # --- Status polling ---

  defp poll_status(%{servo_ids: []} = state), do: state

  defp poll_status(state) do
    servo_ids = state.servo_ids

    temp_results = Robotis.fast_sync_read(state.robotis, servo_ids, :present_temperature)
    voltage_results = Robotis.fast_sync_read(state.robotis, servo_ids, :present_input_voltage)
    current_results = Robotis.fast_sync_read(state.robotis, servo_ids, :present_current)
    error_results = Robotis.fast_sync_read(state.robotis, servo_ids, :hardware_error_status)

    new_last_status =
      Enum.reduce(servo_ids, state.last_status, fn servo_id, acc ->
        status =
          build_status(servo_id, temp_results, voltage_results, current_results, error_results)

        write_servo_status(state.servo_table, servo_id, status)

        last = Map.get(acc, servo_id)

        if status_changed?(status, last) do
          publish_servo_status(state, servo_id, status)
          maybe_report_hardware_error(state, servo_id, status, last)
          emit_status_diagnostics(state, servo_id, status, last)
          Map.put(acc, servo_id, status)
        else
          acc
        end
      end)

    %{state | last_status: new_last_status}
  end

  defp write_servo_status(table, servo_id, status) do
    :ets.update_element(table, servo_id, [
      {@idx_present_temperature, status.temperature},
      {@idx_present_voltage, status.voltage},
      {@idx_present_current, status.current},
      {@idx_hardware_error, status.hardware_error}
    ])
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

  # --- Diagnostics ---

  defp get_joint_name(state, servo_id) do
    case :ets.lookup(state.servo_table, servo_id) do
      [{^servo_id, actuator_path, _, _, _, _, _, _, _, _}] -> joint_name_from_path(actuator_path)
      [] -> String.to_atom("servo_#{servo_id}")
    end
  end

  defp emit_status_diagnostics(state, servo_id, status, last) do
    emit_temperature_diagnostic(state, servo_id, status.temperature, last)
    emit_voltage_diagnostic(state, servo_id, status.voltage, last)
  end

  defp emit_temperature_diagnostic(_state, _servo_id, nil, _last), do: :ok

  defp emit_temperature_diagnostic(state, servo_id, temp, last) do
    last_temp = if last, do: last.temperature, else: nil
    temp_level = temp_diagnostic_level(temp, last_temp)
    emit_temp_diagnostic_for_level(temp_level, state, servo_id, temp)
  end

  defp temp_diagnostic_level(temp, last_temp) do
    crossed_error? = crossed_threshold?(temp, last_temp, @temp_error_threshold, :up)
    crossed_warning? = crossed_threshold?(temp, last_temp, @temp_warning_threshold, :up)
    recovered? = crossed_threshold?(temp, last_temp, @temp_warning_threshold, :down)

    cond do
      crossed_error? -> :error
      crossed_warning? -> :warn
      recovered? -> :ok
      true -> nil
    end
  end

  defp emit_temp_diagnostic_for_level(nil, _state, _servo_id, _temp), do: :ok

  defp emit_temp_diagnostic_for_level(:error, state, servo_id, temp) do
    joint_name = get_joint_name(state, servo_id)
    component = [state.bb.robot | state.bb.path] ++ [joint_name]

    Diagnostic.error(component, "Temperature critical",
      values: %{temperature: temp, threshold: @temp_error_threshold, servo_id: servo_id}
    )
  end

  defp emit_temp_diagnostic_for_level(:warn, state, servo_id, temp) do
    joint_name = get_joint_name(state, servo_id)
    component = [state.bb.robot | state.bb.path] ++ [joint_name]

    Diagnostic.warn(component, "Temperature elevated",
      values: %{temperature: temp, threshold: @temp_warning_threshold, servo_id: servo_id}
    )
  end

  defp emit_temp_diagnostic_for_level(:ok, state, servo_id, temp) do
    joint_name = get_joint_name(state, servo_id)
    component = [state.bb.robot | state.bb.path] ++ [joint_name]

    Diagnostic.ok(component, "Temperature normal",
      values: %{temperature: temp, servo_id: servo_id}
    )
  end

  defp emit_voltage_diagnostic(_state, _servo_id, nil, _last), do: :ok

  defp emit_voltage_diagnostic(state, servo_id, voltage, last) do
    last_voltage = if last, do: last.voltage, else: nil
    voltage_level = voltage_diagnostic_level(voltage, last_voltage)
    emit_voltage_diagnostic_for_level(voltage_level, state, servo_id, voltage)
  end

  defp voltage_diagnostic_level(voltage, last_voltage) do
    crossed_low? = crossed_threshold?(voltage, last_voltage, @voltage_low_threshold, :down)
    crossed_high? = crossed_threshold?(voltage, last_voltage, @voltage_high_threshold, :up)
    recovered? = voltage_recovered?(voltage, last_voltage)

    cond do
      crossed_low? -> :warn_low
      crossed_high? -> :warn_high
      recovered? -> :ok
      true -> nil
    end
  end

  defp voltage_recovered?(_voltage, nil), do: false

  defp voltage_recovered?(voltage, last_voltage) do
    voltage_ok? = voltage >= @voltage_low_threshold and voltage <= @voltage_high_threshold

    last_voltage_ok? =
      last_voltage >= @voltage_low_threshold and last_voltage <= @voltage_high_threshold

    voltage_ok? and not last_voltage_ok?
  end

  defp emit_voltage_diagnostic_for_level(nil, _state, _servo_id, _voltage), do: :ok

  defp emit_voltage_diagnostic_for_level(:warn_low, state, servo_id, voltage) do
    joint_name = get_joint_name(state, servo_id)
    component = [state.bb.robot | state.bb.path] ++ [joint_name]

    Diagnostic.warn(component, "Voltage low",
      values: %{voltage: voltage, threshold: @voltage_low_threshold, servo_id: servo_id}
    )
  end

  defp emit_voltage_diagnostic_for_level(:warn_high, state, servo_id, voltage) do
    joint_name = get_joint_name(state, servo_id)
    component = [state.bb.robot | state.bb.path] ++ [joint_name]

    Diagnostic.warn(component, "Voltage high",
      values: %{voltage: voltage, threshold: @voltage_high_threshold, servo_id: servo_id}
    )
  end

  defp emit_voltage_diagnostic_for_level(:ok, state, servo_id, voltage) do
    joint_name = get_joint_name(state, servo_id)
    component = [state.bb.robot | state.bb.path] ++ [joint_name]

    Diagnostic.ok(component, "Voltage normal", values: %{voltage: voltage, servo_id: servo_id})
  end

  defp crossed_threshold?(value, nil, threshold, :up), do: value >= threshold
  defp crossed_threshold?(value, nil, threshold, :down), do: value < threshold

  defp crossed_threshold?(value, last, threshold, :up),
    do: value >= threshold and last < threshold

  defp crossed_threshold?(value, last, threshold, :down),
    do: value < threshold and last >= threshold

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
    joint_name = get_joint_name(state, servo_id)
    path = state.bb.path ++ [joint_name]
    alert = HardwareAlert.from_bits(servo_id, error)

    BB.Safety.report_error(state.bb.robot, path, alert)

    component = [state.bb.robot | path]

    Diagnostic.error(component, "Hardware error detected",
      values: %{servo_id: servo_id, error_bits: error, alerts: alert.alerts}
    )
  end

  @impl BB.Controller
  def terminate(_reason, state) do
    BB.Loop.cancel(state.loop)

    if Process.alive?(state.robotis) do
      GenServer.stop(state.robotis)
    end

    :ok
  end
end
