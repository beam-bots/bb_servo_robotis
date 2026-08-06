# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Servo.Robotis.Actuator do
  @moduledoc """
  An actuator that uses a Robotis controller to drive a Dynamixel servo.

  Configuration is derived from the joint's `motor_profile` injected by
  `BB.Actuator.Server`:

  - Position limits from `motor_profile.motor_lower` / `motor_upper`
  - Velocity limit from `motor_profile.motor_velocity_limit`
  - Position range maps to the servo's goal_position register

  When initialised, the actuator:
  1. Disables torque on the servo
  2. Asks the servo what model it is, and sets its operating mode if it isn't
     already in the configured one
  3. Registers with the controller, receiving the shared ETS table reference
  4. Writes `profile_velocity` from the joint's velocity limit
  5. Subscribes to the commands its mode admits

  ## Operating modes

  A Dynamixel does one thing at a time, and `:mode` picks which. It's set once at
  startup and never changed while running: switching modes resets the servo's PID
  gains, its profile velocity and acceleration to 0, and its goal current to the
  current limit — including tuning applied through
  `BB.Servo.Robotis.Bridge`, which this driver has no way to put back.

  | `:mode` | commands it accepts |
  | --- | --- |
  | `:position` (default) | `Position`, `Hold`, `Stop` |
  | `:velocity` | `Velocity`, `Hold`, `Stop` |
  | `:current` | `Effort`, `Stop` |
  | `:current_position` | `Position`, `Effort`, `Hold`, `Stop` |

  Anything outside that list is refused by the framework with
  `BB.Error.State.UnsupportedCommand` before it reaches the driver. `Hold` is
  absent from `:current` because a servo in current control is a torque source
  with no position to hold.

  Not every servo implements every mode — an XL430 has no current control at all
  — so the mode is checked against the model the servo reports, and the actuator
  refuses to start rather than run in a mode nobody asked for. See
  `BB.Servo.Robotis.Model`.

  ## Commands

  - `Command.Position` — travel to a position. Clamped to the joint's limits.
  - `Command.Velocity` — turn at a rate, clamped to the joint's velocity limit.
  - `Command.Effort` — in `:current` mode, the torque to aim for; in
    `:current_position` mode, a ceiling on the current a position move may draw.
    Newton metres are converted using the model's published torque constant,
    which is quoted at the recommended supply voltage and drifts with it — treat
    effort as approximate rather than calibrated.
  - `Command.Stop` — cut torque, leaving the joint passive and free to be
    backdriven. Both `:immediate` and `:decelerate` do the same thing, because
    becoming passive is a single register write with no ramp available.
  - `Command.Hold` — stay under power without driving. A servo already doing that
    needs nothing; after a `Stop` it re-applies torque where the joint has come
    to rest.

  A `Position`, `Velocity` or `Effort` command sent to a joint left passive by
  `Stop` re-applies torque on the way past, so callers don't have to pair the two.

  `Velocity` and `Effort` carry a `duration`. When it runs out, `:expiry_action`
  decides whether the joint goes passive (`:stop`, the default) or stays under
  power without driving (`:hold`) — the same choice the controller's
  `:disarm_action` makes for the whole bus.

  Beware that a passive joint under load will move, and `Stop` does not wait for
  it to settle. `Hold` and `Position` both re-apply torque from the servo's own
  present position, so neither snaps back to a pre-`Stop` goal, but a joint that
  has sagged will still be somewhere the caller may not expect.

  None of this is the safety path: making the hardware safe is `disarm/1`, which
  is robot-wide and leaves the robot unable to move until it is armed again.

  ## Example DSL Usage

      controllers do
        controller :dynamixel, {BB.Servo.Robotis.Controller,
          port: "/dev/ttyUSB0",
          baud_rate: 1_000_000
        }
      end

      joint :shoulder, type: :revolute do
        limit lower: ~u(-90 degree), upper: ~u(90 degree), velocity: ~u(60 degree_per_second)

        actuator :servo, {BB.Servo.Robotis.Actuator, servo_id: 1, controller: :dynamixel}
      end
  """
  use BB.Actuator,
    options_schema: [
      servo_id: [
        type: {:in, 1..253},
        doc: "The Dynamixel servo ID (1-253)",
        required: true
      ],
      controller: [
        type: :atom,
        doc: "Name of the Robotis controller in the robot's registry",
        required: true
      ],
      position_deadband: [
        type: :non_neg_integer,
        doc:
          "Minimum position change (raw units) to trigger feedback publish. Filters servo noise.",
        default: 2
      ],
      mode: [
        type: {:in, [:current, :current_position, :position, :velocity]},
        doc: """
        The servo's operating mode, which decides what it can be commanded to do.
        Set once at startup and never changed while running — switching resets
        the servo's PID gains, profile velocity and goal current.
        """,
        default: :position
      ],
      expiry_action: [
        type: {:in, [:hold, :stop]},
        doc: """
        What to do when a velocity or effort command's `duration` runs out:
        `:stop` to go passive, or `:hold` to stay under power without driving.
        A joint that carries a load wants `:hold`; one that should be
        backdrivable when nothing is asking it to move wants `:stop`.
        """,
        default: :stop
      ]
    ]

  alias BB.Error.Invalid.JointConfig, as: JointConfigError
  alias BB.Error.Invalid.Robotis.ServoMode, as: ServoModeError
  alias BB.Message
  alias BB.Message.Actuator.Command
  alias BB.Process, as: BBProcess
  alias BB.Servo.Robotis.Model

  @position_resolution 4096
  @position_center 2048

  # ETS tuple field index for command writes
  @ets_idx_pending_write 10

  @doc """
  Safety disarm callback.

  Returns :ok because torque management is handled by the controller.
  """
  @impl BB.Actuator
  def disarm(_opts), do: :ok

  @impl BB.Actuator
  def init(opts) do
    with {:ok, state} <- build_state(opts),
         :ok <- disable_torque(state),
         {:ok, state} <- fetch_scales(state),
         {:ok, state} <- configure_mode(state),
         {:ok, servo_table} <- register_servo(state),
         state = Map.put(state, :servo_table, servo_table),
         :ok <- write_profile_velocity(state) do
      {:ok, state}
    else
      {:error, reason} -> {:stop, reason}
    end
  end

  @impl BB.Actuator
  def handle_options(new_opts, state) do
    motor_profile = Keyword.fetch!(new_opts, :motor_profile)

    state = %{
      state
      | motor_profile: motor_profile,
        current_motor_angle: clamp_motor_angle(state.current_motor_angle, motor_profile)
    }

    case write_profile_velocity(state) do
      :ok -> {:ok, state}
      {:error, reason} -> {:stop, reason}
    end
  end

  defp build_state(opts) do
    opts = Map.new(opts)
    [name, joint_name | _] = Enum.reverse(opts.bb.path)
    motor_profile = opts.motor_profile
    mode = Map.get(opts, :mode, :position)

    with :ok <- validate_motor_profile(motor_profile, joint_name, mode) do
      state = %{
        bb: opts.bb,
        servo_id: opts.servo_id,
        controller: opts.controller,
        position_deadband: Map.get(opts, :position_deadband, 2),
        mode: mode,
        expiry_action: Map.get(opts, :expiry_action, :stop),
        torque_constant: nil,
        expiry_timer: nil,
        scales: %{},
        motor_profile: motor_profile,
        current_motor_angle: motor_profile.motor_initial_position,
        name: name,
        joint_name: joint_name
      }

      {:ok, state}
    end
  end

  defp validate_motor_profile(%{motor_lower: nil}, joint_name, _mode) do
    {:error,
     %JointConfigError{
       joint: joint_name,
       field: :lower,
       value: nil,
       message: "Joint must have a lower limit defined for servo control"
     }}
  end

  defp validate_motor_profile(%{motor_upper: nil}, joint_name, _mode) do
    {:error,
     %JointConfigError{
       joint: joint_name,
       field: :upper,
       value: nil,
       message: "Joint must have an upper limit defined for servo control"
     }}
  end

  defp validate_motor_profile(%{motor_velocity_limit: nil}, joint_name, _mode) do
    {:error,
     %JointConfigError{
       joint: joint_name,
       field: :velocity,
       value: nil,
       message: "Joint must have a velocity limit defined for servo control"
     }}
  end

  defp validate_motor_profile(%{motor_effort_limit: nil}, joint_name, mode)
       when mode in [:current, :current_position] do
    {:error,
     %JointConfigError{
       joint: joint_name,
       field: :effort,
       value: nil,
       message: "Joint must have an effort limit defined to be driven by current"
     }}
  end

  defp validate_motor_profile(_profile, _joint_name, _mode), do: :ok

  defp disable_torque(state) do
    case BBProcess.call(
           state.bb.robot,
           state.controller,
           {:write, state.servo_id, :torque_enable, false}
         ) do
      :ok -> :ok
      {:ok, _} -> :ok
      {:error, _} = error -> error
    end
  end

  # Goal registers count in model-specific units — 0.229 rev/min of velocity
  # everywhere, but 2.69 mA of current on an XM430 against 1 mA on an XL330. The
  # control table already carries them, so take them from there rather than
  # hardcoding a second copy that only matches some of the servos we support.
  defp fetch_scales(state) do
    case BBProcess.call(state.bb.robot, state.controller, :get_control_table) do
      {:ok, control_table} ->
        scales =
          Map.new([:goal_current, :goal_velocity, :profile_velocity], fn param ->
            {param, scale(control_table, param)}
          end)

        {:ok, %{state | scales: scales}}

      {:error, _} = error ->
        error
    end
  end

  defp scale(control_table, param) do
    case Robotis.ControlTable.info_for_param(control_table, param) do
      {_address, _length, scale} when is_number(scale) -> scale
      _ -> nil
    end
  end

  # Torque is already off at this point, which is what `operating_mode` needs —
  # it lives in EEPROM. Doing it here rather than on demand is deliberate: a mode
  # change resets the servo's PID gains, its profile velocity and acceleration to
  # 0, and its goal current to the current limit. Some of that we'd re-apply
  # below; PID gains set through the parameter bridge we couldn't.
  defp configure_mode(state) do
    with {:ok, model_number} <- read_param(state, :model_number),
         {:ok, torque_constant} <- validate_mode(state, model_number),
         :ok <- write_operating_mode(state) do
      {:ok, %{state | torque_constant: torque_constant}}
    end
  end

  defp validate_mode(state, model_number) do
    case Model.fetch(model_number) do
      {:ok, model} -> validate_model_mode(state, model, model_number)
      :error when state.mode == :position -> {:ok, nil}
      :error -> {:error, mode_error(state, :unknown_model, nil, model_number, [])}
    end
  end

  defp validate_model_mode(state, model, model_number) do
    cond do
      state.mode not in model.modes ->
        {:error, mode_error(state, :unsupported_mode, model, model_number, model.modes)}

      state.mode in [:current, :current_position] and is_nil(model.torque_constant) ->
        {:error, mode_error(state, :no_torque_constant, model, model_number, model.modes)}

      true ->
        {:ok, model.torque_constant}
    end
  end

  defp mode_error(state, reason, model, model_number, supported) do
    %ServoModeError{
      servo_id: state.servo_id,
      model: model && model.name,
      model_number: model_number,
      mode: state.mode,
      reason: reason,
      supported: supported
    }
  end

  # Writing it unconditionally would trigger the resets described above on every
  # restart, so only write when the servo isn't already in the right mode.
  defp write_operating_mode(state) do
    operating_mode = Model.operating_mode(state.mode)

    case read_param(state, :operating_mode) do
      {:ok, ^operating_mode} ->
        :ok

      {:ok, _other} ->
        BBProcess.call(
          state.bb.robot,
          state.controller,
          {:write, state.servo_id, :operating_mode, operating_mode}
        )

      {:error, _} = error ->
        error
    end
  end

  defp read_param(state, param) do
    BBProcess.call(state.bb.robot, state.controller, {:read, state.servo_id, param})
  end

  defp register_servo(state) do
    BBProcess.call(
      state.bb.robot,
      state.controller,
      {:register_servo, state.servo_id, state.bb.path, state.position_deadband}
    )
  end

  # `goal_velocity` has no effect in position mode — the speed of a position move
  # comes from `profile_velocity`, which defaults to 0, meaning "as fast as the
  # servo can". Left alone, the joint ignores its own velocity limit and arrives
  # sooner than the `BeginMotion` estimate (which is computed from that limit)
  # predicts. Clamped to one raw unit because 0 doesn't mean "barely move".
  defp write_profile_velocity(state) do
    raw =
      state.motor_profile.motor_velocity_limit
      |> rad_per_sec_to_raw(state.scales.profile_velocity)
      |> max(1)

    BBProcess.call(
      state.bb.robot,
      state.controller,
      {:write_raw, state.servo_id, :profile_velocity, raw}
    )
  end

  # --- Command handling ---

  # What the servo can be asked for depends on the mode it's configured in.
  # Declaring it means the framework refuses everything else with a structured
  # error, rather than the driver accepting a command and doing nothing with it.
  #
  # `Stop` is in every mode, because cutting torque works whatever the servo is
  # doing. `Hold` isn't: a servo in current mode is a torque source with no
  # position to hold, and pretending otherwise would just be `Stop` by another
  # name.
  @impl BB.Actuator
  def command_payloads(opts) do
    opts
    |> Keyword.get(:mode, :position)
    |> mode_payloads()
    |> Enum.sort()
  end

  defp mode_payloads(:current), do: [Command.Effort, Command.Stop]

  defp mode_payloads(:current_position),
    do: [Command.Effort, Command.Hold, Command.Position, Command.Stop]

  defp mode_payloads(:position), do: [Command.Hold, Command.Position, Command.Stop]
  defp mode_payloads(:velocity), do: [Command.Hold, Command.Stop, Command.Velocity]

  @impl BB.Actuator
  def handle_command(%Message{payload: %Command.Position{} = cmd}, state) do
    do_set_position(cmd.position, cmd.command_id, cancel_expiry(state))
  end

  # `Stop` means cease travelling and become passive — the counterpart to `Hold`,
  # which maintains position. Cutting torque is the only way a Dynamixel becomes
  # passive, so both stop modes arrive here: there is no register write that
  # decelerates *into* being passive, and reporting `:decelerate` as unsupported
  # would refuse a stop rather than perform one.
  #
  # The pending goal goes first, so a resume can't act on a target the caller has
  # abandoned, and so it isn't written to a servo that is on its way to limp.
  #
  # This is not the safety path: making hardware safe is `disarm/1`.
  def handle_command(%Message{payload: %Command.Stop{}}, state) do
    reply(cancel_expiry(state), stop_driving(state))
  end

  # A Dynamixel holds whatever it was last told whenever torque is on, so `Hold`
  # has nothing to do in the common case. It is not a no-op after a `Stop`: torque
  # has to come back on, and at the position the joint has come to rest at rather
  # than the goal it was chasing when it went limp. In velocity mode there is no
  # position to return to — holding is commanding a standstill, which a powered
  # servo resists being moved from.
  def handle_command(%Message{payload: %Command.Hold{}}, state) do
    state = cancel_expiry(state)

    cond do
      not torque_enabled?(state) -> reply(state, resume(state, hold_write(state)))
      state.mode == :velocity -> apply_and_reply(state, {:goal_velocity, 0})
      true -> {:noreply, state}
    end
  end

  def handle_command(%Message{payload: %Command.Velocity{} = cmd}, state) do
    raw =
      cmd.velocity
      |> clamp(state.motor_profile.motor_velocity_limit)
      |> rad_per_sec_to_raw(state.scales.goal_velocity)

    state
    |> schedule_expiry(cmd.duration)
    |> apply_and_reply({:goal_velocity, raw})
  end

  # In `:current` mode this is the torque the servo aims for. In
  # `:current_position` mode the same register is a ceiling on the current a
  # position move may draw, which is how you get a gripper that squeezes to a
  # limit rather than to a position.
  def handle_command(%Message{payload: %Command.Effort{} = cmd}, state) do
    raw =
      cmd.effort
      |> clamp(state.motor_profile.motor_effort_limit)
      |> newton_metres_to_raw(state)

    state
    |> schedule_expiry(cmd.duration)
    |> apply_and_reply({:goal_current, raw})
  end

  # `duration` on a velocity or effort command is how long to drive for. What
  # happens when it runs out is the joint's business rather than the caller's, so
  # it comes from `:expiry_action` — the same choice `:disarm_action` makes for
  # the bus, one joint at a time.
  @impl BB.Actuator
  def handle_info({:expire_command, timer}, %{expiry_timer: timer} = state) do
    state = %{state | expiry_timer: nil}

    case state.expiry_action do
      :stop -> reply(state, stop_driving(state))
      :hold -> apply_and_reply(state, hold_write(state))
    end
  end

  def handle_info(_message, state), do: {:noreply, state}

  defp stop_driving(state) do
    clear_pending_goal(state)
    disable_torque(state)
  end

  # Holding means "stop driving but stay under power", which is a different
  # register in each mode. Current mode has no position to hold, so the honest
  # equivalent is asking for no torque; current-based position mode does have
  # one, and keeps whatever current ceiling it was given.
  defp hold_write(%{mode: :current}), do: {:goal_current, 0}
  defp hold_write(%{mode: :velocity}), do: {:goal_velocity, 0}
  defp hold_write(_state), do: :present_position

  defp schedule_expiry(state, nil), do: cancel_expiry(state)

  defp schedule_expiry(state, duration) do
    state = cancel_expiry(state)
    timer = make_ref()
    Process.send_after(self(), {:expire_command, timer}, duration)
    %{state | expiry_timer: timer}
  end

  # The reference, rather than the timer, is what `handle_info/2` matches on, so
  # a message already in the mailbox when a new command arrives is ignored rather
  # than cutting the new command short.
  defp cancel_expiry(state), do: %{state | expiry_timer: nil}

  # --- Position commands ---

  defp do_set_position(angle, command_id, state) when is_integer(angle),
    do: do_set_position(angle * 1.0, command_id, state)

  defp do_set_position(motor_angle, command_id, state) do
    clamped_motor_angle = clamp_motor_angle(motor_angle, state.motor_profile)
    goal_position = motor_angle_to_position(clamped_motor_angle)

    case apply_write(state, {:goal_position, goal_position}) do
      :ok ->
        publish_begin_motion(state, clamped_motor_angle, command_id)
        {:noreply, %{state | current_motor_angle: clamped_motor_angle}}

      {:error, reason} ->
        {:stop, reason, state}
    end
  end

  # --- Applying writes ---

  # The fast path leaves the write for the controller's next tick to batch onto
  # the bus. A servo left passive by a `Stop` can't be commanded that way — the
  # value would land in a register the servo isn't acting on — so it takes the
  # controller's ordered resume instead, going to what was just asked for rather
  # than to whatever it was chasing before.
  defp apply_write(state, pending_write) do
    if torque_enabled?(state) do
      write_servo_command(state, pending_write)
      :ok
    else
      resume(state, pending_write)
    end
  end

  defp apply_and_reply(state, pending_write), do: reply(state, apply_write(state, pending_write))

  defp reply(state, :ok), do: {:noreply, state}
  defp reply(state, {:error, reason}), do: {:stop, reason, state}

  defp publish_begin_motion(state, target_motor_angle, command_id) do
    travel_distance = abs(state.current_motor_angle - target_motor_angle)
    travel_time_ms = round(travel_distance / state.motor_profile.motor_velocity_limit * 1000)
    expected_arrival = System.monotonic_time(:millisecond) + travel_time_ms

    message_opts =
      [
        initial_position: state.current_motor_angle,
        target_position: target_motor_angle,
        expected_arrival: expected_arrival,
        command_type: :position
      ]
      |> maybe_add_opt(:command_id, command_id)

    BB.Actuator.publish_begin_motion(state.bb.robot, state.bb.path, message_opts)
  end

  defp resume(state, pending_write) do
    BBProcess.call(
      state.bb.robot,
      state.controller,
      {:resume_servo, state.servo_id, pending_write}
    )
  end

  defp torque_enabled?(state) do
    case :ets.lookup(state.servo_table, state.servo_id) do
      [{_, _, _, _, _, _, _, _, _, _, torque_enabled}] -> torque_enabled == true
      [] -> false
    end
  rescue
    # The controller owns the table; if it has gone, so has the bus.
    ArgumentError -> false
  end

  defp clear_pending_goal(state), do: write_servo_command(state, nil)

  defp write_servo_command(state, pending_write) do
    :ets.update_element(state.servo_table, state.servo_id, [
      {@ets_idx_pending_write, pending_write}
    ])
  rescue
    ArgumentError -> false
  end

  # --- Helpers ---

  defp maybe_add_opt(opts, _key, nil), do: opts
  defp maybe_add_opt(opts, key, value), do: Keyword.put(opts, key, value)

  defp clamp_motor_angle(motor_angle, %{motor_lower: lower, motor_upper: upper}) do
    motor_angle
    |> max(lower)
    |> min(upper)
  end

  # Velocity and effort limits are magnitudes; the joint may travel either way.
  defp clamp(value, limit) do
    value
    |> max(-limit)
    |> min(limit)
  end

  defp rad_per_sec_to_raw(rad_per_sec, rev_per_min_per_unit) do
    round(rad_per_sec * 60 / (2 * :math.pi()) / rev_per_min_per_unit)
  end

  # Newton metres to amps is the servo's published torque constant; amps to
  # register units is the control table's. Both are approximations — the torque
  # constant is quoted at the recommended supply voltage and drifts with it.
  defp newton_metres_to_raw(newton_metres, state) do
    round(newton_metres / state.torque_constant / state.scales.goal_current)
  end

  # Map motor-space radians to encoder units. Encoder centre corresponds to
  # motor zero; one full motor rotation spans @position_resolution units.
  defp motor_angle_to_position(motor_angle_rad) do
    offset_units = motor_angle_rad / (2 * :math.pi()) * @position_resolution

    round(@position_center + offset_units)
    |> max(0)
    |> min(@position_resolution - 1)
  end
end
