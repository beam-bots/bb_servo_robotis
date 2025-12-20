# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Servo.Robotis.Actuator do
  @moduledoc """
  An actuator GenServer that uses a Robotis controller to drive a Dynamixel servo.

  This actuator derives its configuration from the joint constraints defined in the robot:
  - Position limits from `joint.limits.lower` and `joint.limits.upper`
  - Velocity limit from `joint.limits.velocity`
  - Position range maps to the servo's goal_position register

  When initialised, the actuator:
  1. Sets the goal position to joint center (servo won't move until armed)
  2. Registers the servo with the controller for position feedback and torque management

  When a position command is received, the actuator:
  1. Clamps the position to joint limits
  2. Converts to servo position units (0-4095 for 360 degrees)
  3. Sends goal_position command to the Robotis controller
  4. Publishes a `BB.Message.Actuator.BeginMotion` message

  Position feedback and torque management are handled by the controller. Servos
  remain unpowered until the robot is armed, then move to their goal positions.

  ## Example DSL Usage

      controller :dynamixel, {BB.Servo.Robotis.Controller,
        port: "/dev/ttyUSB0",
        baud_rate: 1_000_000
      }

      joint :shoulder, type: :revolute do
        limit lower: ~u(-90 degree), upper: ~u(90 degree), velocity: ~u(60 degree_per_second)

        actuator :servo, {BB.Servo.Robotis.Actuator, servo_id: 1, controller: :dynamixel}
      end
  """
  use BB.Actuator

  alias BB.Message
  alias BB.Message.Actuator.BeginMotion
  alias BB.Message.Actuator.Command
  alias BB.Process, as: BBProcess

  @position_resolution 4096
  @position_center 2048

  @doc """
  Safety disarm callback.

  Returns :ok because torque management is handled by the controller.
  The controller receives all registered servo IDs and disables torque
  for all of them in a single sync_write operation, which is more
  efficient for bus-based protocols.
  """
  @impl BB.Actuator
  def disarm(_opts), do: :ok

  @impl BB.Actuator
  def options_schema do
    Spark.Options.new!(
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
      reverse?: [
        type: :boolean,
        doc: "Reverse the servo rotation direction?",
        default: false
      ],
      position_deadband: [
        type: :non_neg_integer,
        doc:
          "Minimum position change (raw units) to trigger feedback publish. Filters servo noise.",
        default: 2
      ]
    )
  end

  @impl GenServer
  def init(opts) do
    with {:ok, state} <- build_state(opts),
         :ok <- disable_torque(state),
         :ok <- set_initial_position(state) do
      # Register servo with controller for position feedback polling and torque management
      :ok =
        BBProcess.call(
          state.bb.robot,
          state.controller,
          {:register_servo, state.servo_id, state.joint_name, state.center_angle,
           state.position_deadband, state.reverse?}
        )

      # Subscribe to position commands published to [:actuator, joint_name, actuator_name]
      BB.subscribe(state.bb.robot, [:actuator, state.joint_name, state.name])

      {:ok, state}
    else
      {:error, reason} -> {:stop, reason}
    end
  end

  defp build_state(opts) do
    opts = Map.new(opts)
    [name, joint_name | _] = Enum.reverse(opts.bb.path)
    robot = opts.bb.robot.robot()

    reverse? = Map.get(opts, :reverse?, false)
    position_deadband = Map.get(opts, :position_deadband, 2)

    with {:ok, joint} <- fetch_joint(robot, joint_name),
         {:ok, limits} <- validate_joint_limits(joint, joint_name) do
      lower_limit = limits.lower
      upper_limit = limits.upper
      range = upper_limit - lower_limit
      center_angle = (lower_limit + upper_limit) / 2
      velocity_limit = limits.velocity

      state = %{
        bb: opts.bb,
        servo_id: opts.servo_id,
        controller: opts.controller,
        reverse?: reverse?,
        position_deadband: position_deadband,
        lower_limit: lower_limit,
        upper_limit: upper_limit,
        center_angle: center_angle,
        range: range,
        velocity_limit: velocity_limit,
        current_angle: center_angle,
        name: name,
        joint_name: joint_name
      }

      {:ok, state}
    end
  end

  defp fetch_joint(robot, joint_name) do
    case BB.Robot.get_joint(robot, joint_name) do
      nil -> {:error, {:joint_not_found, joint_name}}
      joint -> {:ok, joint}
    end
  end

  defp validate_joint_limits(%{type: :continuous}, joint_name) do
    {:error, {:unsupported_joint_type, :continuous, joint_name}}
  end

  defp validate_joint_limits(%{limits: nil}, joint_name) do
    {:error, {:no_limits_defined, joint_name}}
  end

  defp validate_joint_limits(%{limits: %{lower: nil}}, joint_name) do
    {:error, {:missing_limit, :lower, joint_name}}
  end

  defp validate_joint_limits(%{limits: %{upper: nil}}, joint_name) do
    {:error, {:missing_limit, :upper, joint_name}}
  end

  defp validate_joint_limits(%{limits: limits}, _joint_name) do
    {:ok, limits}
  end

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

  defp set_initial_position(state) do
    position = angle_to_position(state.center_angle, state)

    case BBProcess.call(
           state.bb.robot,
           state.controller,
           {:write_raw, state.servo_id, :goal_position, position}
         ) do
      :ok -> :ok
      {:ok, _} -> :ok
      {:error, _} = error -> error
    end
  end

  @impl GenServer
  def handle_info({:bb, _path, %Message{payload: %Command.Position{} = cmd}}, state) do
    {:noreply, state} = do_set_position(cmd.position, cmd.command_id, state)
    {:noreply, state}
  end

  @impl GenServer
  def handle_cast({:command, %Message{payload: %Command.Position{} = cmd}}, state) do
    do_set_position(cmd.position, cmd.command_id, state)
  end

  @impl GenServer
  def handle_call({:command, %Message{payload: %Command.Position{} = cmd}}, _from, state) do
    {:noreply, new_state} = do_set_position(cmd.position, cmd.command_id, state)
    {:reply, {:ok, :accepted}, new_state}
  end

  defp do_set_position(angle, command_id, state) when is_integer(angle),
    do: do_set_position(angle * 1.0, command_id, state)

  defp do_set_position(angle, command_id, state) do
    clamped_angle = clamp_angle(angle, state)
    new_position = angle_to_position(clamped_angle, state)

    :ok =
      BBProcess.cast(
        state.bb.robot,
        state.controller,
        {:write_raw, state.servo_id, :goal_position, new_position}
      )

    travel_distance = abs(state.current_angle - clamped_angle)
    travel_time_ms = round(travel_distance / state.velocity_limit * 1000)
    expected_arrival = System.monotonic_time(:millisecond) + travel_time_ms

    message_opts =
      [
        initial_position: state.current_angle,
        target_position: clamped_angle,
        expected_arrival: expected_arrival,
        command_type: :position
      ]
      |> maybe_add_opt(:command_id, command_id)

    message = Message.new!(BeginMotion, state.joint_name, message_opts)

    BB.publish(state.bb.robot, [:actuator | state.bb.path], message)

    {:noreply, %{state | current_angle: clamped_angle}}
  end

  defp maybe_add_opt(opts, _key, nil), do: opts
  defp maybe_add_opt(opts, key, value), do: Keyword.put(opts, key, value)

  defp clamp_angle(angle, state) do
    angle
    |> max(state.lower_limit)
    |> min(state.upper_limit)
  end

  defp angle_to_position(angle_rad, state) do
    # Convert angle offset from joint center to servo position units
    # Servo center (2048) corresponds to joint center angle
    offset_rad = angle_rad - state.center_angle
    offset_units = offset_rad / (2 * :math.pi()) * @position_resolution

    position =
      if state.reverse? do
        @position_center - offset_units
      else
        @position_center + offset_units
      end

    # Clamp to valid servo range (0-4095)
    round(position)
    |> max(0)
    |> min(@position_resolution - 1)
  end
end
