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
  2. Registers with the controller, receiving the shared ETS table reference
  3. Subscribes to position commands

  When a position command is received, the actuator:
  1. Clamps the position to motor limits
  2. Converts to servo position units (0-4095 for 360 degrees)
  3. Writes goal_position to the controller's ETS table
  4. Publishes a `BB.Message.Actuator.BeginMotion` via
     `BB.Actuator.publish_begin_motion/3` (which handles the
     motor → joint-space conversion)

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
      ]
    ]

  alias BB.Error.Invalid.JointConfig, as: JointConfigError
  alias BB.Message
  alias BB.Message.Actuator.Command
  alias BB.Process, as: BBProcess

  @position_resolution 4096
  @position_center 2048

  # ETS tuple field index for command writes
  @ets_idx_goal_position 10

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
         {:ok, servo_table} <- register_servo(state) do
      {:ok, Map.put(state, :servo_table, servo_table)}
    else
      {:error, reason} -> {:stop, reason}
    end
  end

  @impl BB.Actuator
  def handle_options(new_opts, state) do
    motor_profile = Keyword.fetch!(new_opts, :motor_profile)

    {:ok,
     %{
       state
       | motor_profile: motor_profile,
         current_motor_angle: clamp_motor_angle(state.current_motor_angle, motor_profile)
     }}
  end

  defp build_state(opts) do
    opts = Map.new(opts)
    [name, joint_name | _] = Enum.reverse(opts.bb.path)
    motor_profile = opts.motor_profile

    with :ok <- validate_motor_profile(motor_profile, joint_name) do
      state = %{
        bb: opts.bb,
        servo_id: opts.servo_id,
        controller: opts.controller,
        position_deadband: Map.get(opts, :position_deadband, 2),
        motor_profile: motor_profile,
        current_motor_angle: motor_profile.motor_initial_position,
        name: name,
        joint_name: joint_name
      }

      {:ok, state}
    end
  end

  defp validate_motor_profile(%{motor_lower: nil}, joint_name) do
    {:error,
     %JointConfigError{
       joint: joint_name,
       field: :lower,
       value: nil,
       message: "Joint must have a lower limit defined for servo control"
     }}
  end

  defp validate_motor_profile(%{motor_upper: nil}, joint_name) do
    {:error,
     %JointConfigError{
       joint: joint_name,
       field: :upper,
       value: nil,
       message: "Joint must have an upper limit defined for servo control"
     }}
  end

  defp validate_motor_profile(%{motor_velocity_limit: nil}, joint_name) do
    {:error,
     %JointConfigError{
       joint: joint_name,
       field: :velocity,
       value: nil,
       message: "Joint must have a velocity limit defined for servo control"
     }}
  end

  defp validate_motor_profile(_profile, _joint_name), do: :ok

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

  defp register_servo(state) do
    BBProcess.call(
      state.bb.robot,
      state.controller,
      {:register_servo, state.servo_id, state.bb.path, state.position_deadband}
    )
  end

  # --- Command handling ---

  # This driver implements position commands. Declaring that here means anything
  # else is refused by the framework with a structured error, rather than
  # reaching the catch-all below and being silently dropped.
  @impl BB.Actuator
  def command_payloads(_opts), do: [Command.Position]

  @impl BB.Actuator
  def handle_command(%Message{payload: %Command.Position{} = cmd}, state) do
    do_set_position(cmd.position, cmd.command_id, state)
  end

  def handle_command(%Message{}, state), do: {:noreply, state}

  # --- Position commands ---

  defp do_set_position(angle, command_id, state) when is_integer(angle),
    do: do_set_position(angle * 1.0, command_id, state)

  defp do_set_position(motor_angle, command_id, state) do
    clamped_motor_angle = clamp_motor_angle(motor_angle, state.motor_profile)
    goal_position = motor_angle_to_position(clamped_motor_angle)

    write_servo_command(state, goal_position)

    travel_distance = abs(state.current_motor_angle - clamped_motor_angle)
    travel_time_ms = round(travel_distance / state.motor_profile.motor_velocity_limit * 1000)
    expected_arrival = System.monotonic_time(:millisecond) + travel_time_ms

    message_opts =
      [
        initial_position: state.current_motor_angle,
        target_position: clamped_motor_angle,
        expected_arrival: expected_arrival,
        command_type: :position
      ]
      |> maybe_add_opt(:command_id, command_id)

    BB.Actuator.publish_begin_motion(state.bb.robot, state.bb.path, message_opts)

    {:noreply, %{state | current_motor_angle: clamped_motor_angle}}
  end

  defp write_servo_command(state, goal_position) do
    :ets.update_element(state.servo_table, state.servo_id, [
      {@ets_idx_goal_position, goal_position}
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

  # Map motor-space radians to encoder units. Encoder centre corresponds to
  # motor zero; one full motor rotation spans @position_resolution units.
  defp motor_angle_to_position(motor_angle_rad) do
    offset_units = motor_angle_rad / (2 * :math.pi()) * @position_resolution

    round(@position_center + offset_units)
    |> max(0)
    |> min(@position_resolution - 1)
  end
end
