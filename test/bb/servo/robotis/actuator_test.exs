# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Servo.Robotis.ActuatorTest do
  use ExUnit.Case, async: true
  use Mimic

  alias BB.Error.Invalid.JointConfig, as: JointConfigError
  alias BB.Message
  alias BB.Message.Actuator.Command
  alias BB.Servo.Robotis.Actuator

  @joint_name :test_joint
  @actuator_name :test_servo
  @controller_name :test_dynamixel

  defp position_command(position, opts \\ []) do
    message_opts =
      [position: position * 1.0]
      |> maybe_add_opt(:command_id, opts[:command_id])

    {:command, Message.new!(Command.Position, @joint_name, message_opts)}
  end

  defp maybe_add_opt(opts, _key, nil), do: opts
  defp maybe_add_opt(opts, key, value), do: Keyword.put(opts, key, value)

  defp default_bb_context do
    %{robot: TestRobot, path: [@joint_name, @actuator_name]}
  end

  defp joint_with_limits(lower, upper, velocity) do
    %{
      type: :revolute,
      limits: %{
        lower: lower,
        upper: upper,
        velocity: velocity,
        effort: 1.0
      }
    }
  end

  defp stub_controller_success do
    stub(BB.Process, :call, fn _robot, _name, _msg, _timeout -> :ok end)
    stub(BB.Process, :call, fn _robot, _name, _msg -> :ok end)
    stub(BB.Process, :cast, fn _robot, _name, _msg -> :ok end)
    stub(BB, :subscribe, fn _robot, _path -> :ok end)
    stub(BB.Safety, :armed?, fn _robot -> true end)
  end

  describe "init/1" do
    test "succeeds with valid joint limits" do
      stub(BB.Robot, :get_joint, fn _robot, @joint_name ->
        joint_with_limits(-0.5, 0.5, 1.0)
      end)

      stub_controller_success()

      opts = [bb: default_bb_context(), servo_id: 1, controller: @controller_name]
      assert {:ok, state} = Actuator.init(opts)

      assert state.lower_limit == -0.5
      assert state.upper_limit == 0.5
      assert state.velocity_limit == 1.0
      assert state.range == 1.0
      assert state.center_angle == 0.0
    end

    test "stores servo_id and controller name" do
      stub(BB.Robot, :get_joint, fn _robot, @joint_name ->
        joint_with_limits(-0.5, 0.5, 1.0)
      end)

      stub_controller_success()

      opts = [bb: default_bb_context(), servo_id: 5, controller: @controller_name]
      assert {:ok, state} = Actuator.init(opts)

      assert state.servo_id == 5
      assert state.controller == @controller_name
    end

    test "fails when joint not found" do
      stub(BB.Robot, :get_joint, fn _robot, @joint_name -> nil end)

      opts = [bb: default_bb_context(), servo_id: 1, controller: @controller_name]
      assert {:stop, %JointConfigError{joint: @joint_name}} = Actuator.init(opts)
    end

    test "fails for continuous joints" do
      stub(BB.Robot, :get_joint, fn _robot, @joint_name ->
        %{type: :continuous, limits: %{lower: nil, upper: nil, velocity: 1.0, effort: 1.0}}
      end)

      opts = [bb: default_bb_context(), servo_id: 1, controller: @controller_name]

      assert {:stop, %JointConfigError{joint: @joint_name, field: :type, value: :continuous}} =
               Actuator.init(opts)
    end

    test "fails when limits not defined" do
      stub(BB.Robot, :get_joint, fn _robot, @joint_name ->
        %{type: :revolute, limits: nil}
      end)

      opts = [bb: default_bb_context(), servo_id: 1, controller: @controller_name]
      assert {:stop, %JointConfigError{joint: @joint_name, field: :limits}} = Actuator.init(opts)
    end

    test "fails when lower limit missing" do
      stub(BB.Robot, :get_joint, fn _robot, @joint_name ->
        %{type: :revolute, limits: %{lower: nil, upper: 0.5, velocity: 1.0, effort: 1.0}}
      end)

      opts = [bb: default_bb_context(), servo_id: 1, controller: @controller_name]
      assert {:stop, %JointConfigError{joint: @joint_name, field: :lower}} = Actuator.init(opts)
    end

    test "fails when upper limit missing" do
      stub(BB.Robot, :get_joint, fn _robot, @joint_name ->
        %{type: :revolute, limits: %{lower: -0.5, upper: nil, velocity: 1.0, effort: 1.0}}
      end)

      opts = [bb: default_bb_context(), servo_id: 1, controller: @controller_name]
      assert {:stop, %JointConfigError{joint: @joint_name, field: :upper}} = Actuator.init(opts)
    end

    test "initialises servo at center position" do
      stub(BB.Robot, :get_joint, fn _robot, @joint_name ->
        joint_with_limits(-1.0, 1.0, 1.0)
      end)

      stub_controller_success()

      opts = [bb: default_bb_context(), servo_id: 1, controller: @controller_name]
      assert {:ok, state} = Actuator.init(opts)

      assert state.current_angle == 0.0
    end

    test "disables torque, sends initial position, and registers with controller" do
      stub(BB.Robot, :get_joint, fn _robot, @joint_name ->
        joint_with_limits(-1.0, 1.0, 1.0)
      end)

      test_pid = self()

      expect(BB.Process, :call, fn TestRobot,
                                   @controller_name,
                                   {:write, 3, :torque_enable, false} ->
        send(test_pid, :torque_disabled)
        :ok
      end)

      expect(BB.Process, :call, fn TestRobot,
                                   @controller_name,
                                   {:write_raw, 3, :goal_position, _pos} ->
        send(test_pid, :initial_position_sent)
        :ok
      end)

      expect(BB.Process, :call, fn TestRobot,
                                   @controller_name,
                                   {:register_servo, 3, @joint_name, _center_angle, _deadband,
                                    _reverse?} ->
        send(test_pid, :registered_with_controller)
        :ok
      end)

      stub(BB, :subscribe, fn _robot, _path -> :ok end)

      opts = [bb: default_bb_context(), servo_id: 3, controller: @controller_name]
      {:ok, _state} = Actuator.init(opts)

      assert_receive :torque_disabled
      assert_receive :initial_position_sent
      assert_receive :registered_with_controller
    end
  end

  describe "position clamping" do
    setup do
      stub(BB.Robot, :get_joint, fn _robot, @joint_name ->
        joint_with_limits(-1.0, 1.0, 1.0)
      end)

      stub_controller_success()
      stub(BB, :publish, fn _robot, _path, _msg -> :ok end)

      opts = [bb: default_bb_context(), servo_id: 1, controller: @controller_name]

      {:ok, state} = Actuator.init(opts)

      {:ok, state: state}
    end

    test "clamps position below lower limit", %{state: state} do
      test_pid = self()

      expect(BB.Process, :cast, fn TestRobot,
                                   @controller_name,
                                   {:write_raw, 1, :goal_position, pos} ->
        send(test_pid, {:position, pos})
        :ok
      end)

      Actuator.handle_cast(position_command(-5.0), state)

      assert_receive {:position, pos}
      # -1.0 rad from joint center (0), servo center is 2048
      # position = 2048 + (-1.0 / 2π * 4096) = 2048 - 652 = 1396
      assert pos == 1396
    end

    test "clamps position above upper limit", %{state: state} do
      test_pid = self()

      expect(BB.Process, :cast, fn TestRobot,
                                   @controller_name,
                                   {:write_raw, 1, :goal_position, pos} ->
        send(test_pid, {:position, pos})
        :ok
      end)

      Actuator.handle_cast(position_command(5.0), state)

      assert_receive {:position, pos}
      # 1.0 rad from joint center (0), servo center is 2048
      # position = 2048 + (1.0 / 2π * 4096) = 2048 + 652 = 2700
      assert pos == 2700
    end
  end

  describe "begin_motion publishing" do
    setup do
      stub(BB.Robot, :get_joint, fn _robot, @joint_name ->
        joint_with_limits(-1.0, 1.0, 1.0)
      end)

      stub_controller_success()

      opts = [bb: default_bb_context(), servo_id: 1, controller: @controller_name]
      {:ok, state} = Actuator.init(opts)

      {:ok, state: state}
    end

    test "publishes begin_motion message", %{state: state} do
      test_pid = self()

      expect(BB, :publish, fn robot, path, message ->
        send(test_pid, {:published, robot, path, message})
        :ok
      end)

      Actuator.handle_cast(position_command(0.5), state)

      assert_receive {:published, TestRobot, [:actuator, @joint_name, @actuator_name], message}

      assert %BB.Message{payload: %BB.Message.Actuator.BeginMotion{} = cmd} = message
      assert cmd.initial_position == 0.0
      assert cmd.target_position == 0.5
      assert is_integer(cmd.expected_arrival)
      assert cmd.expected_arrival > System.monotonic_time(:millisecond)
    end

    test "calculates expected arrival based on velocity", %{state: state} do
      test_pid = self()

      expect(BB, :publish, fn _robot, _path, %BB.Message{payload: cmd} ->
        send(test_pid, {:arrival, cmd.expected_arrival})
        :ok
      end)

      before = System.monotonic_time(:millisecond)
      Actuator.handle_cast(position_command(1.0), state)

      assert_receive {:arrival, expected_arrival}

      travel_time_ms = round(1.0 / 1.0 * 1000)
      assert_in_delta expected_arrival, before + travel_time_ms, 50
    end
  end
end
