# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Servo.Robotis.ActuatorTest do
  use ExUnit.Case, async: true
  use Mimic

  alias BB.Actuator.MotorProfile
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

    Message.new!(Command.Position, @joint_name, message_opts)
  end

  defp maybe_add_opt(opts, _key, nil), do: opts
  defp maybe_add_opt(opts, key, value), do: Keyword.put(opts, key, value)

  defp default_bb_context do
    %{robot: TestRobot, path: [@joint_name, @actuator_name]}
  end

  defp motor_profile(overrides \\ []) do
    base = %MotorProfile{
      motor_lower: -0.5,
      motor_upper: 0.5,
      motor_velocity_limit: 1.0,
      motor_initial_position: 0.0
    }

    struct!(base, overrides)
  end

  defp stub_controller_success(servo_table) do
    stub(BB.Process, :call, fn _robot, _name, msg ->
      case msg do
        {:write, _id, :torque_enable, false} -> :ok
        {:register_servo, _, _, _} -> {:ok, servo_table}
      end
    end)

    stub(BB, :subscribe, fn _robot, _path -> :ok end)
    stub(BB.Actuator, :publish_begin_motion, fn _robot, _path, _opts -> :ok end)
  end

  describe "init/1" do
    setup do
      servo_table = :ets.new(:test_init_table, [:set, :public])

      on_exit(fn ->
        if :ets.info(servo_table) != :undefined, do: :ets.delete(servo_table)
      end)

      %{servo_table: servo_table}
    end

    test "succeeds with a complete motor profile", %{servo_table: servo_table} do
      stub_controller_success(servo_table)

      opts = [
        bb: default_bb_context(),
        servo_id: 1,
        controller: @controller_name,
        motor_profile: motor_profile()
      ]

      assert {:ok, state} = Actuator.init(opts)

      assert state.motor_profile.motor_lower == -0.5
      assert state.motor_profile.motor_upper == 0.5
      assert state.motor_profile.motor_velocity_limit == 1.0
    end

    test "stores servo_id and controller name", %{servo_table: servo_table} do
      stub_controller_success(servo_table)

      opts = [
        bb: default_bb_context(),
        servo_id: 5,
        controller: @controller_name,
        motor_profile: motor_profile()
      ]

      assert {:ok, state} = Actuator.init(opts)
      assert state.servo_id == 5
      assert state.controller == @controller_name
    end

    test "fails when motor profile has no lower limit" do
      opts = [
        bb: default_bb_context(),
        servo_id: 1,
        controller: @controller_name,
        motor_profile: motor_profile(motor_lower: nil)
      ]

      assert {:stop, %JointConfigError{joint: @joint_name, field: :lower}} = Actuator.init(opts)
    end

    test "fails when motor profile has no upper limit" do
      opts = [
        bb: default_bb_context(),
        servo_id: 1,
        controller: @controller_name,
        motor_profile: motor_profile(motor_upper: nil)
      ]

      assert {:stop, %JointConfigError{joint: @joint_name, field: :upper}} = Actuator.init(opts)
    end

    test "fails when motor profile has no velocity limit" do
      opts = [
        bb: default_bb_context(),
        servo_id: 1,
        controller: @controller_name,
        motor_profile: motor_profile(motor_velocity_limit: nil)
      ]

      assert {:stop, %JointConfigError{joint: @joint_name, field: :velocity}} =
               Actuator.init(opts)
    end

    test "initialises at motor profile's initial position", %{servo_table: servo_table} do
      stub_controller_success(servo_table)

      opts = [
        bb: default_bb_context(),
        servo_id: 1,
        controller: @controller_name,
        motor_profile: motor_profile(motor_initial_position: 0.25)
      ]

      assert {:ok, state} = Actuator.init(opts)
      assert state.current_motor_angle == 0.25
    end

    test "disables torque and registers with controller, passing its own actuator path",
         %{servo_table: servo_table} do
      test_pid = self()

      expect(BB.Process, :call, fn TestRobot,
                                   @controller_name,
                                   {:write, 3, :torque_enable, false} ->
        send(test_pid, :torque_disabled)
        :ok
      end)

      expect(BB.Process, :call, fn TestRobot,
                                   @controller_name,
                                   {:register_servo, 3, [@joint_name, @actuator_name], _deadband} ->
        send(test_pid, :registered_with_controller)
        {:ok, servo_table}
      end)

      stub(BB, :subscribe, fn _robot, _path -> :ok end)

      opts = [
        bb: default_bb_context(),
        servo_id: 3,
        controller: @controller_name,
        motor_profile: motor_profile()
      ]

      {:ok, state} = Actuator.init(opts)

      assert_receive :torque_disabled
      assert_receive :registered_with_controller
      assert state.servo_table == servo_table
    end
  end

  describe "position clamping" do
    setup do
      servo_table = :ets.new(:test_clamp_table, [:set, :public])

      stub_controller_success(servo_table)

      opts = [
        bb: default_bb_context(),
        servo_id: 1,
        controller: @controller_name,
        motor_profile: motor_profile(motor_lower: -1.0, motor_upper: 1.0)
      ]

      {:ok, state} = Actuator.init(opts)

      :ets.insert(
        servo_table,
        {1, [@joint_name, @actuator_name], 2, nil, nil, nil, nil, nil, nil, nil}
      )

      on_exit(fn ->
        if :ets.info(servo_table) != :undefined, do: :ets.delete(servo_table)
      end)

      {:ok, state: state, servo_table: servo_table}
    end

    test "clamps position below lower limit", %{state: state, servo_table: servo_table} do
      Actuator.handle_command(position_command(-5.0), state)

      [{1, _, _, _, _, _, _, _, _, goal_pos}] = :ets.lookup(servo_table, 1)
      # -1.0 rad from motor zero (servo centre 2048)
      # position = 2048 + (-1.0 / 2π * 4096) = 2048 - 652 = 1396
      assert goal_pos == 1396
    end

    test "clamps position above upper limit", %{state: state, servo_table: servo_table} do
      Actuator.handle_command(position_command(5.0), state)

      [{1, _, _, _, _, _, _, _, _, goal_pos}] = :ets.lookup(servo_table, 1)
      # 1.0 rad from motor zero (servo centre 2048)
      # position = 2048 + (1.0 / 2π * 4096) = 2048 + 652 = 2700
      assert goal_pos == 2700
    end
  end

  describe "begin_motion publishing" do
    setup do
      servo_table = :ets.new(:test_motion_table, [:set, :public])

      stub_controller_success(servo_table)

      opts = [
        bb: default_bb_context(),
        servo_id: 1,
        controller: @controller_name,
        motor_profile: motor_profile(motor_lower: -1.0, motor_upper: 1.0)
      ]

      {:ok, state} = Actuator.init(opts)

      :ets.insert(
        servo_table,
        {1, [@joint_name, @actuator_name], 2, nil, nil, nil, nil, nil, nil, nil}
      )

      on_exit(fn ->
        if :ets.info(servo_table) != :undefined, do: :ets.delete(servo_table)
      end)

      {:ok, state: state}
    end

    test "calls publish_begin_motion with motor-space opts", %{state: state} do
      test_pid = self()

      expect(BB.Actuator, :publish_begin_motion, fn robot, path, opts ->
        send(test_pid, {:published, robot, path, opts})
        :ok
      end)

      Actuator.handle_command(position_command(0.5), state)

      assert_receive {:published, TestRobot, [@joint_name, @actuator_name], opts}

      assert opts[:initial_position] == 0.0
      assert opts[:target_position] == 0.5
      assert is_integer(opts[:expected_arrival])
      assert opts[:expected_arrival] > System.monotonic_time(:millisecond)
    end

    test "calculates expected arrival based on velocity", %{state: state} do
      test_pid = self()

      expect(BB.Actuator, :publish_begin_motion, fn _robot, _path, opts ->
        send(test_pid, {:arrival, opts[:expected_arrival]})
        :ok
      end)

      before = System.monotonic_time(:millisecond)
      Actuator.handle_command(position_command(1.0), state)

      assert_receive {:arrival, expected_arrival}

      travel_time_ms = round(1.0 / 1.0 * 1000)
      assert_in_delta expected_arrival, before + travel_time_ms, 50
    end
  end
end
