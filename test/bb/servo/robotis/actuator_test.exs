# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Servo.Robotis.ActuatorTest do
  use ExUnit.Case, async: true
  use Mimic

  alias BB.Actuator.MotorProfile
  alias BB.Error.Invalid.JointConfig, as: JointConfigError
  alias BB.Error.Invalid.Robotis.ServoMode, as: ServoModeError
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

  defp stop_command(opts \\ []), do: Message.new!(Command.Stop, @joint_name, opts)

  defp hold_command, do: Message.new!(Command.Hold, @joint_name, [])

  defp velocity_command(velocity, opts \\ []),
    do: Message.new!(Command.Velocity, @joint_name, [velocity: velocity] ++ opts)

  defp effort_command(effort, opts \\ []),
    do: Message.new!(Command.Effort, @joint_name, [effort: effort] ++ opts)

  # An initialised actuator whose joint spans ±1 rad and turns at 1 rad/s, ready
  # to be commanded. The servo's ETS row is left to the caller, since whether
  # torque is on is the thing under test.
  defp commanding_actuator(opts \\ []) do
    {profile_overrides, actuator_opts} = Keyword.split(opts, [:motor_effort_limit])
    servo_table = :ets.new(:test_command_table, [:set, :public])

    stub_controller_success(servo_table)

    motor_profile =
      motor_profile([motor_lower: -1.0, motor_upper: 1.0] ++ profile_overrides)

    opts =
      [
        bb: default_bb_context(),
        servo_id: 1,
        controller: @controller_name,
        motor_profile: motor_profile
      ] ++ actuator_opts

    {:ok, state} = Actuator.init(opts)

    # Startup's own traffic, so a test can assert on what its command did.
    flush_controller_calls()

    on_exit(fn ->
      if :ets.info(servo_table) != :undefined, do: :ets.delete(servo_table)
    end)

    %{state: state, servo_table: servo_table}
  end

  defp flush_controller_calls do
    receive do
      {:controller_call, _} -> flush_controller_calls()
    after
      0 -> :ok
    end
  end

  defp insert_servo_row(servo_table, opts \\ []) do
    :ets.insert(
      servo_table,
      {1, [@joint_name, @actuator_name], 2, nil, nil, nil, nil, nil, nil,
       Keyword.get(opts, :pending_write), Keyword.get(opts, :torque_enabled, true)}
    )
  end

  defp pending_write(servo_table) do
    [{1, _, _, _, _, _, _, _, _, pending_write, _}] = :ets.lookup(servo_table, 1)
    pending_write
  end

  defp stub_controller_success(servo_table, overrides \\ %{}) do
    test_pid = self()

    stub(BB.Process, :call, fn _robot, _name, msg ->
      send(test_pid, {:controller_call, msg})
      Map.get_lazy(overrides, msg, fn -> controller_reply(msg, servo_table) end)
    end)

    stub(BB, :subscribe, fn _robot, _path -> :ok end)
    stub(BB.Actuator, :publish_begin_motion, fn _robot, _path, _opts -> :ok end)
  end

  # An XM430-W350 already in position mode, which is what `init/1` interrogates
  # before it will start.
  defp controller_reply(:get_control_table, _table), do: {:ok, Robotis.ControlTable.XM430}
  defp controller_reply({:read, _id, :model_number}, _table), do: {:ok, 1020}
  defp controller_reply({:read, _id, :operating_mode}, _table), do: {:ok, :position_control}
  defp controller_reply({:write, _id, :operating_mode, _mode}, _table), do: :ok
  defp controller_reply({:write, _id, :torque_enable, false}, _table), do: :ok
  defp controller_reply({:write_raw, _id, :profile_velocity, _raw}, _table), do: :ok
  defp controller_reply({:register_servo, _, _, _}, table), do: {:ok, table}

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
      stub_controller_success(servo_table)

      opts = [
        bb: default_bb_context(),
        servo_id: 3,
        controller: @controller_name,
        motor_profile: motor_profile()
      ]

      {:ok, state} = Actuator.init(opts)

      assert_receive {:controller_call, {:write, 3, :torque_enable, false}}

      assert_receive {:controller_call,
                      {:register_servo, 3, [@joint_name, @actuator_name], _deadband}}

      assert state.servo_table == servo_table

      # 1 rad/s is 9.55 rev/min, and the register counts in 0.229 rev/min.
      assert_receive {:controller_call, {:write_raw, 3, :profile_velocity, 42}}
    end

    test "clamps profile_velocity to a unit rather than letting 0 mean unlimited",
         %{servo_table: servo_table} do
      stub_controller_success(servo_table)

      opts = [
        bb: default_bb_context(),
        servo_id: 1,
        controller: @controller_name,
        motor_profile: motor_profile(motor_velocity_limit: 0.0001)
      ]

      {:ok, _state} = Actuator.init(opts)

      assert_receive {:controller_call, {:write_raw, 1, :profile_velocity, 1}}
    end

    test "cuts torque before writing the operating mode, and only when it differs",
         %{servo_table: servo_table} do
      stub_controller_success(servo_table, %{
        {:read, 1, :operating_mode} => {:ok, :velocity_control}
      })

      opts = [
        bb: default_bb_context(),
        servo_id: 1,
        controller: @controller_name,
        motor_profile: motor_profile()
      ]

      {:ok, _state} = Actuator.init(opts)

      # `operating_mode` is EEPROM, so it can only be written with torque off.
      assert_receive {:controller_call, {:write, 1, :torque_enable, false}}
      assert_receive {:controller_call, {:write, 1, :operating_mode, :position_control}}
    end

    test "leaves the operating mode alone when it already matches",
         %{servo_table: servo_table} do
      stub_controller_success(servo_table)

      opts = [
        bb: default_bb_context(),
        servo_id: 1,
        controller: @controller_name,
        motor_profile: motor_profile()
      ]

      {:ok, _state} = Actuator.init(opts)

      # Writing it regardless would reset the servo's PID gains, profile velocity
      # and goal current every time the actuator restarted.
      refute_received {:controller_call, {:write, 1, :operating_mode, _}}
    end

    test "refuses a mode the servo doesn't implement", %{servo_table: servo_table} do
      stub_controller_success(servo_table, %{{:read, 1, :model_number} => {:ok, 1060}})

      opts = [
        bb: default_bb_context(),
        servo_id: 1,
        controller: @controller_name,
        mode: :current,
        motor_profile: motor_profile(motor_effort_limit: 1.0)
      ]

      assert {:stop, %ServoModeError{reason: :unsupported_mode, model: "XL430-W250"}} =
               Actuator.init(opts)
    end

    test "refuses a current mode on a servo with no known torque constant",
         %{servo_table: servo_table} do
      stub_controller_success(servo_table, %{{:read, 1, :model_number} => {:ok, 4242}})

      opts = [
        bb: default_bb_context(),
        servo_id: 1,
        controller: @controller_name,
        mode: :current,
        motor_profile: motor_profile(motor_effort_limit: 1.0)
      ]

      assert {:stop, %ServoModeError{reason: :unknown_model}} = Actuator.init(opts)
    end

    test "drives an unrecognised servo in position mode", %{servo_table: servo_table} do
      stub_controller_success(servo_table, %{{:read, 1, :model_number} => {:ok, 4242}})

      opts = [
        bb: default_bb_context(),
        servo_id: 1,
        controller: @controller_name,
        motor_profile: motor_profile()
      ]

      assert {:ok, state} = Actuator.init(opts)
      assert state.torque_constant == nil
    end

    test "requires an effort limit to be driven by current", %{servo_table: servo_table} do
      stub_controller_success(servo_table)

      opts = [
        bb: default_bb_context(),
        servo_id: 1,
        controller: @controller_name,
        mode: :current,
        motor_profile: motor_profile()
      ]

      assert {:stop, %JointConfigError{field: :effort}} = Actuator.init(opts)
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

      insert_servo_row(servo_table)

      on_exit(fn ->
        if :ets.info(servo_table) != :undefined, do: :ets.delete(servo_table)
      end)

      {:ok, state: state, servo_table: servo_table}
    end

    test "clamps position below lower limit", %{state: state, servo_table: servo_table} do
      Actuator.handle_command(position_command(-5.0), state)

      # -1.0 rad from motor zero (servo centre 2048)
      # position = 2048 + (-1.0 / 2π * 4096) = 2048 - 652 = 1396
      assert pending_write(servo_table) == {:goal_position, 1396}
    end

    test "clamps position above upper limit", %{state: state, servo_table: servo_table} do
      Actuator.handle_command(position_command(5.0), state)

      # 1.0 rad from motor zero (servo centre 2048)
      # position = 2048 + (1.0 / 2π * 4096) = 2048 + 652 = 2700
      assert pending_write(servo_table) == {:goal_position, 2700}
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

      insert_servo_row(servo_table)

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

  describe "Command.Stop" do
    setup do
      %{state: state, servo_table: servo_table} = commanding_actuator()
      insert_servo_row(servo_table, pending_write: {:goal_position, 2700})

      {:ok, state: state, servo_table: servo_table}
    end

    test "cuts torque and drops the pending goal", %{state: state, servo_table: servo_table} do
      test_pid = self()

      expect(BB.Process, :call, fn TestRobot,
                                   @controller_name,
                                   {:write, 1, :torque_enable, false} ->
        send(test_pid, :torque_disabled)
        :ok
      end)

      assert {:noreply, _state} = Actuator.handle_command(stop_command(), state)

      assert_receive :torque_disabled
      assert pending_write(servo_table) == nil
    end

    test "decelerate stops the same way as immediate", %{state: state} do
      expect(BB.Process, :call, fn _robot, _name, {:write, 1, :torque_enable, false} -> :ok end)

      assert {:noreply, _state} = Actuator.handle_command(stop_command(mode: :decelerate), state)
    end

    test "stops the actuator when the bus refuses", %{state: state} do
      expect(BB.Process, :call, fn _robot, _name, {:write, 1, :torque_enable, false} ->
        {:error, :timeout}
      end)

      assert {:stop, :timeout, _state} = Actuator.handle_command(stop_command(), state)
    end
  end

  describe "Command.Hold" do
    test "does nothing while the servo is under power" do
      %{state: state, servo_table: servo_table} = commanding_actuator()
      insert_servo_row(servo_table, torque_enabled: true)

      test_pid = self()

      stub(BB.Process, :call, fn _robot, _name, msg ->
        send(test_pid, {:controller_called, msg})
        :ok
      end)

      assert {:noreply, _state} = Actuator.handle_command(hold_command(), state)

      refute_received {:controller_called, _}
    end

    test "re-applies torque at the present position when the servo is passive" do
      %{state: state, servo_table: servo_table} = commanding_actuator()
      insert_servo_row(servo_table, torque_enabled: false)

      test_pid = self()

      expect(BB.Process, :call, fn TestRobot,
                                   @controller_name,
                                   {:resume_servo, 1, :present_position} ->
        send(test_pid, :resumed)
        :ok
      end)

      assert {:noreply, _state} = Actuator.handle_command(hold_command(), state)

      assert_receive :resumed
    end

    test "stops the actuator when the resume fails" do
      %{state: state, servo_table: servo_table} = commanding_actuator()
      insert_servo_row(servo_table, torque_enabled: false)

      expect(BB.Process, :call, fn _robot, _name, {:resume_servo, 1, :present_position} ->
        {:error, :timeout}
      end)

      assert {:stop, :timeout, _state} = Actuator.handle_command(hold_command(), state)
    end
  end

  describe "Command.Position on a passive servo" do
    setup do
      %{state: state, servo_table: servo_table} = commanding_actuator()
      insert_servo_row(servo_table, torque_enabled: false)

      {:ok, state: state, servo_table: servo_table}
    end

    test "resumes at the commanded position rather than leaving a pending write",
         %{state: state, servo_table: servo_table} do
      test_pid = self()

      expect(BB.Process, :call, fn TestRobot,
                                   @controller_name,
                                   {:resume_servo, 1, {:goal_position, 2700}} ->
        send(test_pid, :resumed)
        :ok
      end)

      assert {:noreply, _state} = Actuator.handle_command(position_command(1.0), state)

      assert_receive :resumed
      assert pending_write(servo_table) == nil
    end

    test "still publishes BeginMotion", %{state: state} do
      test_pid = self()

      stub(BB.Process, :call, fn _robot, _name, {:resume_servo, 1, _pending_write} -> :ok end)

      expect(BB.Actuator, :publish_begin_motion, fn _robot, _path, opts ->
        send(test_pid, {:published, opts})
        :ok
      end)

      Actuator.handle_command(position_command(1.0), state)

      assert_receive {:published, opts}
      assert opts[:target_position] == 1.0
    end

    test "stops the actuator without reporting motion when the resume fails", %{state: state} do
      test_pid = self()

      expect(BB.Process, :call, fn _robot, _name, {:resume_servo, 1, _pending_write} ->
        {:error, :timeout}
      end)

      stub(BB.Actuator, :publish_begin_motion, fn _robot, _path, opts ->
        send(test_pid, {:published, opts})
        :ok
      end)

      assert {:stop, :timeout, _state} = Actuator.handle_command(position_command(1.0), state)

      refute_received {:published, _}
    end
  end

  describe "command_payloads/1" do
    test "position mode can be positioned and held, but not driven" do
      assert Actuator.command_payloads(mode: :position) ==
               [Command.Hold, Command.Position, Command.Stop]
    end

    test "velocity mode swaps position for velocity" do
      assert Actuator.command_payloads(mode: :velocity) ==
               [Command.Hold, Command.Stop, Command.Velocity]
    end

    test "current mode is a torque source with no position to hold" do
      assert Actuator.command_payloads(mode: :current) == [Command.Effort, Command.Stop]
    end

    test "current-based position mode takes both a target and a ceiling" do
      assert Actuator.command_payloads(mode: :current_position) ==
               [Command.Effort, Command.Hold, Command.Position, Command.Stop]
    end

    test "defaults to position" do
      assert Actuator.command_payloads([]) == Actuator.command_payloads(mode: :position)
    end
  end

  describe "Command.Velocity" do
    setup do
      %{state: state, servo_table: servo_table} = commanding_actuator(mode: :velocity)
      insert_servo_row(servo_table)

      {:ok, state: state, servo_table: servo_table}
    end

    test "converts rad/s to the servo's velocity units", %{state: state, servo_table: table} do
      assert {:noreply, _state} =
               Actuator.handle_command(velocity_command(1.0), state)

      # 1 rad/s is 9.55 rev/min over 0.229 rev/min per unit.
      assert pending_write(table) == {:goal_velocity, 42}
    end

    test "drives in reverse", %{state: state, servo_table: table} do
      Actuator.handle_command(velocity_command(-1.0), state)

      assert pending_write(table) == {:goal_velocity, -42}
    end

    test "clamps to the joint's velocity limit in both directions",
         %{state: state, servo_table: table} do
      Actuator.handle_command(velocity_command(50.0), state)
      assert pending_write(table) == {:goal_velocity, 42}

      Actuator.handle_command(velocity_command(-50.0), state)
      assert pending_write(table) == {:goal_velocity, -42}
    end
  end

  describe "Command.Effort" do
    setup do
      %{state: state, servo_table: servo_table} =
        commanding_actuator(mode: :current, motor_effort_limit: 4.0)

      insert_servo_row(servo_table)

      {:ok, state: state, servo_table: servo_table}
    end

    test "converts newton metres to the servo's current units",
         %{state: state, servo_table: table} do
      Actuator.handle_command(effort_command(1.783), state)

      # The XM430-W350 makes 1.783 Nm per amp, and counts current in 2.69 mA.
      assert pending_write(table) == {:goal_current, 372}
    end

    test "clamps to the joint's effort limit", %{state: state, servo_table: table} do
      Actuator.handle_command(effort_command(100.0), state)

      {:goal_current, clamped} = pending_write(table)
      assert clamped == round(4.0 / 1.783 / 0.00269)
    end
  end

  describe "duration" do
    test "goes passive when the command expires and expiry_action is :stop" do
      %{state: state, servo_table: servo_table} = commanding_actuator(mode: :velocity)
      insert_servo_row(servo_table)

      {:noreply, state} = Actuator.handle_command(velocity_command(1.0, duration: 20), state)

      assert_receive {:expire_command, _} = expiry, 200
      assert {:noreply, _state} = Actuator.handle_info(expiry, state)

      assert_receive {:controller_call, {:write, 1, :torque_enable, false}}
      assert pending_write(servo_table) == nil
    end

    test "stays under power when expiry_action is :hold" do
      %{state: state, servo_table: servo_table} =
        commanding_actuator(mode: :velocity, expiry_action: :hold)

      insert_servo_row(servo_table)

      {:noreply, state} = Actuator.handle_command(velocity_command(1.0, duration: 20), state)

      assert_receive {:expire_command, _} = expiry, 200
      assert {:noreply, _state} = Actuator.handle_info(expiry, state)

      refute_received {:controller_call, {:write, 1, :torque_enable, false}}
      assert pending_write(servo_table) == {:goal_velocity, 0}
    end

    test "a later command supersedes an expiry already in flight" do
      %{state: state, servo_table: servo_table} = commanding_actuator(mode: :velocity)
      insert_servo_row(servo_table)

      {:noreply, state} = Actuator.handle_command(velocity_command(1.0, duration: 20), state)
      assert_receive {:expire_command, _} = stale, 200

      {:noreply, state} = Actuator.handle_command(velocity_command(0.5, duration: 5_000), state)

      assert {:noreply, _state} = Actuator.handle_info(stale, state)

      refute_received {:controller_call, {:write, 1, :torque_enable, false}}
      assert pending_write(servo_table) == {:goal_velocity, 21}
    end

    test "no duration means no expiry" do
      %{state: state, servo_table: servo_table} = commanding_actuator(mode: :velocity)
      insert_servo_row(servo_table)

      {:noreply, state} = Actuator.handle_command(velocity_command(1.0), state)

      assert state.expiry_timer == nil
      refute_receive {:expire_command, _}, 50
    end
  end
end
