# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Servo.Robotis.ControllerTest do
  use ExUnit.Case, async: true
  use Mimic

  alias BB.Servo.Robotis.Controller

  @controller_name :test_dynamixel

  defp default_bb_context do
    %{robot: TestRobot, path: [@controller_name], name: @controller_name}
  end

  defp stub_robotis_success do
    stub(Robotis, :start_link, fn _opts -> {:ok, self()} end)
    stub(BB.Safety, :register, fn _module, _opts -> :ok end)
    stub(BB, :subscribe, fn _robot, _path -> :ok end)
  end

  defp init_controller(extra_opts \\ []) do
    opts = [bb: default_bb_context(), port: "/dev/ttyUSB0"] ++ extra_opts
    {:ok, state} = Controller.init(opts)
    state
  end

  defp pending_write(state, servo_id) do
    [{^servo_id, _, _, _, _, _, _, _, _, pending_write, _}] =
      :ets.lookup(state.servo_table, servo_id)

    pending_write
  end

  defp torque_enabled(state, servo_id) do
    [{^servo_id, _, _, _, _, _, _, _, _, _, torque_enabled}] =
      :ets.lookup(state.servo_table, servo_id)

    torque_enabled
  end

  describe "init/1" do
    test "succeeds with valid options" do
      stub_robotis_success()

      opts = [
        bb: default_bb_context(),
        port: "/dev/ttyUSB0",
        baud_rate: 1_000_000,
        control_table: Robotis.ControlTable.XM430
      ]

      assert {:ok, state} = Controller.init(opts)

      assert state.control_table == Robotis.ControlTable.XM430
      assert state.name == @controller_name
      assert is_reference(state.servo_table)
    end

    test "uses default baud rate and control table" do
      stub_robotis_success()

      opts = [bb: default_bb_context(), port: "/dev/ttyUSB0"]

      assert {:ok, state} = Controller.init(opts)

      assert state.control_table == Robotis.ControlTable.XM430
    end

    test "starts Robotis GenServer with correct options" do
      test_pid = self()

      expect(Robotis, :start_link, fn opts ->
        send(test_pid, {:robotis_opts, opts})
        {:ok, spawn(fn -> :timer.sleep(:infinity) end)}
      end)

      stub(BB.Safety, :register, fn _module, _opts -> :ok end)
      stub(BB, :subscribe, fn _robot, _path -> :ok end)

      opts = [
        bb: default_bb_context(),
        port: "/dev/ttyUSB0",
        baud_rate: 115_200,
        control_table: :xl330_m288
      ]

      {:ok, _state} = Controller.init(opts)

      assert_receive {:robotis_opts, robotis_opts}
      assert robotis_opts[:uart_port] == "/dev/ttyUSB0"
      assert robotis_opts[:baud] == 115_200
      assert robotis_opts[:control_table] == :xl330_m288
    end

    test "registers with safety system" do
      stub(Robotis, :start_link, fn _opts -> {:ok, self()} end)
      stub(BB, :subscribe, fn _robot, _path -> :ok end)

      test_pid = self()

      expect(BB.Safety, :register, fn module, opts ->
        send(test_pid, {:safety_registered, module, opts})
        :ok
      end)

      opts = [bb: default_bb_context(), port: "/dev/ttyUSB0"]

      {:ok, _state} = Controller.init(opts)

      assert_receive {:safety_registered, Controller, safety_opts}
      assert safety_opts[:robot] == TestRobot
      assert safety_opts[:path] == [@controller_name]
    end

    test "raises when port not provided" do
      opts = [bb: default_bb_context()]
      assert_raise KeyError, fn -> Controller.init(opts) end
    end

    test "computes status_every_n_ticks from intervals" do
      stub_robotis_success()

      assert {:ok, state} =
               Controller.init(
                 bb: default_bb_context(),
                 port: "/dev/ttyUSB0",
                 loop_interval_ms: 10,
                 status_poll_interval_ms: 1000
               )

      assert state.status_every_n_ticks == 100

      assert {:ok, state} =
               Controller.init(
                 bb: default_bb_context(),
                 port: "/dev/ttyUSB0",
                 loop_interval_ms: 10,
                 status_poll_interval_ms: 0
               )

      assert state.status_every_n_ticks == 0
    end
  end

  describe "handle_call/3" do
    setup do
      robotis_pid = spawn(fn -> :timer.sleep(:infinity) end)
      stub(Robotis, :start_link, fn _opts -> {:ok, robotis_pid} end)
      stub(BB.Safety, :register, fn _module, _opts -> :ok end)
      stub(BB, :subscribe, fn _robot, _path -> :ok end)

      state = init_controller()

      {:ok, state: state, robotis_pid: robotis_pid}
    end

    test "forwards read to Robotis", %{state: state, robotis_pid: robotis_pid} do
      expect(Robotis, :read, fn ^robotis_pid, 1, :present_position ->
        {:ok, 2048}
      end)

      assert {:reply, {:ok, 2048}, _new_state} =
               Controller.handle_call({:read, 1, :present_position}, self(), state)
    end

    test "forwards write to Robotis with await", %{state: state, robotis_pid: robotis_pid} do
      expect(Robotis, :write, fn ^robotis_pid, 1, :goal_position, 2048, true ->
        :ok
      end)

      assert {:reply, :ok, _new_state} =
               Controller.handle_call({:write, 1, :goal_position, 2048}, self(), state)
    end

    test "forwards ping to Robotis", %{state: state, robotis_pid: robotis_pid} do
      expect(Robotis, :ping, fn ^robotis_pid, 1 ->
        {:ok, %{model: 1020}}
      end)

      assert {:reply, {:ok, %{model: 1020}}, _state} =
               Controller.handle_call({:ping, 1}, self(), state)
    end

    test "forwards ping_all to Robotis", %{state: state, robotis_pid: robotis_pid} do
      expect(Robotis, :ping, fn ^robotis_pid ->
        [{:ok, 1, %{model: 1020}}]
      end)

      assert {:reply, [{:ok, 1, %{model: 1020}}], _state} =
               Controller.handle_call(:ping_all, self(), state)
    end

    test "list_servos returns empty list initially", %{state: state} do
      assert {:reply, {:ok, []}, _state} =
               Controller.handle_call(:list_servos, self(), state)
    end

    test "get_control_table returns the control table", %{state: state} do
      assert {:reply, {:ok, Robotis.ControlTable.XM430}, _state} =
               Controller.handle_call(:get_control_table, self(), state)
    end

    test "register_servo inserts into ETS and returns table ref", %{state: state} do
      stub(BB.Safety, :register, fn _module, _opts -> :ok end)

      {:reply, {:ok, table}, state} =
        Controller.handle_call({:register_servo, 1, [:joint1, :servo], 2}, self(), state)

      assert is_reference(table)
      assert state.servo_ids == [1]

      [{1, actuator_path, deadband, last_pos, _, _, _, _, _, goal, torque_enabled}] =
        :ets.lookup(table, 1)

      assert actuator_path == [:joint1, :servo]
      assert deadband == 2
      assert last_pos == nil
      assert goal == nil
      assert torque_enabled == false
    end

    test "list_servos returns registered servo IDs", %{state: state} do
      stub(BB.Safety, :register, fn _module, _opts -> :ok end)

      {:reply, {:ok, _table}, state} =
        Controller.handle_call({:register_servo, 1, [:joint1, :servo], 2}, self(), state)

      {:reply, {:ok, _table}, state} =
        Controller.handle_call({:register_servo, 2, [:joint2, :servo], 2}, self(), state)

      assert {:reply, {:ok, servo_ids}, _state} =
               Controller.handle_call(:list_servos, self(), state)

      assert servo_ids == [1, 2]
    end
  end

  describe "resume_servo" do
    setup do
      robotis_pid = spawn(fn -> :timer.sleep(:infinity) end)
      stub(Robotis, :start_link, fn _opts -> {:ok, robotis_pid} end)
      stub(BB.Safety, :register, fn _module, _opts -> :ok end)
      stub(BB, :subscribe, fn _robot, _path -> :ok end)

      {:reply, {:ok, _table}, state} =
        Controller.handle_call(
          {:register_servo, 1, [:joint1, :servo], 2},
          self(),
          init_controller()
        )

      {:ok, state: state, robotis_pid: robotis_pid}
    end

    test "lands the goal before torque comes back on", %{state: state} do
      test_pid = self()

      stub(Robotis, :write_raw, fn _pid, 1, :goal_position, 2700, true ->
        send(test_pid, {:bus, :goal})
        :ok
      end)

      stub(Robotis, :write, fn _pid, 1, :torque_enable, true, true ->
        send(test_pid, {:bus, :torque})
        :ok
      end)

      assert {:reply, :ok, _state} =
               Controller.handle_call({:resume_servo, 1, {:goal_position, 2700}}, self(), state)

      # Unbound matches take the oldest message, so this asserts the order: the
      # goal must be acknowledged first, or the servo lunges for a stale one.
      assert_receive {:bus, first}
      assert_receive {:bus, second}
      assert [first, second] == [:goal, :torque]
    end

    test "records torque on and spends the pending goal", %{state: state} do
      :ets.update_element(state.servo_table, 1, [{10, {:goal_position, 1396}}])

      stub(Robotis, :write_raw, fn _pid, 1, :goal_position, 2700, true -> :ok end)
      stub(Robotis, :write, fn _pid, 1, :torque_enable, true, true -> :ok end)

      {:reply, :ok, _state} =
        Controller.handle_call({:resume_servo, 1, {:goal_position, 2700}}, self(), state)

      assert pending_write(state, 1) == nil
      assert torque_enabled(state, 1) == true
    end

    test "holds station at the present position when asked for one", %{state: state} do
      test_pid = self()

      stub(Robotis, :read_raw, fn _pid, 1, :present_position -> {:ok, 1800} end)

      stub(Robotis, :write_raw, fn _pid, 1, :goal_position, goal, true ->
        send(test_pid, {:goal, goal})
        :ok
      end)

      stub(Robotis, :write, fn _pid, 1, :torque_enable, true, true -> :ok end)

      assert {:reply, :ok, _state} =
               Controller.handle_call({:resume_servo, 1, :present_position}, self(), state)

      assert_receive {:goal, 1800}
    end

    test "leaves torque off when the goal write is refused", %{state: state} do
      stub(Robotis, :write_raw, fn _pid, 1, :goal_position, _goal, true -> {:error, :timeout} end)

      assert {:reply, {:error, :timeout}, _state} =
               Controller.handle_call({:resume_servo, 1, {:goal_position, 2700}}, self(), state)

      assert torque_enabled(state, 1) == false
    end

    test "reports a failed present-position read", %{state: state} do
      stub(Robotis, :read_raw, fn _pid, 1, :present_position -> {:error, :timeout} end)
      reject(&Robotis.write_raw/5)

      assert {:reply, {:error, :timeout}, _state} =
               Controller.handle_call({:resume_servo, 1, :present_position}, self(), state)
    end
  end

  describe "torque state cache" do
    setup do
      robotis_pid = spawn(fn -> :timer.sleep(:infinity) end)
      stub(Robotis, :start_link, fn _opts -> {:ok, robotis_pid} end)
      stub(BB.Safety, :register, fn _module, _opts -> :ok end)
      stub(BB, :subscribe, fn _robot, _path -> :ok end)

      {:reply, {:ok, _table}, state} =
        Controller.handle_call(
          {:register_servo, 1, [:joint1, :servo], 2},
          self(),
          init_controller()
        )

      {:ok, state: state}
    end

    test "follows torque written through the controller", %{state: state} do
      stub(Robotis, :write, fn _pid, 1, :torque_enable, _value, true -> :ok end)

      Controller.handle_call({:write, 1, :torque_enable, true}, self(), state)
      assert torque_enabled(state, 1) == true

      Controller.handle_call({:write, 1, :torque_enable, false}, self(), state)
      assert torque_enabled(state, 1) == false
    end

    test "follows raw torque writes", %{state: state} do
      stub(Robotis, :write_raw, fn _pid, 1, :torque_enable, _value, true -> :ok end)

      Controller.handle_call({:write_raw, 1, :torque_enable, 1}, self(), state)
      assert torque_enabled(state, 1) == true

      Controller.handle_call({:write_raw, 1, :torque_enable, 0}, self(), state)
      assert torque_enabled(state, 1) == false
    end

    test "is left alone when the write is refused", %{state: state} do
      stub(Robotis, :write, fn _pid, 1, :torque_enable, _value, true -> {:error, :timeout} end)

      assert {:reply, {:error, :timeout}, _state} =
               Controller.handle_call({:write, 1, :torque_enable, true}, self(), state)

      assert torque_enabled(state, 1) == false
    end

    test "records torque on for every servo when the robot arms", %{state: state} do
      stub(Robotis, :fast_sync_read, fn _pid, [1], :present_position -> [{1, {:ok, 180.0}}] end)
      stub(Robotis, :read_raw, fn _pid, 1, :present_position -> {:ok, 2048} end)
      stub(Robotis, :write_raw, fn _pid, 1, :goal_position, 2048, false -> :ok end)
      stub(Robotis, :sync_write, fn _pid, :torque_enable, [{1, true}] -> :ok end)

      armed = %BB.Message{payload: %BB.StateMachine.Transition{from: :disarmed, to: :armed}}

      assert {:noreply, _state} =
               Controller.handle_info({:bb, [:state_machine], armed}, state)

      assert torque_enabled(state, 1) == true
    end
  end

  describe "tick loop" do
    setup do
      robotis_pid = spawn(fn -> :timer.sleep(:infinity) end)
      stub(Robotis, :start_link, fn _opts -> {:ok, robotis_pid} end)
      stub(BB.Safety, :register, fn _module, _opts -> :ok end)
      stub(BB, :subscribe, fn _robot, _path -> :ok end)
      stub(BB.Actuator, :to_joint_space, fn _robot, _path, msg -> msg end)

      state = init_controller()

      {:reply, {:ok, _table}, state} =
        Controller.handle_call({:register_servo, 1, [:joint1, :servo], 2}, self(), state)

      {:ok, state: state, robotis_pid: robotis_pid}
    end

    test "processes pending commands from ETS", %{state: state, robotis_pid: robotis_pid} do
      :ets.update_element(state.servo_table, 1, [{10, {:goal_position, 2048}}])

      expect(Robotis, :write_raw, fn ^robotis_pid, 1, :goal_position, 2048, false -> :ok end)
      stub(Robotis, :fast_sync_read, fn _pid, _ids, _param -> [{1, {:ok, 180.0}}] end)
      stub(BB, :publish, fn _robot, _path, _msg -> :ok end)

      assert {:noreply, _state} = Controller.handle_info(:tick, state)

      [{1, _, _, _, _, _, _, _, _, goal, _}] = :ets.lookup(state.servo_table, 1)
      assert goal == nil
    end

    test "skips commands when no goals pending", %{state: state} do
      reject(&Robotis.write_raw/5)
      stub(Robotis, :fast_sync_read, fn _pid, _ids, _param -> [{1, {:ok, 180.0}}] end)
      stub(BB, :publish, fn _robot, _path, _msg -> :ok end)

      assert {:noreply, _state} = Controller.handle_info(:tick, state)
    end

    test "reads positions and publishes joint state", %{state: state} do
      test_pid = self()

      stub(Robotis, :fast_sync_read, fn _pid, [1], :present_position ->
        [{1, {:ok, 190.0}}]
      end)

      expect(BB, :publish, fn TestRobot, [:sensor, @controller_name, :joint1], msg ->
        send(test_pid, {:published, msg})
        :ok
      end)

      assert {:noreply, _state} = Controller.handle_info(:tick, state)

      assert_receive {:published, %BB.Message{payload: %BB.Message.Sensor.JointState{}}}
    end

    test "updates present_position in ETS", %{state: state} do
      stub(Robotis, :fast_sync_read, fn _pid, _ids, _param -> [{1, {:ok, 195.0}}] end)
      stub(BB, :publish, fn _robot, _path, _msg -> :ok end)

      Controller.handle_info(:tick, state)

      [{1, _, _, _, present_pos, _, _, _, _, _, _}] = :ets.lookup(state.servo_table, 1)
      assert present_pos == 195.0
    end
  end

  describe "disarm/1" do
    test "disables torque on all registered servos with acknowledged writes" do
      test_pid = self()
      robotis_pid = spawn(fn -> :timer.sleep(:infinity) end)

      expect(Robotis, :write, 3, fn ^robotis_pid, id, :torque_enable, false, true ->
        send(test_pid, {:write, id})
        :ok
      end)

      opts = [robotis: robotis_pid, servo_ids: [1, 2, 3]]
      assert :ok = Controller.disarm(opts)

      assert_receive {:write, 1}
      assert_receive {:write, 2}
      assert_receive {:write, 3}
    end

    test "returns ok when no servos registered" do
      robotis_pid = spawn(fn -> :timer.sleep(:infinity) end)
      reject(&Robotis.write/5)
      opts = [robotis: robotis_pid, servo_ids: []]
      assert :ok = Controller.disarm(opts)
    end

    test "returns an error when a torque-disable write is rejected" do
      robotis_pid = spawn(fn -> :timer.sleep(:infinity) end)

      stub(Robotis, :write, fn ^robotis_pid, id, :torque_enable, false, true ->
        if id == 2, do: {:error, :timeout}, else: :ok
      end)

      opts = [robotis: robotis_pid, servo_ids: [1, 2, 3]]
      assert {:error, {:servo, 2, :torque_enable, :timeout}} = Controller.disarm(opts)
    end

    test "returns an error when the process is not alive" do
      dead_pid = spawn(fn -> :ok end)
      Process.sleep(10)
      opts = [robotis: dead_pid, servo_ids: [1]]
      assert {:error, {:exit, _reason}} = Controller.disarm(opts)
    end
  end

  describe "hardware error reporting" do
    test "reports error to safety system when hardware error detected" do
      test_pid = self()

      stub(Robotis, :start_link, fn _opts -> {:ok, self()} end)
      stub(BB.Safety, :register, fn _module, _opts -> :ok end)
      stub(BB, :subscribe, fn _robot, _path -> :ok end)

      expect(BB.Safety, :report_error, fn robot, path, error ->
        send(test_pid, {:error_reported, robot, path, error})
        :ok
      end)

      state = init_controller()

      {:reply, {:ok, _table}, state} =
        Controller.handle_call({:register_servo, 1, [:joint1, :servo], 2}, self(), state)

      alias BB.Error.Protocol.Robotis.HardwareAlert

      status = %{temperature: 45.0, voltage: 12.0, current: 0.5, hardware_error: 0x04}

      new_last_status =
        Enum.reduce([1], state.last_status, fn servo_id, acc ->
          last = Map.get(acc, servo_id)

          if is_nil(last) or status.hardware_error != Map.get(last, :hardware_error) do
            path = state.bb.path ++ [:servo, servo_id]
            alert = HardwareAlert.from_bits(servo_id, status.hardware_error)
            BB.Safety.report_error(state.bb.robot, path, alert)
            Map.put(acc, servo_id, status)
          else
            acc
          end
        end)

      assert new_last_status[1] == status

      assert_receive {:error_reported, TestRobot, [:test_dynamixel, :servo, 1],
                      %HardwareAlert{servo_id: 1, alerts: [:overheating], raw_value: 0x04}}
    end
  end
end
