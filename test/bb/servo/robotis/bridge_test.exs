# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Servo.Robotis.BridgeTest do
  use ExUnit.Case, async: true
  use Mimic

  alias BB.Servo.Robotis.Bridge

  @bridge_name :robotis
  @controller_name :dynamixel

  defp default_bb_context do
    %{robot: TestRobot, path: [@bridge_name], name: @bridge_name}
  end

  defp stub_controller_queries do
    stub(BB.Process, :call, fn
      TestRobot, @controller_name, :get_control_table -> {:ok, :xm430}
      TestRobot, @controller_name, :list_servos -> {:ok, [1, 2]}
      TestRobot, @controller_name, {:read, 1, :position_p_gain} -> {:ok, 800}
      TestRobot, @controller_name, {:read, 1, :torque_enable} -> {:ok, false}
      TestRobot, @controller_name, {:write, 1, :position_p_gain, 1000} -> :ok
    end)
  end

  describe "init/1" do
    test "succeeds with valid options" do
      stub_controller_queries()

      opts = [
        bb: default_bb_context(),
        controller: @controller_name
      ]

      assert {:ok, state} = Bridge.init(opts)
      assert state.robot == TestRobot
      assert state.controller == @controller_name
      assert state.control_table == :xm430
    end
  end

  describe "list_remote/1" do
    test "returns parameters for all registered servos" do
      stub_controller_queries()

      opts = [bb: default_bb_context(), controller: @controller_name]
      {:ok, state} = Bridge.init(opts)

      assert {:ok, params, _state} = Bridge.list_remote(state)

      assert is_list(params)
      refute Enum.empty?(params)

      # Check that we have params for both servos
      servo_1_params = Enum.filter(params, fn p -> String.starts_with?(p.id, "1:") end)
      servo_2_params = Enum.filter(params, fn p -> String.starts_with?(p.id, "2:") end)

      refute Enum.empty?(servo_1_params)
      refute Enum.empty?(servo_2_params)
      assert Enum.count(servo_1_params) == Enum.count(servo_2_params)
    end

    test "includes parameter metadata" do
      stub_controller_queries()

      opts = [bb: default_bb_context(), controller: @controller_name]
      {:ok, state} = Bridge.init(opts)

      assert {:ok, params, _state} = Bridge.list_remote(state)

      p_gain = Enum.find(params, fn p -> p.id == "1:position_p_gain" end)

      assert p_gain != nil
      assert p_gain.writable == true
      assert p_gain.category == :control
      assert p_gain.path == [:robotis, :servo_1, :position_p_gain]
    end

    test "excludes status parameters" do
      stub_controller_queries()

      opts = [bb: default_bb_context(), controller: @controller_name]
      {:ok, state} = Bridge.init(opts)

      assert {:ok, params, _state} = Bridge.list_remote(state)

      temp_param = Enum.find(params, fn p -> String.ends_with?(p.id, ":present_temperature") end)
      assert temp_param == nil
    end
  end

  describe "get_remote/2" do
    test "reads parameter via controller" do
      stub_controller_queries()

      opts = [bb: default_bb_context(), controller: @controller_name]
      {:ok, state} = Bridge.init(opts)

      assert {:ok, 800, _state} = Bridge.get_remote("1:position_p_gain", state)
    end

    test "returns error for invalid param_id format" do
      stub_controller_queries()

      opts = [bb: default_bb_context(), controller: @controller_name]
      {:ok, state} = Bridge.init(opts)

      assert {:error, {:invalid_param_id, "invalid"}, _state} =
               Bridge.get_remote("invalid", state)

      assert {:error, {:invalid_param_id, "abc:param"}, _state} =
               Bridge.get_remote("abc:param", state)
    end

    test "returns error for unknown param" do
      stub_controller_queries()

      opts = [bb: default_bb_context(), controller: @controller_name]
      {:ok, state} = Bridge.init(opts)

      assert {:error, {:unknown_param, :not_a_param}, _state} =
               Bridge.get_remote("1:not_a_param", state)
    end
  end

  describe "set_remote/3" do
    test "writes parameter via controller" do
      stub_controller_queries()

      opts = [bb: default_bb_context(), controller: @controller_name]
      {:ok, state} = Bridge.init(opts)

      assert {:ok, _state} = Bridge.set_remote("1:position_p_gain", 1000, state)
    end

    test "returns error for read-only parameter" do
      stub_controller_queries()

      opts = [bb: default_bb_context(), controller: @controller_name]
      {:ok, state} = Bridge.init(opts)

      assert {:error, {:read_only, :model_number}, _state} =
               Bridge.set_remote("1:model_number", 123, state)
    end

    test "returns error when writing config param with torque enabled" do
      stub(BB.Process, :call, fn
        TestRobot, @controller_name, :get_control_table -> {:ok, :xm430}
        TestRobot, @controller_name, :list_servos -> {:ok, [1]}
        TestRobot, @controller_name, {:read, 1, :torque_enable} -> {:ok, true}
      end)

      opts = [bb: default_bb_context(), controller: @controller_name]
      {:ok, state} = Bridge.init(opts)

      assert {:error, {:torque_must_be_disabled, :velocity_limit}, _state} =
               Bridge.set_remote("1:velocity_limit", 100, state)
    end
  end

  describe "handle_change/3" do
    test "returns ok (no-op for inbound bridge)" do
      stub_controller_queries()

      opts = [bb: default_bb_context(), controller: @controller_name]
      {:ok, state} = Bridge.init(opts)

      assert {:ok, ^state} = Bridge.handle_change(TestRobot, %{}, state)
    end
  end
end
