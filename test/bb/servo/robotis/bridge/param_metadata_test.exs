# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Servo.Robotis.Bridge.ParamMetadataTest do
  use ExUnit.Case, async: true

  alias BB.Servo.Robotis.Bridge.ParamMetadata

  describe "list_params/1" do
    test "returns sorted list of params for xm430" do
      params = ParamMetadata.list_params(Robotis.ControlTable.XM430)

      assert is_list(params)
      refute Enum.empty?(params)
      assert params == Enum.sort(params)
    end

    test "includes info, config, and control params" do
      params = ParamMetadata.list_params(Robotis.ControlTable.XM430)

      assert :model_number in params
      assert :firmware_version in params
      assert :velocity_limit in params
      assert :position_p_gain in params
    end

    test "excludes status params" do
      params = ParamMetadata.list_params(Robotis.ControlTable.XM430)

      refute :present_position in params
      refute :present_temperature in params
      refute :present_current in params
      refute :hardware_error_status in params
    end

    test "excludes indirect addressing params" do
      params = ParamMetadata.list_params(Robotis.ControlTable.XM430)

      refute :indirect_address_1 in params
      refute :indirect_data_1 in params
    end

    test "includes xl320-specific params for xl320" do
      params = ParamMetadata.list_params(:xl320)

      assert :cw_angle_limit in params
      assert :ccw_angle_limit in params
      assert :d_gain in params
    end
  end

  describe "param_info/2" do
    test "returns info category for model_number" do
      assert {:ok, info} = ParamMetadata.param_info(Robotis.ControlTable.XM430, :model_number)

      assert info.category == :info
      assert info.writable == false
      assert info.requires_torque_off == false
    end

    test "returns config category for velocity_limit" do
      assert {:ok, info} = ParamMetadata.param_info(Robotis.ControlTable.XM430, :velocity_limit)

      assert info.category == :config
      assert info.writable == true
      assert info.requires_torque_off == true
    end

    test "returns control category for position_p_gain" do
      assert {:ok, info} = ParamMetadata.param_info(Robotis.ControlTable.XM430, :position_p_gain)

      assert info.category == :control
      assert info.writable == true
      assert info.requires_torque_off == false
    end

    test "returns error for unknown param" do
      assert {:error, :unknown_param} =
               ParamMetadata.param_info(Robotis.ControlTable.XM430, :not_a_real_param)
    end

    test "returns error for status param" do
      assert {:error, :unknown_param} =
               ParamMetadata.param_info(Robotis.ControlTable.XM430, :present_temperature)
    end
  end

  describe "writable?/2" do
    test "returns false for info params" do
      refute ParamMetadata.writable?(Robotis.ControlTable.XM430, :model_number)
      refute ParamMetadata.writable?(Robotis.ControlTable.XM430, :firmware_version)
    end

    test "returns true for config params" do
      assert ParamMetadata.writable?(Robotis.ControlTable.XM430, :velocity_limit)
      assert ParamMetadata.writable?(Robotis.ControlTable.XM430, :operating_mode)
    end

    test "returns true for control params" do
      assert ParamMetadata.writable?(Robotis.ControlTable.XM430, :position_p_gain)
      assert ParamMetadata.writable?(Robotis.ControlTable.XM430, :goal_position)
    end

    test "returns false for unknown params" do
      refute ParamMetadata.writable?(Robotis.ControlTable.XM430, :not_a_real_param)
    end
  end

  describe "requires_torque_off?/2" do
    test "returns false for control params" do
      refute ParamMetadata.requires_torque_off?(Robotis.ControlTable.XM430, :position_p_gain)
      refute ParamMetadata.requires_torque_off?(Robotis.ControlTable.XM430, :goal_position)
    end

    test "returns true for config params" do
      assert ParamMetadata.requires_torque_off?(Robotis.ControlTable.XM430, :velocity_limit)
      assert ParamMetadata.requires_torque_off?(Robotis.ControlTable.XM430, :operating_mode)
    end

    test "returns false for info params" do
      refute ParamMetadata.requires_torque_off?(Robotis.ControlTable.XM430, :model_number)
    end
  end

  describe "status_param?/1" do
    test "returns true for status params" do
      assert ParamMetadata.status_param?(:present_temperature)
      assert ParamMetadata.status_param?(:present_position)
      assert ParamMetadata.status_param?(:hardware_error_status)
    end

    test "returns true for indirect addressing params" do
      assert ParamMetadata.status_param?(:indirect_address_1)
      assert ParamMetadata.status_param?(:indirect_data_5)
    end

    test "returns false for non-status params" do
      refute ParamMetadata.status_param?(:position_p_gain)
      refute ParamMetadata.status_param?(:velocity_limit)
      refute ParamMetadata.status_param?(:model_number)
    end
  end
end
