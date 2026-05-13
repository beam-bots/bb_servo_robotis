# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule Mix.Tasks.BbServoRobotis.InstallTest do
  use ExUnit.Case
  import Igniter.Test

  @moduletag :igniter

  defp project_with_robot do
    test_project()
    |> Igniter.compose_task("bb.install")
    |> apply_igniter!()
  end

  describe "controller" do
    test "uses param refs for port and baud_rate" do
      project_with_robot()
      |> Igniter.compose_task("bb_servo_robotis.install")
      |> assert_has_patch("lib/test/robot.ex", """
      + |    controller(
      + |      :dynamixel,
      + |      {BB.Servo.Robotis.Controller,
      + |       port: param([:config, :robotis, :device]),
      + |       baud_rate: param([:config, :robotis, :baud_rate]),
      + |       control_table: Robotis.ControlTable.XM430}
      + |    )
      """)
    end

    test "uses a custom controller name when --name is given" do
      project_with_robot()
      |> Igniter.compose_task("bb_servo_robotis.install", ["--name", "u2d2"])
      |> assert_has_patch("lib/test/robot.ex", """
      + |    controller(
      + |      :u2d2,
      """)
    end
  end

  describe "bridge" do
    test "adds a parameter bridge wired to the controller" do
      project_with_robot()
      |> Igniter.compose_task("bb_servo_robotis.install")
      |> assert_has_patch("lib/test/robot.ex", """
      + |    bridge(:robotis_bridge, {BB.Servo.Robotis.Bridge, controller: :dynamixel})
      """)
    end

    test "uses the custom controller name in the bridge" do
      project_with_robot()
      |> Igniter.compose_task("bb_servo_robotis.install", [
        "--name",
        "u2d2",
        "--bridge-name",
        "u2d2_bridge"
      ])
      |> assert_has_patch("lib/test/robot.ex", """
      + |    bridge(:u2d2_bridge, {BB.Servo.Robotis.Bridge, controller: :u2d2})
      """)
    end
  end

  describe "parameters group" do
    test "adds a :config.:robotis param group with device and baud_rate" do
      project_with_robot()
      |> Igniter.compose_task("bb_servo_robotis.install")
      |> assert_has_patch("lib/test/robot.ex", """
      + |    group :config do
      + |      group :robotis do
      + |        param(:device, type: :string, doc: "Serial device connected to the Robotis controller")
      """)
    end

    test "adds the baud_rate param" do
      project_with_robot()
      |> Igniter.compose_task("bb_servo_robotis.install")
      |> assert_has_patch("lib/test/robot.ex", """
      + |        param(:baud_rate,
      + |          type: :integer,
      + |          default: 1_000_000,
      + |          doc: "Communications speed for the serial port"
      + |        )
      """)
    end
  end

  describe "application module" do
    test "sets the device path on the robot child spec" do
      project_with_robot()
      |> Igniter.compose_task("bb_servo_robotis.install")
      |> assert_has_patch("lib/test/application.ex", ~s'''
      + |    children = [{Test.Robot, [params: [config: [robotis: [device: "/dev/ttyUSB0"]]]]}]
      ''')
    end

    test "honours a custom --device option" do
      project_with_robot()
      |> Igniter.compose_task("bb_servo_robotis.install", ["--device", "/dev/ttyACM0"])
      |> assert_has_patch("lib/test/application.ex", ~s'''
      + |    children = [{Test.Robot, [params: [config: [robotis: [device: "/dev/ttyACM0"]]]]}]
      ''')
    end
  end

  describe "formatter" do
    test "imports bb_servo_robotis into .formatter.exs" do
      project_with_robot()
      |> Igniter.compose_task("bb_servo_robotis.install")
      |> assert_has_patch(".formatter.exs", """
      + |  import_deps: [:bb_servo_robotis, :bb]
      """)
    end
  end

  describe "notice" do
    test "prints a topology snippet for the user to paste" do
      project_with_robot()
      |> Igniter.compose_task("bb_servo_robotis.install")
      |> assert_has_notice(&String.contains?(&1, "BB.Servo.Robotis.Actuator"))
    end
  end

  describe "idempotency" do
    test "running twice produces no further changes" do
      project_with_robot()
      |> Igniter.compose_task("bb_servo_robotis.install")
      |> apply_igniter!()
      |> Igniter.compose_task("bb_servo_robotis.install")
      |> assert_unchanged()
    end
  end

  describe "robot selection" do
    test "targets a specific robot module via --robot" do
      test_project()
      |> Igniter.compose_task("bb.add_robot", ["--robot", "Test.Arms.Left"])
      |> apply_igniter!()
      |> Igniter.compose_task("bb_servo_robotis.install", ["--robot", "Test.Arms.Left"])
      |> assert_has_patch("lib/test/arms/left.ex", """
      + |    controller(
      + |      :dynamixel,
      """)
    end
  end
end
