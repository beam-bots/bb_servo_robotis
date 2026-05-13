# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

if Code.ensure_loaded?(Igniter) do
  defmodule Mix.Tasks.BbServoRobotis.Install do
    @shortdoc "Installs BB.Servo.Robotis into a robot"
    @moduledoc """
    #{@shortdoc}

    Adds a `BB.Servo.Robotis.Controller` and a `BB.Servo.Robotis.Bridge` to your
    robot module and imports the package's formatter rules.

    Actuator and sensor entries belong on individual joints in your topology and
    are not added automatically — a snippet is printed for you to copy.

    ## Example

    ```bash
    mix igniter.install bb_servo_robotis
    mix igniter.install bb_servo_robotis --robot MyApp.Arm
    mix igniter.install bb_servo_robotis --name u2d2 --bridge-name u2d2_bridge
    ```

    ## Options

    * `--robot` - The robot module (defaults to `{AppPrefix}.Robot`).
    * `--name` - The controller name (default `dynamixel`).
    * `--bridge-name` - The parameter bridge name (default `robotis_bridge`).
    """

    use Igniter.Mix.Task

    alias Igniter.Project.Formatter

    @param_group :robotis
    @default_device "/dev/ttyUSB0"

    @impl Igniter.Mix.Task
    def info(_argv, _parent) do
      %Igniter.Mix.Task.Info{
        schema: [
          robot: :string,
          name: :string,
          bridge_name: :string,
          device: :string
        ],
        aliases: [r: :robot, n: :name]
      }
    end

    @impl Igniter.Mix.Task
    def igniter(igniter) do
      options = igniter.args.options
      robot_module = BB.Igniter.robot_module(igniter)
      name = options |> Keyword.get(:name, "dynamixel") |> String.to_atom()
      bridge_name = options |> Keyword.get(:bridge_name, "robotis_bridge") |> String.to_atom()
      device = Keyword.get(options, :device, @default_device)

      igniter
      |> Formatter.import_dep(:bb_servo_robotis)
      |> BB.Igniter.add_controller(robot_module, name, controller_code(name))
      |> BB.Igniter.add_parameter_bridge(
        robot_module,
        bridge_name,
        bridge_code(bridge_name, name)
      )
      |> BB.Igniter.add_param_group(robot_module, [:config, @param_group], param_group_body())
      |> BB.Igniter.set_robot_opts(robot_module,
        params: [config: [{@param_group, [device: device]}]]
      )
      |> Igniter.add_notice(topology_snippet(name))
    end

    defp controller_code(name) do
      """
      controller :#{name}, {BB.Servo.Robotis.Controller,
        port: param([:config, :#{@param_group}, :device]),
        baud_rate: param([:config, :#{@param_group}, :baud_rate]),
        control_table: Robotis.ControlTable.XM430}
      """
    end

    defp bridge_code(bridge_name, controller_name) do
      "bridge :#{bridge_name}, {BB.Servo.Robotis.Bridge, controller: :#{controller_name}}\n"
    end

    defp param_group_body do
      """
      param :device, type: :string, doc: "Serial device connected to the Robotis controller"

      param :baud_rate,
        type: :integer,
        default: 1_000_000,
        doc: "Communications speed for the serial port"
      """
    end

    defp topology_snippet(controller_name) do
      """
      bb_servo_robotis: add servo actuators/sensors to your joints. Example:

          joint :shoulder, type: :revolute do
            limit lower: ~u(-90 degree), upper: ~u(90 degree), velocity: ~u(60 degree_per_second)

            actuator :servo, {BB.Servo.Robotis.Actuator,
              servo_id: 1,
              controller: :#{controller_name}}

            sensor :position, {BB.Servo.Robotis.Sensor,
              servo_id: 1,
              controller: :#{controller_name}}
          end
      """
    end
  end
else
  defmodule Mix.Tasks.BbServoRobotis.Install do
    @shortdoc "Installs BB.Servo.Robotis into a robot"
    @moduledoc false
    use Mix.Task

    def run(_argv) do
      Mix.shell().error("""
      The bb_servo_robotis.install task requires igniter.

          mix igniter.install bb_servo_robotis
      """)

      exit({:shutdown, 1})
    end
  end
end
