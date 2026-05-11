#!/usr/bin/env python3
"""Load a Franka end-effector JSON profile into ROS 2 parameters."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Dict, List

import rclpy
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.utilities import remove_ros_args


def _number(data: Dict[str, Any], key: str, default: float = 0.0) -> float:
    return float(data.get(key, default))


def _profile_to_parameters(profile: Dict[str, Any]) -> Dict[str, Any]:
    inertial = profile.get("inertial", {})
    center_of_mass = inertial.get("centerOfMass", {})
    inertia = inertial.get("inertia", {})
    tcp = profile.get("tcp", {})
    tcp_translation = tcp.get("translation", {})
    tcp_rotation = tcp.get("rotation", {})

    return {
        "end_effector_profile.id": str(profile.get("id", "")),
        "end_effector_profile.name": str(profile.get("name", "")),
        "end_effector_profile.device_id": str(profile.get("deviceId", "")),
        "end_effector_profile.load.mass": _number(inertial, "mass"),
        "end_effector_profile.load.center_of_mass": [
            _number(center_of_mass, "x"),
            _number(center_of_mass, "y"),
            _number(center_of_mass, "z"),
        ],
        "end_effector_profile.load.inertia": [
            _number(inertia, "x11"),
            _number(inertia, "x12"),
            _number(inertia, "x13"),
            _number(inertia, "x12"),
            _number(inertia, "x22"),
            _number(inertia, "x23"),
            _number(inertia, "x13"),
            _number(inertia, "x23"),
            _number(inertia, "x33"),
        ],
        "end_effector_profile.tcp.translation": [
            _number(tcp_translation, "x"),
            _number(tcp_translation, "y"),
            _number(tcp_translation, "z"),
        ],
        "end_effector_profile.tcp.rotation_rpy": [
            _number(tcp_rotation, "roll"),
            _number(tcp_rotation, "pitch"),
            _number(tcp_rotation, "yaw"),
        ],
        "end_effector_profile.raw_json": json.dumps(profile, separators=(",", ":")),
    }


def _to_parameter(name: str, value: Any) -> Parameter:
    parameter = Parameter()
    parameter.name = name
    parameter.value = ParameterValue()
    if isinstance(value, bool):
        parameter.value.type = ParameterType.PARAMETER_BOOL
        parameter.value.bool_value = value
    elif isinstance(value, float):
        parameter.value.type = ParameterType.PARAMETER_DOUBLE
        parameter.value.double_value = value
    elif isinstance(value, int):
        parameter.value.type = ParameterType.PARAMETER_INTEGER
        parameter.value.integer_value = value
    elif isinstance(value, list):
        parameter.value.type = ParameterType.PARAMETER_DOUBLE_ARRAY
        parameter.value.double_array_value = [float(item) for item in value]
    else:
        parameter.value.type = ParameterType.PARAMETER_STRING
        parameter.value.string_value = str(value)
    return parameter


def _set_remote_parameters(node: Node, target_node: str, parameters: Dict[str, Any]) -> bool:
    client = node.create_client(SetParameters, f"{target_node.rstrip('/')}/set_parameters")
    if not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error(f"Parameter service is not available for {target_node}")
        return False

    request = SetParameters.Request()
    request.parameters = [_to_parameter(name, value) for name, value in parameters.items()]
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    if future.result() is None:
        node.get_logger().error(f"Failed to set parameters on {target_node}: {future.exception()}")
        return False

    ok = True
    for result in future.result().results:
        if not result.successful:
            ok = False
            node.get_logger().error(result.reason)
    return ok


class EndEffectorProfileServer(Node):
    def __init__(self, node_name: str, parameters: Dict[str, Any]) -> None:
        super().__init__(node_name)
        for name, value in parameters.items():
            self.declare_parameter(name, value)
        self.get_logger().info(f"Loaded {len(parameters)} end-effector profile parameters")


def main(argv: List[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("profile", type=Path, help="Path to a *.endeffector-profile.json file")
    parser.add_argument(
        "--node-name",
        default="end_effector_profile_server",
        help="Parameter server node name used when --target-node is not provided",
    )
    parser.add_argument(
        "--target-node",
        default="",
        help="Existing node to receive parameters, for example /cartesian_impedance_controller",
    )
    parser.add_argument(
        "--no-spin",
        action="store_true",
        help="Exit after declaring local parameters or setting the target node",
    )
    raw_argv = sys.argv[1:] if argv is None else argv
    args = parser.parse_args(remove_ros_args(args=raw_argv))

    profile = json.loads(args.profile.read_text(encoding="utf-8"))
    parameters = _profile_to_parameters(profile)

    rclpy.init()
    node = EndEffectorProfileServer(args.node_name, parameters)
    try:
        if args.target_node:
            if not _set_remote_parameters(node, args.target_node, parameters):
                return 1
            node.get_logger().info(f"Set end-effector profile parameters on {args.target_node}")
        if not args.no_spin and not args.target_node:
            rclpy.spin(node)
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
