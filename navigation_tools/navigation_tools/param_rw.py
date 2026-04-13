#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from pumas_interfaces.srv import ParamReadWrite
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.node import Node


class ParamReadWriteServer(Node):
    def __init__(self):
        super().__init__("param_read_write_server")

        self.srv = self.create_service(
            ParamReadWrite, "/param_read_write", self.handle_request
        )
        self.get_logger().info(
            "ParamReadWriteServer.-> ready for parameter read/write requests."
        )

    def _normalize_node_name(self, node_name: str) -> str:
        return node_name.lstrip("/")

    def handle_request(self, request, response):
        node_name = self._normalize_node_name(request.node_name)
        param_name = request.param_name
        write_mode = request.write
        value_str = request.value

        try:
            if write_mode:
                converted_value = self._convert_value(value_str)
                success = self.write_param(
                    node_name, param_name, converted_value)
                response.success = success
                response.param_value = str(converted_value)
                self.get_logger().info(
                    f"ParamWrite.-> /{node_name}/{param_name} = {converted_value} "
                    f"(success={success})"
                )
            else:
                value = self.read_param(node_name, param_name)
                if value is not None:
                    response.success = True
                    response.param_value = str(value)
                    self.get_logger().info(
                        f"ParamRead.-> /{node_name}/{param_name} = {value}"
                    )
                else:
                    response.success = False
                    response.param_value = "None"
                    self.get_logger().warn(
                        f"ParamRead.-> Failed to read /{node_name}/{param_name}"
                    )
        except Exception as e:
            self.get_logger().error(f"ParamRWServer.-> Exception: {e}")
            response.success = False
            response.param_value = "error"

        return response

    def _convert_value(self, value_str):
        lower_val = value_str.lower()
        if lower_val == "true":
            return True
        elif lower_val == "false":
            return False
        try:
            if "." in value_str:
                return float(value_str)
            else:
                return int(value_str)
        except ValueError:
            return value_str

    def read_param(self, node_name, param_name):
        service_name = f"/{node_name}/get_parameters"
        client = self.create_client(GetParameters, service_name)

        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(
                f"ParamRW.-> {service_name} not available."
            )
            return None

        req = GetParameters.Request(names=[param_name])
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self._node, future)

        if future.result() is None:
            self.get_logger().warn(
                f"ParamRW.-> get_parameters returned no result for /{node_name}/{param_name}"
            )
            return None

        result = future.result()
        if not result.values:
            return None

        v = result.values[0]
        if v.type == ParameterType.PARAMETER_BOOL:
            return v.bool_value
        elif v.type == ParameterType.PARAMETER_INTEGER:
            return v.integer_value
        elif v.type == ParameterType.PARAMETER_DOUBLE:
            return v.double_value
        elif v.type == ParameterType.PARAMETER_STRING:
            return v.string_value
        else:
            return None

    def write_param(self, node_name, param_name, new_value):
        service_name = f"/{node_name}/set_parameters"
        self.get_logger().info(
            f"ParamRW.-> Writing /{node_name}/{param_name} = {new_value}"
        )

        client = self.create_client(SetParameters, service_name)
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(
                f"ParamRW.-> {service_name} not available."
            )
            return False

        param_value = ParameterValue()
        if isinstance(new_value, bool):
            param_value.type = ParameterType.PARAMETER_BOOL
            param_value.bool_value = new_value
        elif isinstance(new_value, int):
            param_value.type = ParameterType.PARAMETER_INTEGER
            param_value.integer_value = new_value
        elif isinstance(new_value, float):
            param_value.type = ParameterType.PARAMETER_DOUBLE
            param_value.double_value = new_value
        elif isinstance(new_value, str):
            param_value.type = ParameterType.PARAMETER_STRING
            param_value.string_value = new_value
        else:
            self.get_logger().warn(
                f"Unsupported type for parameter: {type(new_value)}"
            )
            return False

        req = SetParameters.Request(
            parameters=[Parameter(name=param_name, value=param_value)]
        )
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self._node, future)

        if future.result() is None:
            self.get_logger().warn(
                f"ParamRW.-> set_parameters returned no result for /{node_name}/{param_name}"
            )
            return False

        result = future.result()
        if not result.results:
            return False

        success = all(r.successful for r in result.results)
        if not success:
            reasons = [r.reason for r in result.results if not r.successful]
            self.get_logger().warn(
                f"ParamRW.-> Failed to set /{node_name}/{param_name}: {reasons}"
            )
        return success


def main(args=None):
    rclpy.init(args=args)
    node = ParamReadWriteServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
