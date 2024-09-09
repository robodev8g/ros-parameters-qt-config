import rclpy
from rclpy.node import Node

from ros2node.api import get_node_names
import os

from my_interfaces.msg import ParameterInfo
from my_interfaces.srv import NodeNames, ParameterList
from rcl_interfaces.srv import ListParameters, GetParameterTypes, GetParameters

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup


PARAMETER_TYPES = {
                    "1": "bool",
                    "2": "integer",
                    "3": "double",
                    "4": "string",
                    "5": "byte array",
                    "6": "bool array",
                    "7": "integer array",
                    "8": "double array",
                    "9": "string array"
                }

class ParameterServer(Node):

    def __init__(self):
        super().__init__('parameter_server')
        self.mcb1 = MutuallyExclusiveCallbackGroup()
        self.mcb2 = MutuallyExclusiveCallbackGroup()
        self.mcb3 = MutuallyExclusiveCallbackGroup()
        self.create_service(NodeNames, 'get_node_names', self.get_node_names_callback)
        self.create_service(ParameterList, 'get_node_parameter_list', self.get_node_parameter_list_callback, callback_group=self.mcb1)

    def get_node_names_callback(self, request: NodeNames.Request, response: NodeNames.Response):
        self.get_logger().info("Got node names request ! (option 1 implemented)")

        nodes = get_node_names(node=self, include_hidden_nodes=False)
        response.names = [full_name for name, namespace, full_name in nodes if "zenoh" not in name]

        # cmd_str = '/bin/bash /opt/ros/humble/setup.bash;ros2 node list'
        # node_list = os.popen(cmd_str).read()
        # response.names = node_list.split("\n")[:-1]

        return response

    async def get_node_parameter_list_callback(self, request: ParameterList.Request, response: ParameterList.Response):
        self.get_logger().info(f"Got request of parameter list for {request.node_name}")
        # Names
        client = self.create_client(ListParameters, f"{request.node_name}/list_parameters", callback_group=self.mcb2)
        future: ListParameters.Response = await client.call_async(request=ListParameters.Request())
        param_names = future.result.names
        
        # Types
        client = self.create_client(GetParameterTypes, f"{request.node_name}/get_parameter_types", callback_group=self.mcb3)
        future: GetParameterTypes.Response = await client.call_async(request=GetParameterTypes.Request(names=param_names))
        # param_types = [PARAMETER_TYPES[str(type)] for type in future.types]
        param_types = future.types

        # Values
        client = self.create_client(GetParameters, f"{request.node_name}/get_parameters", callback_group=self.mcb2)
        future: GetParameters.Response = await client.call_async(request=GetParameters.Request(names=param_names))
        param_values = []
        for param_value in future.values:
            attribute_name = PARAMETER_TYPES[str(param_value.type)] + " value"
            attribute_name = attribute_name.replace(" ", "_")            
            param_values.append(str(eval(f"param_value.{attribute_name}")))
        
        infos = []
        for param_name, param_type, param_value in zip(param_names, param_types, param_values):
            self.get_logger().info(f"Parameter Info: {param_name}, {param_type}, {param_value}")
            infos.append(ParameterInfo(param_name=param_name, param_type=param_type, param_value=param_value))
        
        response.parameters_info = infos
        return response



def main():
    rclpy.init()
    parameters_server_node = ParameterServer()
    rclpy.spin(parameters_server_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
