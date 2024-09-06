import rclpy
from ros2node.api import get_node_names


# rclpy.init()
# manager_node = rclpy.create_node("manager_node")
# nodes = get_node_names(node=manager_node, include_hidden_nodes=False)
# for name, namespace, full_name in nodes:
#     print(full_name)
# manager_node.destroy_node()
# rclpy.shutdown()
import os
import rclpy
from rclpy.node import Node

from my_interfaces.srv import NodeNames


class ParametersHandler(Node):

    def __init__(self):
        super().__init__('parameters_handler')
        self.create_service(NodeNames, 'get_node_names', self.get_node_names_callback)

    def get_node_names_callback(self, request: NodeNames.Request, response: NodeNames.Response):
        self.get_logger().info("Got node names request ! (option 1 implemented)")

        nodes = get_node_names(node=self, include_hidden_nodes=False)
        response.names = [full_name for name, namespace, full_name in nodes]

        # cmd_str = '/bin/bash /opt/ros/humble/setup.bash;ros2 node list'
        # node_list = os.popen(cmd_str).read()
        # response.names = node_list.split("\n")[:-1]
        
        return response


def main():
    rclpy.init()
    parameters_handler_node = ParametersHandler()
    rclpy.spin(parameters_handler_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
