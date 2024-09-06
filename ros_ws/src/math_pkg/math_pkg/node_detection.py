import rclpy
from ros2node.api import get_node_names


# rclpy.init()
# manager_node = rclpy.create_node("manager_node")
# nodes = get_node_names(node=manager_node, include_hidden_nodes=False)
# for name, namespace, full_name in nodes:
#     print(full_name)
# manager_node.destroy_node()
# rclpy.shutdown()

import subprocess
cmd_str = '/bin/bash /opt/ros/humble/setup.bash;ros2 node list'

node_list = subprocess.run(cmd_str, shell=True)
print(node_list)