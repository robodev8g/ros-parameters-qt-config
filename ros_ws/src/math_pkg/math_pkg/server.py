import rclpy
from rclpy.node import Node

from my_interfaces.srv import Exercise

class MathServer(Node):

    def __init__(self):
        super().__init__('math_server_node')
        self.declare_parameter('legal_operators', ['+', '-', '*'])
        self.srv = self.create_service(Exercise, 'solve', self.solve_callback)

    def solve_callback(self, request: Exercise.Request, response: Exercise.Response):
        self.get_logger().info(f"Incoming request: {request.num1} {request.op} {request.num2}")
        legal_operators = self.get_parameter('legal_operators').get_parameter_value().string_array_value
        if request.op in legal_operators:
            response.res = float(eval(f"{request.num1}{request.op}{request.num2}"))
            response.success = True
            self.get_logger().info(f"success: returns {response.res}")
        else:
            response.res = -1.0
            response.success = False
            self.get_logger().info(f"fail: returns {response.res}")
        
        return response


def main():
    rclpy.init()
    math_server_node = MathServer()
    rclpy.spin(math_server_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()