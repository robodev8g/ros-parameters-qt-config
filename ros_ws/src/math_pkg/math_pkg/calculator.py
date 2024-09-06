import rclpy
from rclpy.node import Node

from my_interfaces.srv import Exercise


class Calculator(Node):

    def __init__(self):
        super().__init__('calculator_node')
        self.declare_parameter('a', 0)
        self.declare_parameter('operator', '+')
        self.declare_parameter('b', 0)

        self.client = self.create_client(Exercise, 'solve')
        self.send_exercise()

    def send_exercise(self):
        # Create a service request
        request = Exercise.Request()
        request.num1 = self.get_parameter('a').get_parameter_value().integer_value
        request.op = self.get_parameter('operator').get_parameter_value().string_value
        request.num2 = self.get_parameter('b').get_parameter_value().integer_value

        # Send the request and wait for the response
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        response = future.result()
        self.get_logger().info(f"Response: {response}")



def main():
    rclpy.init()
    calculator_node = Calculator()
    rclpy.spin(calculator_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()