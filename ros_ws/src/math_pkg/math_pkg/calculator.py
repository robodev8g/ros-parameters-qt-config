import rclpy
from rclpy.node import Node

from my_interfaces.srv import Exercise
from std_srvs.srv import Trigger

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from threading import Event

class Calculator(Node):

    def __init__(self):
        super().__init__('calculator_node')
        self.declare_parameter('a', 0)
        self.declare_parameter('operator', '+')
        self.declare_parameter('b', 0)


        self.client = self.create_client(Exercise, 'solve')

        # self.create_service(Trigger, 'use_calculator_command', self.use_calculator_command_callback, callback_group=self.rcb)
        self.timer = self.create_timer(1, self.send_exercise)


    def send_exercise(self):
        # Create a service request
        request = Exercise.Request()
        request.num1 = self.get_parameter('a').get_parameter_value().integer_value
        request.op = self.get_parameter('operator').get_parameter_value().string_value
        request.num2 = self.get_parameter('b').get_parameter_value().integer_value

        # Send the request and wait for the response
        self.get_logger().info("send an exercise to server")
        future = self.client.call_async(request)
        future.add_done_callback(self.print_result)

    def print_result(self, future):
        exercise_response: Exercise.Response  = future.result()
        self.get_logger().info(f"Response: {exercise_response}")

    # def use_calculator_command_callback(self, calculator_request: Trigger.Request, calculator_response: Trigger.Response):
    #     self.get_logger().info("get trigger to send new exercise")

    #     event=Event()
    #     def done_callback(future):
    #         self.get_logger().info(f"done callback")
    #         nonlocal event
    #         event.set()
        
    #     future = self.send_exercise()
    #     self.get_logger().info(f"exit send_exercise method")
    #     future.add_done_callback(done_callback)

    #     event.wait()

    #     # rclpy.spin_until_future_complete(self, executor=self.executor, future=future)
    #     exercise_response: Exercise.Response  = future.result()
    #     self.get_logger().info(f"Response: {exercise_response}")
    #     calculator_response.success = exercise_response.success

    #     # calculator_response.success = True
    #     calculator_response.message = "exercise was calculated successfully" if calculator_response.success else "illegal exercise"
    #     return calculator_response




def main():
    rclpy.init()
    calculator_node = Calculator()
    rclpy.spin(calculator_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()