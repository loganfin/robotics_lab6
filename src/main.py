import sys
from time import sleep

# ROS packages
import rclpy
from rclpy.action.client import ActionClient
from rclpy.node import Node

sys.path.append("../dependencies/")
# Fanuc packages
import fanuc_interfaces
from fanuc_interfaces.action import CartPose, SchunkGripper
from key_commander import KeyCommander

# This is important for running your nodes from the terminal
from pynput.keyboard import KeyCode

namespace = "bunsen"

p = [
    # Rest position
    [522.7, 5.9, 96.4, 179.9, 0.0, 30.0],
    # Die table position
    [528.298, 3.742, -196.421, -179.9, 0.0, 30.0],
    # Start of conveyor (above)
    [895.488892, -596.086243, -39.261475, 179.9, 0.0, 30.0],
    # Start of conveyor (contact with belt)
    [888.968872, -593.533020, -205.173096, -179.9, 0.0, 30.0],
    # End of conveyor (above)
    [77.675018, -575.866882, -27.547470, 179.9, 0.0, 30.0],
    # End of conveyor (contact with belt)
    [70.0, -574.967712, -203.650116, -179.9, 0.0, 30.0],
    # Die table position + 90 degree roll rotation
    [528.298, 3.742, -196.421, -179.9, 0.0, 120.0],
]


class Demo(Node):
    def __init__(self, namespace):
        super().__init__("robot")

        # Actions
        self.cart_ac = ActionClient(
            self, CartPose, f"/{namespace}/cartesian_pose"
        )

        # self.gripper_action = ActionClient(
        #     self, SchunkGripper, f"/{namespace}/schunk_gripper"
        # )

    def send_cart_action(self, coord):
        # Do we need a new goal per action?
        goal = CartPose.Goal()  # Make Goal

        goal.x = coord[0]
        goal.y = coord[1]
        goal.z = coord[2]
        goal.w = coord[3]
        goal.p = coord[4]
        goal.r = coord[5]

        result = self.cart_ac.send_goal(goal).result
        self.get_logger().info(f"Result {result}")

    def demo(self):
        self.cart_ac.wait_for_server()  # Wait till it's ready

        # self.get_logger().info("p[0]")
        # self.send_cart_action(p[0])

        # self.get_logger().info("p[1]")
        # self.send_cart_action(p[1])

        # self.get_logger().info("p[2]")
        # self.send_cart_action(p[2])

        # self.get_logger().info("p[3]")
        # self.send_cart_action(p[3])

        # self.get_logger().info("p[2]")
        # self.send_cart_action(p[2])

        # self.get_logger().info("p[4]")
        # self.send_cart_action(p[4])

        # self.get_logger().info("p[5]")
        # self.send_cart_action(p[5])

        # self.get_logger().info("p[4]")
        # self.send_cart_action(p[4])

        while True:
            self.get_logger().info("p[0]")
            self.send_cart_action(p[0])

            self.get_logger().info("p[6]")
            self.send_cart_action(p[6])

        # self.get_logger().info("p[0]")
        # self.send_cart_action(p[0])

        # self.cart_ac.wait_for_server()  # Wait till it's ready

        # Do we need to wait_for_server() after every action?
        # self.send_cart_action(cart_goal, p[1])


if __name__ == "__main__":
    rclpy.init()

    bunsen = Demo(namespace)

    # This allows us to start the function once the node is spinning
    keycom = KeyCommander(
        [
            (KeyCode(char="s"), bunsen.demo),
        ]
    )
    print("S")

    rclpy.spin(bunsen)  # Start executing the node
    rclpy.shutdown()

    print("Program finished")
