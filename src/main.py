import sys

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
    [522.718262, 5.908576, 96.417938, 179.9, 0, 30],
    # Die table position
    [528.298, 3.742, -196.421, -179.9, 0, 30],
    # Start of conveyor (above)
    [895.488892, -596.086243, -39.261475, 179.9, 0, 30],
    # Start of conveyor (contact with belt)
    [888.968872, -593.533020, -205.173096, -179.9, 0, 30],
    # End of conveyor (above)
    [77.675018, -575.866882, -27.547470, 179.9, 0, 30],
    # End of conveyor (contact with belt)
    [70, -574.967712, -203.650116, -179.9, 0, 30],
    # Die table position + 90 degree roll rotation
    [528.298, 3.742, -196.421, -179.9, 0, 120],
]


class Demo(Node):
    def __init__(self, namespace):
        super().__init__("robot")

        # Actions
        self.cart_ac = ActionClient(
            self, CartPose, f"/{namespace}/cartesian_pose"
        )

        self.gripper_action = ActionClient(
            self, SchunkGripper, f"/{namespace}/schunk_gripper"
        )

    def send_cart_action(self, goal, coord):
        self.cart_ac.x = coord[0]
        self.cart_ac.y = coord[1]
        self.cart_ac.z = coord[2]
        self.cart_ac.w = coord[3]
        self.cart_ac.p = coord[4]
        self.cart_ac.r = coord[5]
        self.cart_ac.send_goal(goal)

    def demo(self):
        self.cart_ac.wait_for_server()  # Wait till it's ready

        # Do we need a new goal per action?
        cart_goal = CartPose.Goal()  # Make Goal

        self.send_cart_action(cart_goal, p[0])

        # Do we need to wait_for_server() after every action?
        self.send_cart_action(cart_goal, p[1])


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
