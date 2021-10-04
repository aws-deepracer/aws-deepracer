#################################################################################
#   Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.          #
#                                                                               #
#   Licensed under the Apache License, Version 2.0 (the "License").             #
#   You may not use this file except in compliance with the License.            #
#   You may obtain a copy of the License at                                     #
#                                                                               #
#       http://www.apache.org/licenses/LICENSE-2.0                              #
#                                                                               #
#   Unless required by applicable law or agreed to in writing, software         #
#   distributed under the License is distributed on an "AS IS" BASIS,           #
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.    #
#   See the License for the specific language governing permissions and         #
#   limitations under the License.                                              #
#################################################################################

"""
enable_deepracer_nav_node.py
This module enables the camera and servo GPIO.
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from deepracer_interfaces_pkg.srv import (VideoStateSrv,
                                          ServoGPIOSrv)

from enable_deepracer_nav_pkg import constants


class EnableDeepracerNavNode(Node):
    """This node enables the services for nodes required for DR Nav.
    """

    def __init__(self):
        """Create a EnableDeepracerNavNode.
        """
        super().__init__('enable_deepracer_nav_node')
        self.get_logger().info("enable_deepracer_nav_node started.")

        activate_camera_cb_group = ReentrantCallbackGroup()
        self.get_logger().info("Create activate camera service client: "
                               f"{constants.ACTIVATE_CAMERA_SERVICE}")
        self.activate_camera_cli = self.create_client(VideoStateSrv,
                                                     constants.ACTIVATE_CAMERA_SERVICE,
                                                     callback_group=activate_camera_cb_group)
        self.wait_for_service_availability(self.activate_camera_cli)

        enable_servo_gpio_cb_group = ReentrantCallbackGroup()
        self.get_logger().info("Create enable servo GPIO service client: "
                               f"{constants.SERVO_GPIO_SERVICE}")
        self.enable_servo_gpio_cli = self.create_client(ServoGPIOSrv,
                                                     constants.SERVO_GPIO_SERVICE,
                                                     callback_group=enable_servo_gpio_cb_group)
        self.wait_for_service_availability(self.enable_servo_gpio_cli)

        self.activate_camera_req = VideoStateSrv.Request()
        self.activate_camera_req.activate_video = 1
        self.send_request(self.activate_camera_req, self.activate_camera_cli)

        self.enable_servo_gpio_req = ServoGPIOSrv.Request()
        self.enable_servo_gpio_req.enable = 0
        self.send_request(self.enable_servo_gpio_req, self.enable_servo_gpio_cli)

    def wait_for_service_availability(self, client):
        """Helper function to wait for the service to which the client subscribes to is alive.
        Args:
            client (rclpy.client.Client): Client object for the service to wait for.
        """
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"{client.srv_name} service not available, waiting again...")

    def send_request(self, req, cli):
        """A wrapper function to call the services.
        Args:
            req (Request): Service request object.
            cli (rclpy.client.Client): Client object using which we call the service.
        """
        try:
            if cli.service_is_ready():
                self.get_logger().info(f"Service call initiated: {req}")
                future = cli.call_async(req)
        except Exception as ex:
            self.get_logger().error(f"Service call failed: {ex}")


def main(args=None):
    rclpy.init(args=args)
    enable_deepracer_nav_node = EnableDeepracerNavNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(enable_deepracer_nav_node, executor)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    enable_deepracer_nav_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
