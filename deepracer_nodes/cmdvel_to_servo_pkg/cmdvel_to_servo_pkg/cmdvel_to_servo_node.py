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
cmdvel_to_servo_node.py
This module decides the action messages (servo control messages specifically angle
and throttle) to be sent out after converting the cmd_vel.

The node defines:
    cmdvel_subscriber: A subscriber to the /cmd_vel (twist messages) published by
                       ROS2 Nav stack.
    action_publisher: A publisher to publish the action (angle and throttle values).
    set_max_speed_service: A service to dynamically set MAX_SPEED_PCT representing
                           the max speed percentage scale as per request.
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import (QoSProfile,
                       QoSHistoryPolicy,
                       QoSReliabilityPolicy)

import geometry_msgs.msg
from deepracer_interfaces_pkg.msg import ServoCtrlMsg
from deepracer_interfaces_pkg.srv import SetMaxSpeedSrv
from cmdvel_to_servo_pkg import constants
import math
import threading


class CmdvelToServoNode(Node):
    """Node responsible for deciding the action messages (servo control messages specifically angle
       and throttle) to be sent out after converting the cmd_vel.
    """

    def __init__(self, qos_profile):
        """Create a CmdvelToServoNode.
        """
        super().__init__('cmdvel_to_servo_node')
        self.get_logger().info("cmdvel_to_servo_node started.")

        # Create subscription to cmd_vel.
        self.cmdvel_subscriber = \
            self.create_subscription(geometry_msgs.msg.Twist,
                                     constants.CMDVEL_TOPIC,
                                     self.on_cmd_vel,
                                     qos_profile)

        # Creating publisher to publish action (angle and throttle).
        self.action_publisher = self.create_publisher(ServoCtrlMsg,
                                                      constants.ACTION_PUBLISH_TOPIC,
                                                      qos_profile)

        # Service to dynamically set MAX_SPEED_PCT.
        self.set_max_speed_service = self.create_service(SetMaxSpeedSrv,
                                                         constants.SET_MAX_SPEED_SERVICE_NAME,
                                                         self.set_max_speed_cb)

        # Linear velocity in X received on command (m/s).
        self.target_linear = 0.0
        # Angular velocity in Z received on command (rad/s).
        self.target_rot = 0.0
        # Max speed pct for throttle output to be rescaled with respect to.
        self.max_speed_pct = constants.MAX_SPEED_PCT
        self.lock = threading.Lock()

    def set_max_speed_cb(self, req, res):
        """Callback which dynamically sets the max_speed_pct.
        Args:
            req (SetMaxSpeedSrv.Request): Request object with the updated
                                                  max speed percentage.
            res (SetMaxSpeedSrv.Response): Response object with error(int) flag
                                                   indicating successful max speed pct
                                                   update.
        Returns:
            SetMaxSpeedSrv.Response: Response object with error(int) flag indicating
                                             successful max speed pct update.
        """
        with self.lock:
            try:
                self.max_speed_pct = req.max_speed_pct
                self.get_logger().info(f"Incoming request: max_speed_pct: {req.max_speed_pct}")
                res.error = 0
            except Exception as ex:
                self.get_logger().error(f"Failed set max speed pct: {ex}")
                res.error = 1
        return res

    def get_rescaled_manual_speed(self, categorized_throttle, max_speed_pct):
        """Return the non linearly rescaled speed value based on the max_speed_pct.
        Args:
            categorized_throttle (float): Float value ranging from -1.0 to 1.0.
            max_speed_pct (float): Float value ranging from 0.0 to 1.0 taken as input
                                   from maximum speed input.
        Returns:
            float: Categorized value of the input speed.
        """
        # return 0.0 if categorized_throttle or maximum speed pct is 0.0.
        if categorized_throttle == 0.0 or max_speed_pct == 0.0:
            return 0.0

        # get the parameter value to calculate the coefficients a, b in the equation y=ax^2+bx
        # The lower the update_speed_scale_value parameter, higher the impact on the
        # final mapped_speed.
        # Hence the update_speed_scale_value parameter is inversely associated with max_speed_pct
        # and bounded by MANUAL_SPEED_SCALE_BOUNDS.
        # Ex: max_speed_pct = 0.5; update_speed_scale_value = 3
        #     max_speed_pct = 1.0; update_speed_scale_value = 1
        # Lower the update_speed_scale_value: categorized_throttle value gets mapped to
        # higher possible values.
        #   Example: update_speed_scale_value = 1.0;
        #            categorized_throttle = 0.8 ==> mapped_speed = 0.992
        # Higher the update_speed_scale_value: categorized_throttle value gets mapped to
        # lower possible values.
        #   Example: update_speed_scale_value = 3.0;
        #            categorized_throttle = 0.8 ==> mapped_speed = 0.501

        inverse_max_speed_pct = (1 - max_speed_pct)

        update_speed_scale_value = \
            constants.MANUAL_SPEED_SCALE_BOUNDS[0] + \
            inverse_max_speed_pct * \
            (constants.MANUAL_SPEED_SCALE_BOUNDS[1] - constants.MANUAL_SPEED_SCALE_BOUNDS[0])

        if update_speed_scale_value < 0.0:
            self.get_logger().info("The update_speed_scale_value is negative, taking absolute value.")
            update_speed_scale_value = abs(update_speed_scale_value)
        speed_mapping_coefficients = dict()

        # recreate the mapping coefficients for the non-linear equation ax^2 + bx based on
        # the update_speed_scale_value.
        # These coefficents map the [update_speed_scale_value, update_speed_scale_value/2]
        # values to DEFAULT_SPEED_SCALE values [1.0, 0.8].
        speed_mapping_coefficients["a"] = \
            (1.0 / update_speed_scale_value**2) * \
            (2.0 * constants.DEFAULT_SPEED_SCALES[0] - 4.0 * constants.DEFAULT_SPEED_SCALES[1])
        speed_mapping_coefficients["b"] = \
            (1.0 / update_speed_scale_value) * \
            (4.0 * constants.DEFAULT_SPEED_SCALES[1] - constants.DEFAULT_SPEED_SCALES[0])
        return math.copysign(1.0, categorized_throttle) * \
            (speed_mapping_coefficients["a"] * abs(categorized_throttle)**2 +
             speed_mapping_coefficients["b"] * abs(categorized_throttle))

    def on_cmd_vel(self, msg):
        """Callback on receiving a velocity update from ROS2 Nav stack.
        Args:
            msg: (geometry_msgs.msg.Twist): Geometry twist message.
        """
        try:
            self.target_linear = msg.linear.x
            self.target_rot = msg.angular.z
            target_steer, target_speed = self.plan_action()
            self.action_publish(target_steer, target_speed)
        except Exception as ex:
            self.get_logger().error(f"Failed to publish action: {ex}")
            self.action_publish(constants.ActionValues.DEFAULT_OUTPUT, constants.ActionValues.DEFAULT_OUTPUT)

    def get_mapped_throttle(self, target_linear_clamped):
        """Get the mapped throttle value for DR servo scaled between [-1, 1].
        Args:
            target_linear_clamped (float): Clamped linear velocity between MAX_SPEED 
                                           and MIN_SPEED supported by DeepRacer.

        Returns:
            (float): Throttle value mapped wrt DeepRacer.
        """
        target_linear_pct = abs(target_linear_clamped / constants.VehicleNav2Dynamics.MAX_SPEED)
        if target_linear_pct >= constants.VehicleNav2Dynamics.MAX_THROTTLE_RATIO:
            return constants.ActionValues.MAX_THROTTLE_OUTPUT
        elif target_linear_pct >= constants.VehicleNav2Dynamics.MID_THROTTLE_RATIO:
            return constants.ActionValues.MID_THROTTLE_OUTPUT
        elif target_linear_pct >= constants.VehicleNav2Dynamics.MIN_THROTTLE_RATIO:
            return constants.ActionValues.MIN_THROTTLE_OUTPUT
        else:
            return constants.ActionValues.DEFAULT_OUTPUT

    def get_mapped_steering(self, target_rot_clamped):
        """Get the mapped steering value for DR servo scaled between [-1, 1].
        Args:
            target_rot_clamped (float): Clamped rotation between MAX_STEER
                                        and MIN_STEER supported by DeepRacer.
        Returns:
            (float): Angle value mapped wrt DeepRacer.
        """
        target_rot_pct = abs(target_rot_clamped / constants.VehicleNav2Dynamics.MAX_STEER)
        if target_rot_pct >= constants.VehicleNav2Dynamics.MAX_STEERING_RATIO:
            return constants.ActionValues.MAX_STEERING_OUTPUT
        elif target_rot_pct >= constants.VehicleNav2Dynamics.MID_STEERING_RATIO:
            return constants.ActionValues.MID_STEERING_OUTPUT
        elif target_rot_pct >= constants.VehicleNav2Dynamics.MIN_STEERING_RATIO:
            return constants.ActionValues.MIN_STEERING_OUTPUT
        else:
            return constants.ActionValues.DEFAULT_OUTPUT

    def plan_action(self):
        """Calculate the target steering and throttle.

        Returns:
            steering (float): Angle value to be published to servo.
            throttle (float): Throttle value to be published to servo.
        """
        # Clamping the linear velocity between MAX_SPEED and MIN_SPEED supported by DeepRacer.
        target_linear_clamped = max(min(self.target_linear, constants.VehicleNav2Dynamics.MAX_SPEED),
                                            constants.VehicleNav2Dynamics.MIN_SPEED)
        # Get the throttle values mapped wrt DeepRacer servo.
        target_throttle_mapped = self.get_mapped_throttle(target_linear_clamped)
        # Set the direction.
        target_throttle_signed = target_throttle_mapped * math.copysign(1.0, self.target_linear)
        # Get rescaled throttle.
        throttle = self.get_rescaled_manual_speed(target_throttle_signed, self.max_speed_pct)
        # Clamping the rotation between MAX_STEER and MIN_STEER supported by DeepRacer.
        target_rot_clamped = max(min(self.target_rot, constants.VehicleNav2Dynamics.MAX_STEER),
                                        constants.VehicleNav2Dynamics.MIN_STEER)
        # Get the steering angle mapped wrt DeepRacer servo.
        target_steering_mapped = self.get_mapped_steering(target_rot_clamped)
        # Set the direction.
        steering = target_steering_mapped * math.copysign(1.0, self.target_rot) * math.copysign(1.0, self.target_linear)
        return steering, throttle

    def action_publish(self, target_steer, target_speed):
        """Function publishes the action and sends it to servo.

        Args:
            target_steer (float): Angle value to be published to servo.
            target_speed (float): Throttle value to be published to servo.
        """
        result = ServoCtrlMsg()
        result.angle, result.throttle = target_steer, target_speed
        self.get_logger().info(f"Publishing to servo: Steering {target_steer} | Throttle {target_speed}")
        self.action_publisher.publish(result)


def main(args=None):
    rclpy.init(args=args)
    qos = QoSProfile(reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
                     depth=1,
                     history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST)
    cmdvel_to_servo_node = CmdvelToServoNode(qos)
    executor = MultiThreadedExecutor()
    rclpy.spin(cmdvel_to_servo_node, executor)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    cmdvel_to_servo_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
