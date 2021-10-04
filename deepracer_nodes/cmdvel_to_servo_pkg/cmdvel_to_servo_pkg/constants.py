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

ACTION_PUBLISH_TOPIC = "servo_msg"
SET_MAX_SPEED_SERVICE_NAME = "set_max_speed"

CMDVEL_TOPIC = "/cmd_vel"


class VehicleNav2Dynamics():
    """Class with the vehicle dynamics constants for physical
    DeepRacer.
    """
    # Max speed in m/s
    MAX_SPEED = 4.0
    # Min speed in m/s
    MIN_SPEED = -4.0
    # Max steering angle of tyre
    MAX_STEER = 0.523599
    # Min steering angle of tyre
    MIN_STEER = -0.523599
    # Ratios for throttle calculated using abs(given_throttle / max_throttle)
    MIN_THROTTLE_RATIO = 0.5
    MID_THROTTLE_RATIO = 0.3
    MAX_THROTTLE_RATIO = 0.1
    # Ratios for steering angles calculated using abs(given_angle / max_angle)
    MIN_STEERING_RATIO = 0.8
    MID_STEERING_RATIO = 0.4
    MAX_STEERING_RATIO = 0.2


class ActionValues():
    """Class with the PWM values with respect to
       the possible actions that can be sent to servo, pertaining to
       the angle and throttle.
    """
    MIN_THROTTLE_OUTPUT = 0.5
    MID_THROTTLE_OUTPUT = 0.8
    MAX_THROTTLE_OUTPUT = 1.0
    MAX_STEERING_OUTPUT = 1.0
    MID_STEERING_OUTPUT = 0.5
    MIN_STEERING_OUTPUT = 0.2
    DEFAULT_OUTPUT = 0.0

# Max speed percentage on a scale between 0.0 and 1.0.
# The maximum speed value is used to non linearly map the raw value obtained for the forward
# and reverse throttle to the PWM values of the servo/motor.
# We use the maximum speed % to map to a range of [1.0, 5.0] speed scale values using the
# calculated coefficients of the equation y = ax^2 + bx.
# This allows us to recalculate the curve for each maximum speed % value and use that to
# map the speed values. The idea behind this mapping is a lower percentage of maximum speed %
# should map to a higher speed scale value while calculating the coefficients so that the curve
# is more flatter and the impact of actual speed values is less for lower max speed %.
MAX_SPEED_PCT = 0.68

# Action space mapped to on the vehicle for speed values of 0.8 and 0.4.
DEFAULT_SPEED_SCALES = [1.0, 0.8]
# Speed scale bounds to pick from while calculating the coefficients.
MANUAL_SPEED_SCALE_BOUNDS = [1.0, 5.0]
