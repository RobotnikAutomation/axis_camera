#!/usr/bin/env python3

# Software License Agreement (BSD License)
#
# Copyright (c) 2014, Robotnik Automation SLL
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Robotnik Automation SSL nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from math import pi as PI
from math import degrees as rad2deg
from math import radians as deg2rad
from copy import deepcopy

from threading import Thread

import rclpy
import rclpy.time
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup

from axis_camera.axis_lib.axis_control import ControlAxis

from robotnik_actuators_msgs.msg import Ptz as PtzMsg
from robotnik_actuators_msgs.action import SetPtz
from robotnik_sensors_msgs.msg import Axis
from sensor_msgs.msg import JointState


class Joint:
    def __init__(self, min_position, max_position, joint_name, offset, error, invert=False):
        self._min = min_position
        self._max = max_position
        self._total_range = max_position - min_position
        self._name = joint_name
        self._offset = offset
        self._error = error
        self._invert = -1 if invert else 1
        self._raw_current_position = 0.0
        self._current_position_processed = 0.0
        self._raw_user_desired_position = 0.0
        self._user_desired_position_processed = 0.0
        self._real_desired_position = self._offset
        self._velocity = 0.0
    
    def updatePosition(self, position):
        """
        Update the current position of the joint.

        This function takes the position received from the real camera
        and transforms it to the position of the camera frame.
        
        Args:
            position (float): The position received from the real camera.
        """
        # Position read from the camera
        self._raw_current_position = position
        # Apply offset and inversion to real position and normalize the angle. Data used in joint_states.
        self._current_position_processed = self._invert * self._normalizeAngle(position - self._offset)
    
    def setDesiredPosition(self, position):
        """
        Set the desired position of the camera.

        This method transforms the desired position in reference to the camera frame
        into the real position of the camera. The transformation is done by applying
        an inversion factor and an offset.

        Args:
            position (float): The desired position in reference to the camera frame.
        """
        # Position received from user
        self._raw_user_desired_position = position
        # Enforce limits to the desired position [min', max']. Target joint_states.
        self._user_desired_position_processed = self._normalizeAngle(self._enforceLimits(position))
        # Calculate the real desired position adding the offset and taking into account the inversion
        self._real_desired_position = self._invert * rad2deg(self._user_desired_position_processed + self._offset)

    def _enforceLimits(self, position, use_offset = True) -> float:
        """
        Enforces the limits of the joint position.

        This method ensures that the position is within the defined limits
        of the joint. If the position is outside the limits, it will be
        clamped to the nearest limit.

        Args:
            position (float): The position to enforce limits on.
            use_offset (bool): If True, the offset will be considered when enforcing limits.

        Returns:
            float: The enforced position within the limits.
        """
        offset = self._offset if use_offset else 0.0
        return max(self._min - offset, min(self._max - offset, position))

    def _normalizeAngle(self, angle) -> float:
        normalized_angle = angle
        while normalized_angle > PI:
            normalized_angle -= 2 * PI
        while normalized_angle < -PI:
            normalized_angle += 2 * PI
        
        return normalized_angle

    def setCurrentPositionAsDesired(self):
        """
        Sets the current position as the desired position.
        
        This method is useful when the current position is already the desired one,
        and we want to avoid sending unnecessary commands to the camera.
        """
        self.setDesiredPosition(self._current_position_processed)

    def isInDesiredPosition(self):
        """
        Checks if the current position is within the desired position range.

        Returns:
            bool: True if the current position is within the desired range, False otherwise.
        """
        return abs(self._current_position_processed - self._user_desired_position_processed) <= self._error

    def getRemainingPosition(self):
        """
        Returns the remaining position to reach the desired position.

        This method calculates the difference between the desired position and the current position.

        Returns:
            float: The remaining position to reach the desired position.
        """
        return self._user_desired_position_processed - self._current_position_processed

    def getCurrentPosition(self):
        return self._current_position_processed

    def getRawCurrentPosition(self):
        return self._raw_current_position

    def getName(self):
        return self._name

    def setDesiredVelocity(self, velocity):
        self._velocity = velocity

    def getDesiredVelocity(self):
        """
        Returns the desired velocity of the joint.

        This method returns the desired velocity that has been set for the joint.
        """
        return self._velocity

class ZoomJoint(Joint):
    def __init__(self, min_position, max_position, joint_name, offset, error, min_augment, max_augment, min_step):
        super().__init__(min_position, max_position, joint_name, offset, error, invert = False)
        self._min_augment = min_augment
        self._max_augment = max_augment
        self._total_augment = max_augment - min_augment
        self._min_step = min_step
    
    def updatePosition(self, position):
        # Position read from the camera
        self._raw_current_position = position
        # Apply offset to real position. This is the position used in joint_states.
        self._current_position_processed = position - self._offset

    def setCurrentPositionAsDesired(self):
        self.setDesiredPosition(self._current_position_processed * self._total_augment / self._total_range)
        # self._raw_user_desired_position = self._current_position_processed
        # self._real_desired_position = self._enforceLimits(self._raw_user_desired_position + self._offset)
    
    def setDesiredPosition(self, position):
        """
        Set the desired position of the zoom joint. This should be a value between the minimum and maximum augment values.
        """
        # Apply offset to min and max augments
        max_augment = round(self._max_augment - self._offset * self._total_augment / self._total_range)
        min_augment = round(self._min_augment - self._offset * self._total_augment / self._total_range)

        # Desired position received from user
        self._raw_user_desired_position = round(position)

        # Enforce limits to the user desired position [min_augment', max_augment']
        limited_position = max(min_augment, min(max_augment, position))

        # Process the desired position to the camera frame in zoom units [min', max']. Target joint_states.
        self._user_desired_position_processed = self._enforceLimits((self._total_range / self._total_augment) * limited_position  + self._min)
        # Calculate the real desired position adding the offset
        self._real_desired_position = self._user_desired_position_processed + self._offset

    def getNormalizedPosition(self):
        """
        Returns the normalized position of the zoom joint.

        The normalized position is calculated based on the current position,
        the minimum and maximum augment values, and the minimum step.
        """
        return (self._total_augment / self._total_range) * (self._current_position_processed - self._min)

class Ptz:
    def __init__(self, hostname : str, camera_id : int, pan : Joint, tilt : Joint, zoom : ZoomJoint):
        self.controller = ControlAxis(hostname, camera_id)
        self.hostname = hostname
        self.pan = pan
        self.tilt = tilt
        self.zoom = zoom
        self.iris = -1
        self.autoiris = False
        self.focus = -1
        self.autofocus = False
        self._is_syncronized = False
        info = self.controller.getPTZInfo()
        self._pan_tilt_velocity_control = "continuouspantiltmove" in info.keys()
        self._zoom_velocity_control = "continuouszoommove" in info.keys()
    
    def updatePtzPosition(self):
        """ Updates the PTZ position with new values. """
        # First time saves the current values
        ptz_read = self.controller.getPTZState()
        if not ptz_read["error_reading"]:
            self.pan.updatePosition(ptz_read["pan"])
            self.tilt.updatePosition(ptz_read["tilt"])
            self.zoom.updatePosition(ptz_read["zoom"])
            self.iris = ptz_read["iris"]
            self.autoiris = ptz_read["autoiris"]
            self.focus = ptz_read["focus"]
            self.autofocus = ptz_read["autofocus"]
        
            if not self._is_syncronized:
                self.setCurrentPtzPositionAsDesired()
                self._is_syncronized = True
        
        return ptz_read["error_reading"], ptz_read["error_reading_msg"]
    
    def isSyncronized(self):
        """ Checks if the PTZ state is synchronized. """
        return self._is_syncronized
    
    def setDesiredPtzPosition(self, pan = None, tilt = None, zoom = None):
        """ Sets the desired PTZ position with new values. """
        if pan is not None:
            self.pan.setDesiredPosition(pan)
        if tilt is not None:
            self.tilt.setDesiredPosition(tilt)
        if zoom is not None:
            self.zoom.setDesiredPosition(zoom)

    def setDesiredVelocity(self, pan = 0, tilt = 0, zoom = 0):
        self.pan.setDesiredVelocity(pan)
        self.tilt.setDesiredVelocity(tilt)
        self.zoom.setDesiredVelocity(zoom)

    def setCurrentPtzPositionAsDesired(self):
        """ Sets the current PTZ position as the desired position. """
        self.pan.setCurrentPositionAsDesired()
        self.tilt.setCurrentPositionAsDesired()
        self.zoom.setCurrentPositionAsDesired()
    
    def getRemainingPtzPosition(self):
        """
        Returns the remaining position to reach the desired PTZ position.

        This method calculates the difference between the desired position and the current position
        for each joint (pan, tilt, zoom).

        Returns:
            tuple: A tuple containing the remaining pan, tilt, and zoom positions.
        """
        return (self.pan.getRemainingPosition(), 
                self.tilt.getRemainingPosition(), 
                self.zoom.getRemainingPosition())
    
    def isInDesiredPosition(self):
        """
        Checks if the PTZ is in the desired position.

        This method checks if the current position of the pan, tilt, and zoom
        joints are within their respective desired positions.
        
        Returns:
            bool: True if all joints are in their desired positions, False otherwise.
        """
        return (self.pan.isInDesiredPosition() and 
                self.tilt.isInDesiredPosition() and 
                self.zoom.isInDesiredPosition())
    
    def sendPtzCommand(self, pan = None, tilt = None, zoom = None):
        """
        Sends the PTZ command to the camera.

        This method sends the current desired position of the PTZ to the camera.
        If a specific pan, tilt, or zoom value is provided, it will override the
        current desired position for that axis.
        """
        self.setDesiredPtzPosition(pan, tilt, zoom)

        control = self.controller.sendPTZCommand(
            self.pan._real_desired_position,
            self.tilt._real_desired_position,
            self.zoom._real_desired_position
        )

        if control['status'] != 204 and not control['exception']:
            msg = 'sendPTZCommand: Error getting response. url = %s%s'% (self.hostname, control['url'])
            return False, msg
        elif control['exception']:
            msg = 'sendPTZCommand: Exception connecting to the camera: %s '% (control['error_msg'])
            return False, msg
        else:
            return True, 'PTZ command sent successfully to %s' % (self.hostname)
        
    def sendPtzDesiredVelocityCommand(self):
        return self.sendPtzVelocityCommand(
            self.pan.getDesiredVelocity(),
            self.tilt.getDesiredVelocity(),
            self.zoom.getDesiredVelocity()
        )

    def sendPtzVelocityCommand(self, pan = 0, tilt = 0, zoom = 0):
        self.setDesiredVelocity(pan, tilt, zoom)

        control = self.controller.sendPTZVelocityCommand(
            self.pan.getDesiredVelocity(),
            self.tilt.getDesiredVelocity(),
            self.zoom.getDesiredVelocity()
        )

        if control['status'] != 204 and not control['exception']:
            msg = 'sendPtzVelocityCommand: Error getting response. url = %s%s'% (self.hostname, control['url'])
            return False, msg
        elif control['exception']:
            msg = 'sendPtzVelocityCommand: Exception connecting to the camera: %s '% (control['error_msg'])
            return False, msg
        else:
            return True, 'PTZ command sent successfully to %s' % (self.hostname)

    def stopPtzVelocityCommand(self):
        return self.sendPtzVelocityCommand(0, 0, 0)

class AxisPtz(Node):
    """
      Provides interfaces for controlling PTZ of supported Axis cameras. 
    """

    def __init__(self):
        super().__init__('axis_ptz_node')

        self.rosReadParams()
        self.current_goal = None
        self.feedback = SetPtz.Feedback()
        self.run_control = True
        self.action_result = SetPtz.Result()
        self.command_sent = False
        self.idle = "idle"
        self.control_mode = self.idle

        # Timer to get/release ptz control
        if self.use_control_timeout:
            self.duration_command_timeout = rclpy.time.Duration(seconds=self.control_timeout_value)

        self.time_last_command_received = self.get_clock().now()
        self.duration_last_command_watchdog = rclpy.time.Duration(seconds=10.0)

        self.default_control_mode = PtzMsg.POSITION

        self.rosSetup()
        self.ptz.updatePtzPosition()
        # self.timer = self.create_timer(1/self.desired_freq, self.controlLoop)
        self.update_thread = Thread(target = self.controlLoop, daemon = True)
        self.update_thread.start()
           
    def readParam(self, param_name, default_value):
        """ Reads a parameter value from the node's parameters. """
        self.declare_parameter(param_name, default_value)
        return self.getParameterValue(self.get_parameter(param_name).get_parameter_value())
    
    def getParameterValue(self, parameter_value):
        param = None
        if rclpy.Parameter.Type.BOOL.value == parameter_value.type:
            param = parameter_value.bool_value
        elif rclpy.Parameter.Type.INTEGER.value == parameter_value.type:
            param = parameter_value.integer_value
        elif rclpy.Parameter.Type.DOUBLE.value == parameter_value.type:
            param = parameter_value.double_value
        elif rclpy.Parameter.Type.STRING.value == parameter_value.type:
            param = parameter_value.string_value
        elif rclpy.Parameter.Type.BYTE_ARRAY.value == parameter_value.type:
            param = parameter_value.byte_array_value
        elif rclpy.Parameter.Type.BOOL_ARRAY.value == parameter_value.type:
            param = parameter_value.bool_array_value
        elif rclpy.Parameter.Type.INTEGER_ARRAY.value == parameter_value.type:
            param = parameter_value.integer_array_value
        elif rclpy.Parameter.Type.DOUBLE_ARRAY.value == parameter_value.type:
            param = parameter_value.double_array_value
        elif rclpy.Parameter.Type.STRING_ARRAY.value == parameter_value.type:
            param = parameter_value.string_array_value
        return param

    def rosReadParams(self):
        self.hostname = self.readParam('hostname', '192.168.0.185')
        self.camera_id = self.readParam('camera_id', 1)
        self.camera_model = self.readParam('camera_model', 'axis_m5525')
        self.desired_freq = self.readParam('desired_freq', 20.0)
        pan = Joint(
            self.readParam('pan.min_value', -PI), 
            self.readParam('pan.max_value', PI), 
            self.readParam('pan.joint', 'axis_pan_joint'),
            self.readParam('pan.offset', 0.5),
            self.readParam('pan.error_pos', 0.02),
            invert = self.readParam('pan.invert', False)
        )
        tilt = Joint(
            self.readParam('tilt.min_value', 0.0), 
            self.readParam('tilt.max_value', 1.50), 
            self.readParam('tilt.joint', 'axis_tilt_joint'),
            self.readParam('tilt.offset', 0.0),
            self.readParam('tilt.error_pos', 0.02),
            invert = self.readParam('tilt.invert', False)
        )
        zoom = ZoomJoint(
            self.readParam('zoom.min_value', 1.0), 
            self.readParam('zoom.max_value', 9999.0), 
            self.readParam('zoom.joint', 'axis_zoom_joint'),
            self.readParam('zoom.offset', 1000),
            self.readParam('zoom.error_pos', 99.0),
            self.readParam('zoom.min_augment', 0.0),
            self.readParam('zoom.max_augment', 30.0),
            self.readParam('zoom.min_step', 1.0)
        )

        self.ptz = Ptz(self.hostname, self.camera_id, pan, tilt, zoom)

        self.use_control_timeout = self.readParam('use_control_timeout', False)
        self.control_timeout_value = self.readParam('control_timeout_value', 0.5)
        self.send_constantly = self.readParam('send_constantly', False)

    def rosSetup(self):
        self.set_ptz_action_server = ActionServer(
            self,
            SetPtz,
            '~/set_ptz',
            self.setPtzExecuteCb,
            goal_callback = self.setPtzGoalCb,
            handle_accepted_callback = self.setPtzAcceptedCb,
            cancel_callback = self.cancelPtzCb,
            callback_group = ReentrantCallbackGroup()
        )

        self.home_action_server = ActionServer(
            self,
            SetPtz,
            '~/home_ptz',
            self.setPtzExecuteCb,
            handle_accepted_callback = self.homeCb,
            cancel_callback = self.cancelHomeCb,
            callback_group = ReentrantCallbackGroup()
        )

        self.joint_state_pub = self.create_publisher(JointState, '~/joint_states', 1)
        self.axis_status_pub = self.create_publisher(Axis, '~/status', 1)
        self.axis_status_raw_pub = self.create_publisher(Axis, '~/status_raw', 1)

    def controlLoop(self):
        rate = self.create_rate(self.desired_freq)
        while rclpy.ok():
            if self.ptz.isSyncronized() and (self.send_constantly or not self.command_sent):
                if self.control_mode == PtzMsg.POSITION:
                    self.sendPtzCommand()
                elif self.control_mode == PtzMsg.VELOCITY:
                    self.sendPtzVelocityCommand()

            self.ptz.updatePtzPosition()
            # Publish ROS msgs
            self.publishROS()
            rate.sleep()

    def getPtzDesiredPositionFromGoal(self, goal):
        if goal.ptz.relative:
            new_pan = self.ptz.pan.getCurrentPosition() + goal.ptz.pan
            new_tilt = self.ptz.tilt.getCurrentPosition() + goal.ptz.tilt
            new_zoom = round(self.ptz.zoom.getNormalizedPosition()+ goal.ptz.zoom)
        else:
            new_pan = goal.ptz.pan
            new_tilt = goal.ptz.tilt
            new_zoom = goal.ptz.zoom
        
        return new_pan, new_tilt, new_zoom

    def setPtzDesiredPosition(self, pan=None, tilt=None, zoom=None, current_position = False):
        """
        Sets the desired position for the PTZ camera.

        This method updates the desired position of the PTZ camera based on the provided pan, tilt, and zoom values.
        If a value is None, it will not update that axis.

        Args:
            pan (float): The desired pan position.
            tilt (float): The desired tilt position.
            zoom (float): The desired zoom position.
            current_position (bool): If True, sets the current position as the desired position.
        """
        if current_position:
            self.ptz.setCurrentPtzPositionAsDesired()
        else:
            self.ptz.setDesiredPtzPosition(pan, tilt, zoom)
        self.command_sent = False

    def setPtzDesiredVelocity(self, pan=0, tilt=0, zoom=0):
        """
        Sets the desired velocity for the PTZ camera.

        This method updates the desired velocity of the PTZ camera based on the provided pan, tilt, and zoom values.
        If a value is None, it will not update that axis.

        Args:
            pan (float): The desired pan velocity.
            tilt (float): The desired tilt velocity.
            zoom (float): The desired zoom velocity.
        """
        self.ptz.setDesiredVelocity(pan, tilt, zoom)
        self.command_sent = False

    def sendPtzVelocityCommand(self):
        success, msg = self.ptz.sendPtzDesiredVelocityCommand()
        if not success:
            self.get_logger().error(msg)
        else:
            self.command_sent = True

    def sendPtzCommand(self, pan = None, tilt = None, zoom = None):
        """
        Sends the PTZ command to the camera.
        """
        success, msg = self.ptz.sendPtzCommand(pan, tilt, zoom)
        if not success:
            self.get_logger().error(msg)
        else:
            self.command_sent = True

    def publishROS(self):
        """ Publishes the current state of the PTZ camera. """
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = [
            self.ptz.pan.getName(),
            self.ptz.tilt.getName(),
            self.ptz.zoom.getName()
        ]

        joint_state_msg.position = [
            self.ptz.pan.getCurrentPosition(),
            self.ptz.tilt.getCurrentPosition(),
            float(round(self.ptz.zoom.getNormalizedPosition()))
        ]

        joint_state_msg.velocity = [
            0.0,
            0.0,
            0.0
        ]
        joint_state_msg.effort = [
            0.0,
            0.0,
            0.0
        ]

        axis_status_raw_msg = Axis()
        axis_status_raw_msg.pan = self.ptz.pan.getRawCurrentPosition()
        axis_status_raw_msg.tilt = self.ptz.tilt.getRawCurrentPosition()
        axis_status_raw_msg.zoom = self.ptz.zoom.getRawCurrentPosition()
        axis_status_raw_msg.iris = self.ptz.iris
        axis_status_raw_msg.autoiris = self.ptz.autoiris
        axis_status_raw_msg.focus = self.ptz.focus
        axis_status_raw_msg.autofocus = self.ptz.autofocus

        axis_status_msg = deepcopy(axis_status_raw_msg)
        axis_status_msg.pan = self.ptz.pan.getCurrentPosition()
        axis_status_msg.tilt = self.ptz.tilt.getCurrentPosition()
        axis_status_msg.zoom = self.ptz.zoom.getCurrentPosition()

        self.joint_state_pub.publish(joint_state_msg)
        self.axis_status_pub.publish(axis_status_msg)
        self.axis_status_raw_pub.publish(axis_status_raw_msg)

    def setPtzGoalCb(self, goal_handle : ServerGoalHandle):
        """ Callback for the SetPtz action. """
        if goal_handle.ptz.mode not in [PtzMsg.POSITION, PtzMsg.VELOCITY]:
            return GoalResponse.REJECT
        else:
            return GoalResponse.ACCEPT

    def setPtzAcceptedCb(self, goal_handle : ServerGoalHandle):
        self.control_mode = goal_handle.request.ptz.mode
        pan, tilt, zoom = self.getPtzDesiredPositionFromGoal(goal_handle.request)
        self.handleGoal(goal_handle, pan, tilt, zoom)

    def cancelPtzCb(self, goal_handle : ServerGoalHandle):
        """ Callback for cancelling the SetPtz action. """
        self.handleCancel()
        self.get_logger().info('Cancelling PTZ action')
        return CancelResponse.ACCEPT
    
    def homeCb(self, goal_handle : ServerGoalHandle):
        """ Callback for the Home action. """
        self.control_mode = PtzMsg.POSITION
        self.handleGoal(goal_handle, 0.0, 0.0, 0.0)

    def cancelHomeCb(self, goal_handle : ServerGoalHandle):
        """ Callback for cancelling the Home action. """
        self.handleCancel()
        self.get_logger().info('Cancelling Home action')
        return CancelResponse.ACCEPT
    
    def handleGoal(self, goal_handle : ServerGoalHandle, pan, tilt, zoom):
        self.time_last_command_received = self.get_clock().now()
        self.current_goal = goal_handle
        self.control_mode = goal_handle.request.ptz.mode
        if self.control_mode == PtzMsg.POSITION:
            self.setPtzDesiredPosition(pan, tilt, zoom)
        elif self.control_mode == PtzMsg.VELOCITY:
            self.setPtzDesiredVelocity(pan, tilt, zoom)
        self.current_goal.execute()

    def handleCancel(self):
        self.current_goal = None
        if self.control_mode == PtzMsg.POSITION:
            self.setPtzDesiredPosition(current_position = True)
        elif self.control_mode == PtzMsg.VELOCITY:
            self.setPtzDesiredVelocity(0, 0, 0)
        self.action_result.response.success = False
        self.action_result.response.message = 'PTZ action cancelled'
        self.control_mode = self.idle

    def setPtzExecuteCb(self, goal_handle : ServerGoalHandle):
        """
        Execute callback for the SetPtz action.

        This method is called when a new goal is received. It sets the desired PTZ position
        based on the goal and starts the control loop.
        
        Args:
            goal_handle (ServerGoalHandle): The handle for the goal.
        
        Returns:
            SetPtz.Result: The result of the action execution.
        """
        while self.current_goal is not None and rclpy.ok():
            # If goal has been reached or there is no goal -> control mode is idle
            if self.ptz.isInDesiredPosition():
                self.current_goal.succeed()
                self.current_goal = None
                self.action_result.response.success = True
                self.action_result.response.message = 'PTZ position reached successfully'
                break

            # If timeout has been reached, abort the goal
            elif self.get_clock().now() - self.time_last_command_received > self.duration_last_command_watchdog:
                self.current_goal.abort()
                self.current_goal = None
                self.action_result.response.success = False
                self.action_result.response.message = 'PTZ position not reached in time'
                break

            self.publishFeedback(self.current_goal)
        
        return self.action_result

    def publishFeedback(self, goal_handle: ServerGoalHandle):
        if goal_handle is None:
            return
        
        feedback = SetPtz.Feedback()
        feedback.remaining_pan, feedback.remaining_tilt, feedback.remaining_zoom = self.ptz.getRemainingPtzPosition()

        goal_handle.publish_feedback(feedback)
