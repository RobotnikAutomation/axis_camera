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
        self.min = min_position
        self.max = max_position
        self.name = joint_name
        self.offset = offset
        self.error = error
        self.invert = -1 if invert else 1
        self.current_position = 0.0
        self.raw_current_position = 0.0
        self.desired_position = 0.0
        self.real_desired_position = self.offset
    
    def updatePosition(self, position):
        """
        Update the current position of the joint.

        This function takes the position received from the real camera
        and transforms it to the position of the camera frame.
        
        Args:
            position (float): The position received from the real camera.
        """
        self.raw_current_position = position
        self.current_position = self.invert * self.normalizeAngle(position - self.offset)
    
    def setDesiredPosition(self, position):
        """
        Set the desired position of the camera.

        This method transforms the desired position in reference to the camera frame
        into the real position of the camera. The transformation is done by applying
        an inversion factor and an offset.

        Args:
            position (float): The desired position in reference to the camera frame.
        """
        self.desired_position = self.enforceLimits(position)
        self.real_desired_position = self.invert * rad2deg(self.desired_position + self.offset)

    def enforceLimits(self, position):
        """
        Enforces the limits of the joint position.

        This method ensures that the position is within the defined limits
        of the joint. If the position is outside the limits, it will be
        clamped to the nearest limit.

        Args:
            position (float): The position to enforce limits on.

        Returns:
            float: The enforced position within the limits.
        """
        return max(self.min, min(self.max, position))

    def normalizeAngle(self, angle) -> float:
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
        self.setDesiredPosition(self.current_position)

    def isInDesiredPosition(self):
        """
        Checks if the current position is within the desired position range.

        Returns:
            bool: True if the current position is within the desired range, False otherwise.
        """
        return abs(self.current_position - self.desired_position) <= self.error

    def getRemainingPosition(self):
        """
        Returns the remaining position to reach the desired position.

        This method calculates the difference between the desired position and the current position.

        Returns:
            float: The remaining position to reach the desired position.
        """
        return self.desired_position - self.current_position

class ZoomJoint(Joint):
    def __init__(self, min_position, max_position, joint_name, offset, error, min_augment, max_augment, min_step):
        super().__init__(min_position, max_position, joint_name, offset, error, invert = False)
        self.min_augment = min_augment
        self.max_augment = max_augment
        self.min_step = min_step
    
    def updatePosition(self, position):
        self.raw_current_position = position
        self.current_position = position

    def setCurrentPositionAsDesired(self):
        self.desired_position = self.current_position
        self.real_desired_position = self.enforceLimits(self.desired_position)
    
    def setDesiredPosition(self, position):
        """
        Set the desired position of the zoom joint. This should be a value between the minimum and maximum augment values.
        """
        position = max(self.min_augment, min(self.max_augment, position))
        self.desired_position = position / (self.max_augment - self.min_augment) * (self.max - self.min) + self.min
        self.real_desired_position = self.enforceLimits(self.desired_position)

    def getNormalizedPosition(self):
        """
        Returns the normalized position of the zoom joint.

        The normalized position is calculated based on the current position,
        the minimum and maximum augment values, and the minimum step.
        """
        return (self.current_position - self.min) / (self.max - self.min) * (self.max_augment - self.min_augment)

class Ptz:
    def __init__(self, hostname : str, camera_id : int, pan : Joint, tilt : Joint, zoom : ZoomJoint):
        self.controller = ControlAxis(hostname, camera_id)
        self.hostname = hostname
        self.pan = pan
        self.tilt = tilt
        self.zoom = zoom
        self.iris = 0.0
        self.autoiris = False
        self.focus = 0.0
        self.autofocus = False
        self._is_syncronized = False
    
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
            self.pan.real_desired_position,
            self.tilt.real_desired_position,
            self.zoom.real_desired_position
        )

        if control['status'] != 204 and not control['exception']:
            msg = 'sendPTZCommand: Error getting response. url = %s%s'% (self.hostname, control['url'])
            return False, msg
        elif control['exception']:
            msg = 'sendPTZCommand: Exception connecting to the camera: %s '% (control['error_msg'])
            return False, msg
        else:
            return True, 'PTZ command sent successfully to %s' % (self.hostname)
        
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
        self.idle = "IDLE"
        self.control_mode = self.idle
        self.action_result = SetPtz.Result()

        # Timer to get/release ptz control
        if self.use_control_timeout:
            self.duration_command_timeout = rclpy.time.Duration(seconds=self.control_timeout_value)

        self.time_last_command_received = self.get_clock().now()
        self.duration_last_command_watchdog = rclpy.time.Duration(seconds=10.0)

        self.time_idle_state_position_update = self.get_clock().now()
        self.duration_idle_state_position_update = rclpy.time.Duration(seconds=10.0)

        self.default_control_mode = PtzMsg.POSITION

        self.rosSetup()
        self.ptz.updatePtzPosition()
        # self.timer = self.create_timer(1/self.desired_freq, self.controlLoop)
        self.update_thread = Thread(target = self.controlLoop)
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
        self.autoflip = self.readParam('autoflip', False)
        self.eflip = self.readParam('eflip', False)
        pan = Joint(
            self.readParam('pan.min_value', -PI), 
            self.readParam('pan.max_value', PI), 
            self.readParam('pan.joint', 'axis_pan_joint'),
            self.readParam('pan.offset', 0.0),
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
            self.readParam('zoom.offset', 0),
            self.readParam('zoom.error_pos', 99.0),
            self.readParam('zoom.min_augment', 0.0),
            self.readParam('zoom.max_augment', 30.0),
            self.readParam('zoom.min_step', 1.0)
        )

        self.ptz = Ptz(self.hostname, self.camera_id, pan, tilt, zoom)

        self.use_control_timeout = self.readParam('use_control_timeout', False)
        self.control_timeout_value = self.readParam('control_timeout_value', 0.5)
        self.send_constantly = self.readParam('send_constantly', True)

    def rosSetup(self):
        self.set_ptz_action_server = ActionServer(
            self,
            SetPtz,
            '~/set_ptz',
            self.setPtzExecuteCb,
            handle_accepted_callback = self.setPtzCb,
            cancel_callback=self.cancelPtzCb,
            callback_group=ReentrantCallbackGroup()
        )

        self.home_action_server = ActionServer(
            self,
            SetPtz,
            '~/home_ptz',
            self.setPtzExecuteCb,
            handle_accepted_callback = self.homeCb,
            cancel_callback=self.cancelHomeCb,
            callback_group=ReentrantCallbackGroup()
        )

        self.joint_state_pub = self.create_publisher(JointState, '~/joint_states', 1)
        self.axis_status_pub = self.create_publisher(Axis, '~/status', 1)

    def controlLoop(self):
        rate = self.create_rate(self.desired_freq)
        while rclpy.ok():
            if self.ptz.isSyncronized() and self.send_constantly:
                success, msg = self.ptz.sendPtzCommand()
                if not success:
                    self.get_logger().error(msg)
            self.ptz.updatePtzPosition()
            # Publish ROS msgs
            self.publishROS()
            rate.sleep()

    def manageControl(self):
        """
            Gets/releases ptz control using a timeout
        """
        self.run_control = (self.get_clock().now() - self.time_last_command_received) < self.duration_command_timeout

    def controlPTZ(self):
        time_now = self.get_clock().now()

        # If goal has been reached or there is no goal -> control mode is idle
        if self.current_goal is not None:
            if self.ptz.isInDesiredPosition():
                self.control_mode = self.idle
                self.current_goal.succeed()
                self.current_goal = None
                self.action_result.response.success = True
                self.action_result.response.message = 'PTZ position reached successfully'

            # If timeout has been reached, abort the goal
            elif time_now - self.time_last_command_received > self.duration_last_command_watchdog:
                self.control_mode = self.idle
                self.current_goal.abort()
                self.current_goal = None
                self.action_result.response.success = False
                self.action_result.response.message = 'PTZ position not reached in time'
        else:
            self.ptz.setCurrentPtzPositionAsDesired()
            self.control_mode = self.idle

        # If control mode is not idle, move ptz to desired position
        if self.control_mode == PtzMsg.POSITION or self.control_mode == PtzMsg.VELOCITY:
            if self.ptz.isSyncronized() and self.send_constantly:
                success, msg = self.ptz.sendPtzCommand()
                if not success:
                    self.get_logger().error(msg)
            self.publishFeedback(self.current_goal)
        elif time_now - self.time_idle_state_position_update > self.duration_idle_state_position_update:
            # In idle state, update the desired position to the current one
            self.ptz.setCurrentPtzPositionAsDesired()
            self.time_idle_state_position_update = time_now
    
    def getPtzDesiredPositionFromGoal(self, goal):
        if goal.ptz.relative:
            new_pan = self.ptz.pan.current_position + goal.ptz.pan
            new_tilt = self.ptz.tilt.current_position + goal.ptz.tilt
            new_zoom = round(self.ptz.zoom.getNormalizedPosition()) + goal.ptz.zoom
        else:
            new_pan = goal.ptz.pan
            new_tilt = goal.ptz.tilt
            new_zoom = goal.ptz.zoom
        
        return new_pan, new_tilt, new_zoom

    def setPtzDesiredPosition(self, pan=None, tilt=None, zoom=None):
        self.ptz.setDesiredPtzPosition(pan, tilt, zoom)

    def publishROS(self):
        """ Publishes the current state of the PTZ camera. """
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = [
            self.ptz.pan.name,
            self.ptz.tilt.name,
            self.ptz.zoom.name
        ]

        joint_state_msg.position = [
            self.ptz.pan.current_position,
            self.ptz.tilt.current_position,
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

        axis_status_msg = Axis()
        axis_status_msg.pan = self.ptz.pan.current_position
        axis_status_msg.tilt = self.ptz.tilt.current_position
        axis_status_msg.zoom = self.ptz.zoom.current_position
        axis_status_msg.iris = self.ptz.iris
        axis_status_msg.autoiris = self.ptz.autoiris
        axis_status_msg.focus = self.ptz.focus
        axis_status_msg.autofocus = self.ptz.autofocus

        self.joint_state_pub.publish(joint_state_msg)
        self.axis_status_pub.publish(axis_status_msg)

    def setPtzCb(self, goal_handle : ServerGoalHandle):
        """ Callback for the SetPtz action. """
        if goal_handle.request.ptz.mode not in [PtzMsg.POSITION, PtzMsg.VELOCITY]:
            return GoalResponse.REJECT

        pan, tilt, zoom = self.getPtzDesiredPositionFromGoal(goal_handle.request)
        self.handleGoal(goal_handle, pan, tilt, zoom)
        return GoalResponse.ACCEPT

    def cancelPtzCb(self, goal_handle : ServerGoalHandle):
        """ Callback for cancelling the SetPtz action. """
        self.handleCancel()
        self.get_logger().info('Cancelling PTZ action')
        return CancelResponse.ACCEPT
    
    def homeCb(self, goal_handle : ServerGoalHandle):
        """ Callback for the Home action. """
        self.handleGoal(goal_handle, 0.0, 0.0, 0.0)
        return GoalResponse.ACCEPT

    def cancelHomeCb(self, goal_handle : ServerGoalHandle):
        """ Callback for cancelling the Home action. """
        self.handleCancel()
        self.get_logger().info('Cancelling Home action')
        return CancelResponse.ACCEPT
    
    def handleGoal(self, goal_handle : ServerGoalHandle, pan, tilt, zoom):
        self.time_last_command_received = self.get_clock().now()
        self.current_goal = goal_handle
        self.control_mode = goal_handle.request.ptz.mode
        self.ptz.setDesiredPtzPosition(pan, tilt, zoom)
        self.current_goal.execute()
        if not self.send_constantly:
            success, msg = self.ptz.sendPtzCommand()
            if not success:
                self.get_logger().error(msg)

    def handleCancel(self):
        self.current_goal = None
        self.control_mode = self.idle
        self.ptz.setCurrentPtzPositionAsDesired()
        if not self.send_constantly:
            success, msg = self.ptz.sendPtzCommand()
            if not success:
                self.get_logger().error(msg)
    
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
        while self.current_goal is not None:
            self.controlPTZ()
        
        return self.action_result

    def publishFeedback(self, goal_handle):
        if goal_handle is None:
            return
        
        feedback = SetPtz.Feedback()
        feedback.remaining_pan, feedback.remaining_tilt, feedback.remaining_zoom = self.ptz.getRemainingPtzPosition()

        goal_handle.publish_feedback(feedback)
