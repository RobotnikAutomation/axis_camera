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
from copy import deepcopy

from threading import Thread

import rclpy
import rclpy.time
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup

from axis_camera.axis_lib.ptz import Ptz
from axis_camera.axis_lib.joints import Joint, ZoomJoint

from robotnik_actuators_msgs.msg import Ptz as PtzMsg
from robotnik_actuators_msgs.action import SetPtz
from robotnik_sensors_msgs.msg import Axis
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist

from std_srvs.srv import Trigger

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
        self.previous_velocity = Twist()

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
            self.readParam('zoom.offset', 0.0),
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
            goal_callback = self.setPtzGoalCb,
            handle_accepted_callback = self.homeCb,
            cancel_callback = self.cancelHomeCb,
            callback_group = ReentrantCallbackGroup()
        )

        self.velocity_sub = self.create_subscription(
            Twist,
            '~/velocity',
            self.velocityCb,
            1
        )

        self.stop_velocity_control_service = self.create_service(
            Trigger,
            '~/stop_velocity_control',
            self.stopVelocityControlCb
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

    def switchToControlState(self, new_state : str):
        """
        Switches the control mode of the PTZ camera.

        This method changes the control mode to the specified new state.
        It also resets the desired position and velocity to the current position
        when switching to idle mode.
        
        Args:
            new_state (str): The new control state to switch to.
        """
        if self.control_mode == new_state:
            return
        
        if self.idle not in [new_state, self.control_mode]:
            self.get_logger().error(f'Cannot switch to {new_state} control mode from {self.control_mode} mode. Please, stop current control first.')
            return
        
        if self.control_mode == PtzMsg.POSITION and new_state == self.idle:
            self.switchFromPositionToIdle()
        elif self.control_mode == PtzMsg.VELOCITY and new_state == self.idle:
            self.switchFromVelocityToIdle()
        else:
            self.control_mode = new_state

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
    
    def switchFromVelocityToIdle(self):
        """
        Stops the velocity control of the PTZ camera.

        This method sets the control mode to idle and resets the desired velocity.
        """
        self.control_mode = self.idle
        self.setPtzDesiredVelocity(0, 0, 0)
        self.sendPtzVelocityCommand()
    
    def velocityCb(self, msg: Twist):
        """
        Callback for the velocity topic.

        This method is called when a new velocity message is received. It sets the desired velocity
        for the PTZ camera based on the received message.
        
        Args:
            msg (Twist): The velocity message containing pan, tilt, and zoom velocities.
        """

        # If the camera is being controlled, we cannot set the velocity
        if self.control_mode == PtzMsg.POSITION:
            self.logger().error(f'Cannot set velocity when the camera is being controlled (control_mode = {self.control_mode}). Please, stop current control first.',
                                 throttle_duration_sec = 5.0
                                )
            return
        
        # If the velocity is zero, we set the control mode to idle
        if msg.linear.x == 0.0 and msg.linear.y == 0.0 and msg.angular.z == 0.0:
            self.previous_velocity = msg
            self.switchToControlState(self.idle)
            return
        
        # If the velocity is the same as the previous one, we do not send the command
        if msg == self.previous_velocity:
            return
        
        # If the velocity is different, we set the control mode to velocity and send the command
        self.previous_velocity = msg
        self.setPtzDesiredVelocity(msg.linear.x, msg.linear.y, msg.angular.z)

    def stopVelocityControlCb(self, request : Trigger.Request, response : Trigger.Response):
        """
        Callback for the stop velocity control service.

        This method is called when the stop velocity control service is requested. It stops the velocity control
        of the PTZ camera by setting the control mode to idle and resetting the desired velocity.
        
        Args:
            request: The service request.
            response: The service response.
        
        Returns:
            Trigger.Response: The response indicating success or failure.
        """
        # Reinit previous velocity to avoid sending the same command again
        self.previous_velocity = Twist()
        self.switchToControlState(self.idle)
        response.success = True
        response.message = 'Velocity control stopped successfully'
        return response
    
    def switchFromPositionToIdle(self):
        """
        Stops the position control of the PTZ camera.

        This method sets the control mode to idle and resets the desired position.
        """
        self.control_mode = self.idle
        self.setPtzDesiredPosition(current_position = True)
        self.sendPtzCommand()

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
        if self.control_mode != self.idle:
            self.get_logger().error(f'Cannot set a new position goal when the camera is being controlled (control_mode = {self.control_mode}). Please, stop current control first.')
            return GoalResponse.REJECT
        else:
            self.switchToControlState(PtzMsg.POSITION)
            return GoalResponse.ACCEPT

    def setPtzAcceptedCb(self, goal_handle : ServerGoalHandle):
        pan, tilt, zoom = self.getPtzDesiredPositionFromGoal(goal_handle.request)
        self.handleGoal(goal_handle, pan, tilt, zoom)

    def cancelPtzCb(self, goal_handle : ServerGoalHandle):
        """ Callback for cancelling the SetPtz action. """
        self.handleCancel()
        self.get_logger().info('Cancelling PTZ action')
        return CancelResponse.ACCEPT
    
    def homeCb(self, goal_handle : ServerGoalHandle):
        """ Callback for the Home action. """
        self.handleGoal(goal_handle, 0.0, 0.0, 0.0)

    def cancelHomeCb(self, goal_handle : ServerGoalHandle):
        """ Callback for cancelling the Home action. """
        self.handleCancel()
        self.get_logger().info('Cancelling Home action')
        return CancelResponse.ACCEPT
    
    def handleGoal(self, goal_handle : ServerGoalHandle, pan, tilt, zoom):
        self.time_last_command_received = self.get_clock().now()
        self.current_goal = goal_handle
        self.setPtzDesiredPosition(pan, tilt, zoom)
        self.current_goal.execute()

    def handleCancel(self):
        self.current_goal = None
        self.switchToControlState(self.idle)
        self.action_result.response.success = False
        self.action_result.response.message = 'PTZ action cancelled'

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
        
        self.switchToControlState(self.idle)
        return self.action_result

    def publishFeedback(self, goal_handle: ServerGoalHandle):
        if goal_handle is None:
            return
        
        feedback = SetPtz.Feedback()
        feedback.remaining_pan, feedback.remaining_tilt, feedback.remaining_zoom = self.ptz.getRemainingPtzPosition()

        goal_handle.publish_feedback(feedback)
