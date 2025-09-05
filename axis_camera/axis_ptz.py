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
import time

import rclpy
import rclpy.time
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup

from axis_camera.axis_lib.ptz import Ptz
from axis_camera.axis_lib.joints import Joint, ZoomJoint

from robotnik_actuators_msgs.action import SetPtz
from robotnik_sensors_msgs.msg import Axis
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist

from std_srvs.srv import Trigger

class AxisPtz(Node):
    """
      Provides interfaces for controlling PTZ of supported Axis cameras. 
    """
    IDLE = "idle"
    POSITION = "position"
    VELOCITY = "velocity"

    def __init__(self):
        super().__init__('axis_ptz_node')

        self.rosReadParams()
        self.current_goal = None
        self.feedback = SetPtz.Feedback()
        self.run_control = True
        self.action_result = SetPtz.Result()
        self.command_sent = False
        self.control_mode = self.IDLE
        self.previous_velocity = Twist()

        self.time_last_command_received = self.get_clock().now()
        self.last_time_moving = None

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
        """
        Converts the parameter value to its corresponding Python type.
        Args:
            parameter_value (rclpy.ParameterValue): The value of the parameter.
        Returns:
            The value of the parameter converted to its corresponding Python type.
        """
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
        """ Reads parameters from the ROS parameter server."""
        self.hostname = self.readParam('hostname', '192.168.0.185')
        self.camera_number = self.readParam('camera_number', 1)
        self.desired_freq = self.readParam('desired_freq', 20.0)
        self.connection_timeout = self.readParam('connection_timeout', 5.0)
        pan = Joint(
            self.readParam('pan.min_value', -PI), 
            self.readParam('pan.max_value', PI), 
            self.readParam('pan.joint', 'axis_pan_joint'),
            self.readParam('pan.offset', 0.0),
            self.readParam('pan.error_pos', 0.01),
            invert = self.readParam('pan.invert', False)
        )
        tilt = Joint(
            self.readParam('tilt.min_value', 0.0), 
            self.readParam('tilt.max_value', PI/2),
            self.readParam('tilt.joint', 'axis_tilt_joint'),
            self.readParam('tilt.offset', 0.0),
            self.readParam('tilt.error_pos', 0.01),
            invert = self.readParam('tilt.invert', False)
        )
        zoom = ZoomJoint(
            self.readParam('zoom.min_value', 1.0), 
            self.readParam('zoom.max_value', 9999.0),
            self.readParam('zoom.joint', 'axis_zoom_joint'),
            self.readParam('zoom.offset', 0.0),
            self.readParam('zoom.error_pos', 99.0),
            self.readParam('zoom.min_augment', 0.0),
            self.readParam('zoom.max_augment', 30.0)
        )

        ptz_connected = False
        while not ptz_connected and rclpy.ok():
            try:
                self.get_logger().info(f'Connecting to PTZ camera {self.hostname}...')
                # Initialize the PTZ camera with the provided parameters
                self.ptz = Ptz(self.hostname, self.camera_number, pan, tilt, zoom, self.connection_timeout)
                ptz_connected = True
                self.get_logger().info(f'Successfully connected to PTZ camera {self.hostname}')
            except Exception as e:
                self.get_logger().error(f'Error connecting to PTZ camera: {e}')

        camera_not_moving_timeout_value = self.readParam('camera_not_moving_timeout_value', 3.0)
        self.camera_not_moving_timeout = rclpy.time.Duration(seconds=camera_not_moving_timeout_value)
        duration_last_command_watchdog_value = self.readParam('duration_last_command_watchdog_value', 10.0)
        self.duration_last_command_watchdog = rclpy.time.Duration(seconds=duration_last_command_watchdog_value)
        self.reject_new_goal = self.readParam('reject_new_goal', False)

    def rosSetup(self):
        """
        Sets up the ROS interfaces for the PTZ camera.
        This method initializes the action servers, publishers, and subscribers
        required for controlling the PTZ camera.
        """
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
            cancel_callback = self.cancelPtzCb,
            callback_group = ReentrantCallbackGroup()
        )

        if self.ptz.hasVelocityControl():
            self.velocity_sub = self.create_subscription(
                Twist,
                '~/cmd_vel',
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

    def handlePtzStoppedMoving(self):
        if self.ptz.isMoving():
            self.last_time_moving = self.get_clock().now()
            return

        if self.control_mode == self.IDLE:
            self.last_time_moving = None

        # If the camera is not moving and the control mode is not idle, we set control mode to idle
        elif self.last_time_moving and \
            (self.get_clock().now() - self.last_time_moving > self.camera_not_moving_timeout):

            self.get_logger().info(f'PTZ camera is not moving, switching to idle mode')
            if self.current_goal is not None and self.current_goal.is_active: #Pos control
                self.abortAction(self.current_goal, 'PTZ camera stopped moving')
            else: #Vel control
                self.switchToControlState(self.IDLE)
        else:
            # Try to send the command again while the timeout is not reached
            if self.control_mode == self.POSITION:
                self.sendPtzCommand()
            elif self.control_mode == self.VELOCITY:
                self.sendPtzVelocityCommand()

    def controlLoop(self):
        """
        Main control loop for the PTZ camera.
        This method runs in a separate thread and continuously checks the control mode.
        If the camera is synchronized and the control mode is set to position or velocity,
        it sends the appropriate command to the camera.
        It also publishes the current state of the PTZ camera.
        """
        rate = self.create_rate(self.desired_freq)
        while rclpy.ok():
            if self.ptz.isSyncronized() and not self.command_sent:
                if self.control_mode == self.POSITION:
                    self.sendPtzCommand()
                elif self.control_mode == self.VELOCITY:
                    self.sendPtzVelocityCommand()

            self.handlePtzStoppedMoving()
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
        
        if self.IDLE not in [new_state, self.control_mode]:
            self.get_logger().error(f'Cannot switch to {new_state} control mode from {self.control_mode} mode. Please, stop current control first.')
            return
        
        self.get_logger().info(f'Switching control mode from {self.control_mode} to {new_state}')
        if self.control_mode == self.POSITION and new_state == self.IDLE:
            self.switchFromPositionToIdle()
        elif self.control_mode == self.VELOCITY and new_state == self.IDLE:
            self.switchFromVelocityToIdle()
        else:
            self.control_mode = new_state

    def getPtzDesiredPositionFromGoal(self, goal):
        """
        Gets the desired PTZ position from the action goal.
        This method retrieves the desired pan, tilt, and zoom values from the action goal.
        If the goal specifies relative positions, it adds the current position to the desired position.
        Args:
            goal (SetPtz.Goal): The action goal containing the desired PTZ position.
        Returns:
            tuple: A tuple containing the desired pan, tilt, and zoom positions.
        """
        if goal.ptz.relative:
            new_pan = self.ptz.pan.getCurrentPosition() + goal.ptz.pan
            new_tilt = self.ptz.tilt.getCurrentPosition() + goal.ptz.tilt
            new_zoom = round(self.ptz.zoom.getNormalizedPosition() + goal.ptz.zoom)
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
        """
        Sends the desired velocity command to the PTZ camera.
        This method sends the current desired velocity of the PTZ camera to the camera.
        """
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
        # Reinit previous velocity to let the user send the previous command again
        self.previous_velocity = Twist()
        self.setPtzDesiredVelocity(0, 0, 0)
        self.sendPtzVelocityCommand()
        self.control_mode = self.IDLE
    
    def velocityCb(self, msg: Twist):
        """
        Callback for the velocity topic.

        This method is called when a new velocity message is received. It sets the desired velocity
        for the PTZ camera based on the received message.
        
        Args:
            msg (Twist): The velocity message containing pan, tilt, and zoom velocities.
        """

        # If the camera is being controlled, we cannot set the velocity
        if self.control_mode == self.POSITION:
            self.get_logger().error(f'Cannot set velocity when the camera is being controlled (control_mode = {self.control_mode}). Please, stop current control first.',
                                 throttle_duration_sec = 5.0
                                )
            return
        
        # If the velocity is zero, we set the control mode to idle
        if msg.angular.z == 0.0 and msg.linear.y == 0.0 and msg.linear.x == 0.0:
            self.previous_velocity = msg
            self.switchToControlState(self.IDLE)
            return
        
        # If the velocity is the same as the previous one, we do not send the command
        if msg == self.previous_velocity and self.control_mode == self.VELOCITY:
            return
        
        # If the velocity is different, we set the control mode to velocity and send the command
        self.previous_velocity = msg
        self.setPtzDesiredVelocity(msg.linear.x, msg.linear.y, msg.angular.z)
        self.switchToControlState(self.VELOCITY)

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

        self.switchToControlState(self.IDLE)
        response.success = True
        response.message = 'Velocity control stopped successfully'
        return response
    
    def switchFromPositionToIdle(self):
        """
        Stops the position control of the PTZ camera.

        This method sets the control mode to idle and resets the desired position.
        """
        self.setPtzDesiredPosition(current_position = True)
        self.sendPtzCommand()
        self.control_mode = self.IDLE

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
        axis_status_raw_msg.pan = float(self.ptz.pan.getRawCurrentPosition())
        axis_status_raw_msg.tilt = float(self.ptz.tilt.getRawCurrentPosition())
        axis_status_raw_msg.zoom = float(self.ptz.zoom.getRawCurrentPosition())
        axis_status_raw_msg.iris = float(self.ptz.iris)
        axis_status_raw_msg.autoiris = self.ptz.autoiris
        axis_status_raw_msg.focus = float(self.ptz.focus)
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
        if self.control_mode == self.VELOCITY:
            self.get_logger().error(f'Cannot set a new position goal when the camera is being controlled (control_mode = {self.control_mode}). Please, stop current control first.')
            return GoalResponse.REJECT
        elif self.control_mode == self.IDLE:
            self.get_logger().info(f'Accepting new goal.')
            self.switchToControlState(self.POSITION)
            return GoalResponse.ACCEPT
        else:
            if self.reject_new_goal:
                self.get_logger().error(f'Cannot set a new position goal when the camera is being controlled (control_mode = {self.control_mode}). Please, stop current control first.')
                return GoalResponse.REJECT
            else:
                self.get_logger().info(f'New goal received, aborting previous goal')
                if self.current_goal is not None and self.current_goal.is_active:
                    self.abortAction(self.current_goal, 'New goal received, aborting previous goal')

                # Wait until the control mode is idle
                goal_received_time = self.get_clock().now()
                while self.control_mode != self.IDLE and rclpy.ok() and \
                (self.get_clock().now() - goal_received_time < rclpy.time.Duration(seconds=5.0)):
                    pass

                # If the control mode is idle, accept the goal
                if self.control_mode == self.IDLE:
                    self.switchToControlState(self.POSITION)
                    return GoalResponse.ACCEPT
                else:
                    self.get_logger().error(f'Control mode is not idle after 5 seconds for some unknown reason, rejecting goal')
                    return GoalResponse.REJECT   

    def setPtzAcceptedCb(self, goal_handle : ServerGoalHandle):
        """ Callback for accepting the SetPtz action goal. """
        pan, tilt, zoom = self.getPtzDesiredPositionFromGoal(goal_handle.request)
        self.handleGoal(goal_handle, pan, tilt, zoom)

    def cancelPtzCb(self, goal_handle : ServerGoalHandle):
        """ Callback for cancelling the SetPtz action. """
        self.get_logger().info('Cancelling PTZ action')
        return CancelResponse.ACCEPT
    
    def homeCb(self, goal_handle : ServerGoalHandle):
        """ Callback for the Home action. """
        self.handleGoal(goal_handle, 0.0, 0.0, 0.0)
    
    def handleGoal(self, goal_handle : ServerGoalHandle, pan, tilt, zoom):
        """
        Handles the received goal for the PTZ action.
        This method sets the desired position for the PTZ camera based on the goal
        and updates the last command received time.
        Args:
            goal_handle (ServerGoalHandle): The handle for the goal.
            pan (float): The desired pan position.
            tilt (float): The desired tilt position.
            zoom (float): The desired zoom position.
        """
        self.time_last_command_received = self.get_clock().now()
        self.setPtzDesiredPosition(pan, tilt, zoom)
        goal_handle.execute()

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
        self.current_goal = goal_handle
        while self.current_goal is not None and rclpy.ok():
            # If goal has been reached or there is no goal -> control mode is idle
            if self.ptz.isInDesiredPosition():
                self.succeedAction(goal_handle, 'PTZ position reached successfully')
                break

            # If timeout has been reached, abort the goal
            elif self.get_clock().now() - self.time_last_command_received > self.duration_last_command_watchdog:
                self.abortAction(goal_handle, 'PTZ position not reached in time')
                break

            elif goal_handle.is_cancel_requested:
                self.cancelAction(goal_handle, 'PTZ action cancelled by user')
                break

            self.publishFeedback(goal_handle)
            time.sleep(1/self.desired_freq)

        if goal_handle.status < 4: # Not in [Succeeded, Cancelled, Aborted]
            self.get_logger().error(f'Exited loop in a bad state: {goal_handle.status}')
            self.abortAction(goal_handle, f'Exited loop in a bad state: {goal_handle.status}')

        return self.action_result

    def publishFeedback(self, goal_handle: ServerGoalHandle):
        """
        Publishes feedback for the current goal.
        This method sends feedback about the remaining pan, tilt, and zoom positions
        to the action server.
        Args:
            goal_handle (ServerGoalHandle): The handle for the current goal.
        """
        if goal_handle is None:
            return
        
        feedback = SetPtz.Feedback()
        feedback.remaining_pan, feedback.remaining_tilt, feedback.remaining_zoom = self.ptz.getRemainingPtzPosition()

        goal_handle.publish_feedback(feedback)

    def abortAction(self, goal_handle : ServerGoalHandle, msg : str):
        self.current_goal = None
        goal_handle.abort()
        self.action_result.response.success = False
        self.action_result.response.message = msg
        self.get_logger().error(f'Action aborted: {msg}')
        self.switchToControlState(self.IDLE)

    def succeedAction(self, goal_handle: ServerGoalHandle, msg: str):
        self.current_goal = None
        goal_handle.succeed()
        self.action_result.response.success = True
        self.action_result.response.message = msg
        self.get_logger().info(f'Action succeeded: {msg}')
        self.switchToControlState(self.IDLE)

    def cancelAction(self, goal_handle: ServerGoalHandle, msg: str):
        self.current_goal = None
        goal_handle.canceled()
        self.action_result.response.success = False
        self.action_result.response.message = msg
        self.get_logger().error(f'Action cancelled: {msg}')
        self.switchToControlState(self.IDLE)
