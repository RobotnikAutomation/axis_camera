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
    def __init__(self, min_position, max_position, joint_name, offset, error, min_augment, max_augment):
        super().__init__(min_position, max_position, joint_name, offset, error, invert = False)
        self._min_augment = min_augment
        self._max_augment = max_augment
        self._total_augment = max_augment - min_augment
    
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