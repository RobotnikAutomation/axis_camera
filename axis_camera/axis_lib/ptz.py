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

from axis_camera.axis_lib.axis_control import ControlAxis
from axis_camera.axis_lib.joints import Joint, ZoomJoint

class Ptz:
    def __init__(self, hostname : str, camera_number : int, pan : Joint, tilt : Joint, zoom : ZoomJoint, connection_timeout = 1000):
        self.controller = ControlAxis(hostname, camera_number, connection_timeout)
        info = self.controller.getPTZInfo()
        if info["error"]:
            raise Exception("Error reading PTZ info: %s" % info["error_msg"])
        self.hostname = hostname
        self.pan = pan
        self.tilt = tilt
        self.zoom = zoom
        self.iris = -1
        self.autoiris = False
        self.focus = -1
        self.autofocus = False
        self._is_syncronized = False
        self._pan_tilt_velocity_control = "continuouspantiltmove" in info.keys()
        self._zoom_velocity_control = "continuouszoommove" in info.keys()
    
    def isMoving(self):
        """
        Returns True if the camera is currently moving.
        """
        status = self.controller.getPTZStatus()
        if status["error"] or "moving" not in status:
            return False
        return not (status["moving"] == 'no')

    def hasVelocityControl(self):
        """
        Returns True if the camera supports velocity control for pan/tilt and zoom.
        """
        return self._pan_tilt_velocity_control

    def updatePtzPosition(self):
        """ Updates the PTZ position with new values. """
        # First time saves the current values
        ptz_read = self.controller.getPTZPosition()
        if not ptz_read["error"]:
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
        
        return ptz_read["error"], ptz_read["error_msg"]
    
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

        if control['status'] != 204 and not control['error']:
            msg = 'sendPTZCommand: Error getting response. url = %s%s'% (self.hostname, control['url'])
            return False, msg
        elif control['error']:
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
        if not (self._pan_tilt_velocity_control or self._zoom_velocity_control):
            return False, 'PTZ velocity control is not supported by the camera.'

        self.setDesiredVelocity(pan, tilt, zoom)
        control = self.controller.sendPTZVelocityCommand(
            self.pan.getDesiredVelocity(),
            self.tilt.getDesiredVelocity(),
            self.zoom.getDesiredVelocity()
        )

        if control['status'] != 204 and not control['error']:
            msg = 'sendPtzVelocityCommand: Error getting response. url = %s%s'% (self.hostname, control['url'])
            return False, msg
        elif control['error']:
            msg = 'sendPtzVelocityCommand: Exception connecting to the camera: %s '% (control['error_msg'])
            return False, msg
        else:
            return True, 'PTZ command sent successfully to %s' % (self.hostname)

    def stopPtzVelocityCommand(self):
        return self.sendPtzVelocityCommand(0, 0, 0)
