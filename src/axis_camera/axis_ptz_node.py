#!/usr/bin/env python

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

import threading
import math

import rospy

from std_srvs.srv import Empty
from sensor_msgs.msg import JointState

from robotnik_msgs.msg import Axis as AxisMsg
from robotnik_msgs.msg import ptz
from robotnik_msgs.msg import CameraParameters
from robotnik_msgs.msg import ImageSettings
from robotnik_msgs.msg import ReturnMessage
from robotnik_msgs.srv import SetCameraFocus, SetCameraFocusResponse
from robotnik_msgs.srv import SetCameraIris, SetCameraIrisResponse
from robotnik_msgs.srv import GetAxisDeviceInfo, GetAxisDeviceInfoResponse
import diagnostic_updater
import diagnostic_msgs

from axis_camera.axis_lib.axis_control import ControlAxis
from axis_camera.axis_lib.device_info import get_device_info_with_fallback
from robotnik_msgs.srv import SetInt16, SetInt16Response
from robotnik_msgs.srv import SetString, SetStringResponse
from robotnik_msgs.srv import GetStringList, GetStringListResponse
from robotnik_msgs.srv import GetImageSettings, GetImageSettingsResponse
from robotnik_msgs.srv import SetAutoTracking, SetAutoTrackingResponse
from robotnik_msgs.srv import GetAutoTrackingCapabilities, GetAutoTrackingCapabilitiesResponse
from robotnik_msgs.msg import AutoTrackingStatus
from std_msgs.msg import Bool

class AxisPTZ(threading.Thread):
    """
            Class interface to set the Pan Tilt Zoom of the camera
    """

    def __init__(self, args):
        self.hostname = args['hostname']
        self.camera_id = args['camera_id']
        self.camera_model = args['camera_model']
        self.username = args['username']
        self.password = args['password']
        self.enable_auth = args['enable_auth']
        self.rate = args['ptz_rate']
        self.autoflip = args['autoflip']
        self.eflip = args['eflip']
        self.tilt_joint = args['tilt_joint']
        self.pan_joint = args['pan_joint']
        self.zoom_joint = args['zoom_joint']
        self.min_pan_value = args['min_pan_value']
        self.max_pan_value = args['max_pan_value']
        self.min_tilt_value = args['min_tilt_value']
        self.max_tilt_value = args['max_tilt_value']
        self.min_zoom_value = args['min_zoom_value']
        self.max_zoom_value = args['max_zoom_value']
        self.min_zoom_augment = args['min_zoom_augment']
        self.max_zoom_augment = args['max_zoom_augment']
        self.min_zoom_step = args['min_zoom_step']
        self.error_pos = args['error_pos']
        self.error_zoom = args['error_zoom']
        self.joint_states_topic = args['joint_states_topic']
        self.use_control_timeout = args['use_control_timeout']
        self.control_timeout_value = args['control_timeout_value']
        if args['invert_pan'] == True:
            self.invert_pan = -1.0
        else:
            self.invert_pan = 1.0
        if args['invert_tilt'] == True:
            self.invert_tilt = -1.0
        else:
            self.invert_tilt = 1.0
        self.send_constantly = args['send_constantly']

        # Offset values to the center of the camera if it is not mounted center.
        self.pan_offset = args['pan_offset']    # Offset in radians with sign, positive to the right
        self.tilt_offset = args['tilt_offset']  # Offset in radians and sign, positive to the down

        self.current_ptz = AxisMsg()
        self.last_msg = ptz()
        threading.Thread.__init__(self)

        self.daemon = True
        self.run_control = True
        # Flag to know if the current params of the camera has been read
        self.ptz_syncronized = False
        # used in control position (degrees)

        self.desired_pan = 0.0
        self.desired_tilt = 0.0
        self.desired_zoom = 0.0
        self.desired_focus = 0.0
        self.desired_autofocus = False
        self.desired_iris = 0.0
        self.desired_autoiris = False
        self.error_reading = False
        self.error_reading_msg = ''
        self.focus_supported = True
        self.iris_supported = True
        self.focus_support_warned = False
        self.iris_support_warned = False
        self.focus_min_value = 1.0
        self.focus_max_value = 9999.0
        self.iris_min_value = 1.0
        self.iris_max_value = 9999.0
        self.iris_two_step_control = args.get('iris_two_step_control', False)

        # Auto-tracking state
        self.autotracking_active = False
        self.autotracking_requested_mode = 'auto'
        self.autotracking_active_mode = 'motion'
        self.autotracking_supported_modes = ['motion']
        self.autotracking_fallback_applied = False

        # Detect effective focus range at runtime
        self.effective_focus_min_raw = None
        self.effective_focus_min_percent = None
        self.last_commanded_focus_percent = None
        self.focus_clipping_warned = False
        self.timeout = 5
        self.device_model = 'unknown'
        self.device_serial = 'unknown'
        self.device_firmware = 'unknown'

        # Timer to get/release ptz control
        if(self.use_control_timeout):
            self.last_command_time = rospy.Time(0)
            self.command_timeout = rospy.Duration(self.control_timeout_value)
        
        self.controller = ControlAxis(self.hostname, args.get('username', 'root'), args.get('password', ''))
        # Time to set when the last command was received
        self.t_last_command_time = rospy.Time(0)
        # Time to control when the last command was received
        self.t_last_command_watchdog = rospy.Duration(1.0)

        # position, velocity
        self.default_control_mode = 'position'
        self.control_mode = 'position'
        self.t_last_command_sent = rospy.Time(0)
        self.t_control_loop = 1 / self.rate
        self.image_settings_pub_rate = args['image_settings_pub_rate']
        if self.image_settings_pub_rate <= 0.0:
            self.image_settings_pub_rate = 1.0
        self.image_settings_pub_period = rospy.Duration(1.0 / self.image_settings_pub_rate)
        self.t_last_image_settings_pub = rospy.Time(0)
        self.image_settings_metadata = self.controller.getImageSettingsMetadata()
        self.image_settings_error_active = False
        self.last_image_settings_error_message = ''

        self._readPTZLimitsFromCamera()

    def _readPTZLimitsFromCamera(self):
        ptz_limits = self.controller.getPTZLimits()
        if ptz_limits['error_reading']:
            rospy.logwarn('%s:_readPTZLimitsFromCamera: using default focus limits [%.1f, %.1f] and iris limits [%.1f, %.1f]: %s', rospy.get_name(), self.focus_min_value, self.focus_max_value, self.iris_min_value, self.iris_max_value, ptz_limits['error_reading_msg'])
            return

        focus_min = ptz_limits.get('focus_min')
        focus_max = ptz_limits.get('focus_max')
        if focus_min is None or focus_max is None or focus_max <= focus_min:
            rospy.logwarn('%s:_readPTZLimitsFromCamera: camera did not provide valid focus limits, using defaults [%.1f, %.1f]', rospy.get_name(), self.focus_min_value, self.focus_max_value)
        else:
            self.focus_min_value = focus_min
            self.focus_max_value = focus_max

        iris_min = ptz_limits.get('iris_min')
        iris_max = ptz_limits.get('iris_max')
        if iris_min is None or iris_max is None or iris_max <= iris_min:
            rospy.logwarn('%s:_readPTZLimitsFromCamera: camera did not provide valid iris limits, using defaults [%.1f, %.1f]', rospy.get_name(), self.iris_min_value, self.iris_max_value)
        else:
            self.iris_min_value = iris_min
            self.iris_max_value = iris_max

    def _focusRawToPercentage(self, focus_value):
        if self.focus_max_value <= self.focus_min_value:
            return 0.0

        clamped_focus = min(max(focus_value, self.focus_min_value), self.focus_max_value)
        return ((clamped_focus - self.focus_min_value) / (self.focus_max_value - self.focus_min_value)) * 100.0

    def _focusPercentageToRaw(self, focus_percentage):
        clamped_percentage = min(max(focus_percentage, 0.0), 100.0)
        return self.focus_min_value + ((self.focus_max_value - self.focus_min_value) * (clamped_percentage / 100.0))

    def _irisRawToPercentage(self, iris_value):
        if self.iris_max_value <= self.iris_min_value:
            return 0.0

        clamped_iris = min(max(iris_value, self.iris_min_value), self.iris_max_value)
        return ((clamped_iris - self.iris_min_value) / (self.iris_max_value - self.iris_min_value)) * 100.0

    def _irisPercentageToRaw(self, iris_percentage):
        clamped_percentage = min(max(iris_percentage, 0.0), 100.0)
        return self.iris_min_value + ((self.iris_max_value - self.iris_min_value) * (clamped_percentage / 100.0))
    
    def _updateEffectiveFocusFromCommand(self, commanded_focus_percent, actual_focus_percent, autofocus_enabled):
        """
        Learn effective focus minimum only from manual command responses.
        This avoids false estimates from passive readings.
        """
        if autofocus_enabled or commanded_focus_percent is None:
            return

        # Ignore tiny deviations due to quantization/noise.
        tolerance_percent = 2.0
        if actual_focus_percent <= commanded_focus_percent + tolerance_percent:
            return

        # With no prior estimate, only trust low commands to infer the lower bound.
        if self.effective_focus_min_percent is None and commanded_focus_percent > 50.0:
            return

        candidate_min_percent = actual_focus_percent
        candidate_min_raw = self._focusPercentageToRaw(candidate_min_percent)

        if self.effective_focus_min_percent is None or candidate_min_percent < self.effective_focus_min_percent:
            self.effective_focus_min_percent = candidate_min_percent
            self.effective_focus_min_raw = candidate_min_raw
    
    def _checkFocusClipping(self, commanded_focus_percent, actual_focus_percent):
        """
        Detect if camera clipped a focus command and warn the user.
        """
        # Only warn once per session to avoid log spam
        if self.focus_clipping_warned:
            return
        
        # If commanded is below effective minimum, warn
        if self.effective_focus_min_percent is not None and commanded_focus_percent < self.effective_focus_min_percent:
            # Check if this is actually being clipped (not just initial sync)
            if abs(actual_focus_percent - self.effective_focus_min_percent) < 2.0:
                rospy.logwarn(
                    '%s:Focus command limited by camera: requested %.1f%%, effective minimum is %.1f%%. '
                    'Camera will clamp to [%.1f%%, 100%%] in current conditions.',
                    rospy.get_name(), commanded_focus_percent, self.effective_focus_min_percent, self.effective_focus_min_percent
                )
                self.focus_clipping_warned = True

    def rosSetup(self):
        """
                Sets the ros connections
        """
        ns = rospy.get_namespace()
        self.pub = rospy.Publisher("~camera_params", AxisMsg, queue_size=10)
        self.sub = rospy.Subscriber("~ptz_command", ptz, self.commandPTZCb)
        # Publish the joint state of the pan & tilt
        self.joint_state_publisher = rospy.Publisher(self.joint_states_topic, JointState, queue_size=10)
        # Publish camera zoom info
        self.zoom_parameter_pub = rospy.Publisher("~camera_parameters", CameraParameters, queue_size=10)
        # Publish image settings state (base + explicit current alias)
        self.image_settings_pub = rospy.Publisher("~image_settings", ImageSettings, queue_size=10)
        # Services
        self.home_service = rospy.Service('~home_ptz', Empty, self.homeService)
        self.focus_service = rospy.Service('~set_focus', SetCameraFocus, self.setFocusService)
        self.iris_service = rospy.Service('~set_iris', SetCameraIris, self.setIrisService)
        self.set_brightness_service = rospy.Service('~set_brightness', SetInt16, self.setBrightnessServiceCb)
        self.set_contrast_service = rospy.Service('~set_contrast', SetInt16, self.setContrastServiceCb)
        self.set_saturation_service = rospy.Service('~set_saturation', SetInt16, self.setSaturationServiceCb)
        self.set_day_night_mode_service = rospy.Service('~set_day_night_mode', SetString, self.setDayNightModeServiceCb)
        self.set_day_night_shift_level_service = rospy.Service('~set_day_night_shift_level', SetInt16, self.setDayNightShiftLevelServiceCb)
        self.set_white_balance_service = rospy.Service('~set_white_balance', SetString, self.setWhiteBalanceServiceCb)
        self.get_white_balance_mode_service = rospy.Service('~get_white_balance_mode', GetStringList, self.getWhiteBalanceModeServiceCb)
        self.get_image_settings_service = rospy.Service('~get_image_settings', GetImageSettings, self.getImageSettingsServiceCb)
        self.autotracking_pub = rospy.Publisher('~autotracking_active', Bool, queue_size=10, latch=True)
        self.autotracking_status_pub = rospy.Publisher('~autotracking_status', AutoTrackingStatus, queue_size=10, latch=True)
        self.autotracking_pub.publish(Bool(data=self.autotracking_active))
        self._publishAutoTrackingStatus()
        self.set_autotracking_service = rospy.Service('~set_autotracking', SetAutoTracking, self.setAutoTrackingServiceCb)
        self.get_autotracking_capabilities_service = rospy.Service(
            '~get_autotracking_capabilities',
            GetAutoTrackingCapabilities,
            self.getAutoTrackingCapabilitiesServiceCb
        )
        self.loadDeviceInfo()
        self._loadAutoTrackingCapabilities()
        self.device_info_service = rospy.Service('~get_device_info', GetAxisDeviceInfo, self.getDeviceInfoServiceCb)

        # Diagnostic Updater
        self.diagnostics_updater = diagnostic_updater.Updater()
        self.diagnostics_updater.setHardwareID("%s-%s:%s" % (self.camera_model, self.camera_id, self.hostname))
        self.diagnostics_updater.add("Ptz state updater", self.getStateDiagnostic)
        # Creates a periodic callback to publish the diagnostics at desired freq
        self.diagnostics_timer = rospy.Timer(rospy.Duration(1.0), self.publishDiagnostics)
        # Poll autotracking state to detect changes made outside ROS (web UI, VMS, etc.)
        self.autotracking_poll_timer = rospy.Timer(rospy.Duration(2.0), self.pollAutoTrackingState)

        self.zoom_augments = []
        for i in range(int(self.min_zoom_augment), int(self.max_zoom_augment) + 1, int(self.min_zoom_step)):
            self.zoom_augments.append(i)

        rospy.loginfo('%s: device info model=%s serial=%s firmware=%s' %
                      (rospy.get_name(), self.device_model, self.device_serial, self.device_firmware))

    def loadDeviceInfo(self):
        """Load device info using shared device_info module"""
        info = get_device_info_with_fallback(
            self.hostname,
            timeout=self.timeout,
            enable_auth=self.enable_auth,
            username=self.username,
            password=self.password,
            logger=rospy.logwarn
        )
        self.device_model = info.get('model', 'unknown')
        self.device_serial = info.get('serial', 'unknown')
        self.device_firmware = info.get('firmware', 'unknown')

        rospy.set_param('~device/model', self.device_model)
        rospy.set_param('~device/serial', self.device_serial)
        rospy.set_param('~device/firmware', self.device_firmware)

    def getDeviceInfoServiceCb(self, req):
        return GetAxisDeviceInfoResponse(
            model=self.device_model,
            serial=self.device_serial,
            firmware=self.device_firmware
        )

    def setAutoTrackingServiceCb(self, req):
        response = SetAutoTrackingResponse()
        result = self.controller.setAutoTrackingWithMode(req.enabled, req.mode)
        response.success = result['success']
        response.applied_mode = result.get('applied_mode', 'motion')
        response.fallback_applied = result.get('fallback_applied', False)
        response.message = result['message']
        if result['success']:
            self.autotracking_active = req.enabled
            self.autotracking_requested_mode = req.mode
            self.autotracking_active_mode = response.applied_mode
            self.autotracking_fallback_applied = response.fallback_applied
            self.autotracking_pub.publish(Bool(data=self.autotracking_active))
            self._publishAutoTrackingStatus()
            rospy.loginfo('%s:setAutoTrackingServiceCb: %s', rospy.get_name(), result['message'])
        else:
            rospy.logerr('%s:setAutoTrackingServiceCb: %s', rospy.get_name(), result['message'])
        return response

    def getAutoTrackingCapabilitiesServiceCb(self, req):
        response = GetAutoTrackingCapabilitiesResponse()
        result = self.controller.getAutoTrackingCapabilities()
        response.supported_modes = result.get('supported_modes', ['motion'])
        response.current_mode = result.get('current_mode', 'motion')
        response.enabled = bool(result.get('enabled', False))

        # keep cached state aligned for status publication
        self.autotracking_supported_modes = list(response.supported_modes)
        self.autotracking_active_mode = response.current_mode
        self.autotracking_active = response.enabled
        self._publishAutoTrackingStatus()
        return response

    def _loadAutoTrackingCapabilities(self):
        result = self.controller.getAutoTrackingCapabilities()
        self.autotracking_supported_modes = result.get('supported_modes', ['motion'])
        self.autotracking_active_mode = result.get('current_mode', 'motion')
        self.autotracking_active = bool(result.get('enabled', False))
        self.autotracking_pub.publish(Bool(data=self.autotracking_active))
        self._publishAutoTrackingStatus()

    def _publishAutoTrackingStatus(self):
        msg = AutoTrackingStatus()
        msg.enabled = self.autotracking_active
        msg.requested_mode = self.autotracking_requested_mode
        msg.active_mode = self.autotracking_active_mode
        msg.fallback_applied = self.autotracking_fallback_applied
        self.autotracking_status_pub.publish(msg)

    def pollAutoTrackingState(self, event):
        """
        Synchronizes autotracking state with camera in case it is changed externally.
        """
        result = self.controller.getAutoTrackingState()
        if not result['success']:
            rospy.logdebug_throttle(30, '%s:pollAutoTrackingState: %s', rospy.get_name(), result['message'])
            return

        camera_state = bool(result['enabled'])
        caps = self.controller.getAutoTrackingCapabilities()
        camera_mode = caps.get('current_mode', self.autotracking_active_mode)
        self.autotracking_supported_modes = caps.get('supported_modes', self.autotracking_supported_modes)

        if camera_state != self.autotracking_active or camera_mode != self.autotracking_active_mode:
            self.autotracking_active = camera_state
            self.autotracking_active_mode = camera_mode
            self.autotracking_pub.publish(Bool(data=self.autotracking_active))
            self._publishAutoTrackingStatus()
            rospy.loginfo('%s:pollAutoTrackingState: autotracking updated from camera: enabled=%s mode=%s',
                          rospy.get_name(), self.autotracking_active, self.autotracking_active_mode)

    def commandPTZCb(self, msg):
        """
            Command for ptz movements
        """
        self.t_last_command_time = rospy.Time.now()

        if self.autotracking_active:
            rospy.logwarn_throttle(5, '%s:commandPTZCb: ignoring PTZ command, auto-tracking is active', rospy.get_name())
            return

        if self.ptz_syncronized:
            self.setCommandPTZ(msg)
            if self.send_constantly == False:
                self.sendPTZCommand()
        else:
            rospy.logwarn_throttle(1, '%s:commandPTZCb: PTZ not syncronized!', rospy.get_name())

        
    def setCommandPTZ(self, command):
        # Save time of requested command
        if(self.use_control_timeout):
            self.last_command_time = rospy.get_rostime()
            #rospy.loginfo("Last command time %i %i", self.last_command_time.secs, self.last_command_time.nsecs)
        new_control_mode = self.default_control_mode
        # Check available control modes
        if command.mode == 'position' or command.mode == 'velocity':
            new_control_mode = command.mode
        if new_control_mode != self.control_mode:
            self.control_mode = new_control_mode
        
        # Need to convert from rad to degree
        # relative motion
        if command.relative:            
            new_pan = self.invert_pan*command.pan + self.desired_pan
            new_tilt = self.invert_tilt*command.tilt + self.desired_tilt
            # new_zoom = (command.zoom / self.max_zoom_augment ) * self.max_zoom_value + self.desired_zoom
            new_zoom = (command.zoom)/(self.max_zoom_augment - 1) * (self.max_zoom_value - self.min_zoom_value) + self.desired_zoom
            #rospy.loginfo('setCommandPTZ: new zoom = %.3lf +  %.3lf  = %.3lf', command.zoom, self.desired_zoom,new_zoom)
        else:
            new_pan = self.invert_pan*command.pan
            new_tilt = self.invert_tilt*command.tilt
            if command.zoom == 0:
                command.zoom = 1
            # new_zoom = (command.zoom / self.max_zoom_augment ) * self.max_zoom_value
            new_zoom = (command.zoom - 1)/(self.max_zoom_augment - 1) * (self.max_zoom_value - self.min_zoom_value) 
            
            # Applies limit restrictions
        new_pan, new_tilt, new_zoom = self.enforcePTZLimits(new_pan, new_tilt, new_zoom)            
            
        self.desired_pan = new_pan
        self.desired_tilt = new_tilt
        self.desired_zoom = new_zoom

        #rospy.loginfo_throttle(1, 'setCommandPTZ: pan = %.3lf, tilt = %.3lf, zoom = %.3lf', self.desired_pan, self.desired_tilt, self.desired_zoom)


    def enforcePTZLimits(self, pan, tilt, zoom):
        """
            Enforces the limits of the PTZ values
        """
        if pan > self.max_pan_value:
            pan = self.max_pan_value
            rospy.logerr('PAN out of limits, setting max value')
        elif pan < self.min_pan_value:
            pan = self.min_pan_value
            rospy.logerr('PAN out of limits, setting min value')
        
        if tilt > self.max_tilt_value:
            tilt = self.max_tilt_value
            rospy.logerr('TILT out of limits, setting max value')
        elif tilt < self.min_tilt_value:
            tilt = self.min_tilt_value
            rospy.logerr('TILT out of limits, setting min value')
        
        if zoom > self.max_zoom_value:
            zoom = self.max_zoom_value
            # rospy.logerr('ZOOM out of limits, setting max value')
        elif zoom < self.min_zoom_value:
            zoom = self.min_zoom_value
            # rospy.logerr('ZOOM out of limits, setting min value')
            
        return pan, tilt, zoom

    def homeService(self, req):
        
        # Set home values
        home_command = ptz()
        home_command.relative = False
        home_command.pan = 0.0
        home_command.tilt = 0.0
        home_command.zoom = 0
        home_command.mode = 'position'
        
        if self.ptz_syncronized:
            self.setCommandPTZ(home_command)
            if self.send_constantly == False:
                self.sendPTZCommand()
        else:
            rospy.logwarn('%s:homeService: PTZ not syncronized!', rospy.get_name())
            
        return {}

    def setBrightnessServiceCb(self, req):
        result = self.controller.setBrightness(req.data.data)
        if result['success']:
            rospy.loginfo('%s:setBrightnessServiceCb: %s', rospy.get_name(), result['message'])
        else:
            rospy.logerr('%s:setBrightnessServiceCb: %s', rospy.get_name(), result['message'])
        return SetInt16Response(ret=ReturnMessage(success=result['success'], message=result['message']))

    def setContrastServiceCb(self, req):
        result = self.controller.setContrast(req.data.data)
        if result['success']:
            rospy.loginfo('%s:setContrastServiceCb: %s', rospy.get_name(), result['message'])
        else:
            rospy.logerr('%s:setContrastServiceCb: %s', rospy.get_name(), result['message'])
        return SetInt16Response(ret=ReturnMessage(success=result['success'], message=result['message']))

        
    def setSaturationServiceCb(self, req):
        result = self.controller.setSaturation(req.data.data)
        if result['success']:
            rospy.loginfo('%s:setSaturationServiceCb: %s', rospy.get_name(), result['message'])
        else:
            rospy.logerr('%s:setSaturationServiceCb: %s', rospy.get_name(), result['message'])
        return SetInt16Response(ret=ReturnMessage(success=result['success'], message=result['message']))

    def setDayNightModeServiceCb(self, req):
        result = self.controller.setDayNightMode(req.data)
        if result['success']:
            rospy.loginfo('%s:setDayNightModeServiceCb: %s', rospy.get_name(), result['message'])
        else:
            rospy.logerr('%s:setDayNightModeServiceCb: %s', rospy.get_name(), result['message'])
        return SetStringResponse(ret=ReturnMessage(success=result['success'], message=result['message']))

    def setDayNightShiftLevelServiceCb(self, req):
        result = self.controller.setDayNightShiftLevel(req.data.data)
        if result['success']:
            rospy.loginfo('%s:setDayNightShiftLevelServiceCb: %s', rospy.get_name(), result['message'])
        else:
            rospy.logerr('%s:setDayNightShiftLevelServiceCb: %s', rospy.get_name(), result['message'])
        return SetInt16Response(ret=ReturnMessage(success=result['success'], message=result['message']))

    def setWhiteBalanceServiceCb(self, req):
        result = self.controller.setWhiteBalance(req.data)
        if result['success']:
            rospy.loginfo('%s:setWhiteBalanceServiceCb: %s', rospy.get_name(), result['message'])
        else:
            rospy.logerr('%s:setWhiteBalanceServiceCb: %s', rospy.get_name(), result['message'])
        return SetStringResponse(ret=ReturnMessage(success=result['success'], message=result['message']))

    def getWhiteBalanceModeServiceCb(self, req):
        result = self.controller.getWhiteBalanceModes()
        if result['success']:
            rospy.loginfo('%s:getWhiteBalanceModeServiceCb: %s', rospy.get_name(), result['message'])
        else:
            rospy.logerr('%s:getWhiteBalanceModeServiceCb: %s', rospy.get_name(), result['message'])
        return GetStringListResponse(
            strings=result['modes'],
            ret=ReturnMessage(success=result['success'], message=result['message'])
        )

    def getImageSettingsServiceCb(self, req):
        result = self.controller.getImageSettings()
        metadata = self.image_settings_metadata

        image_settings_msg = ImageSettings()
        image_settings_msg.header.stamp = rospy.Time.now()
        image_settings_msg.is_valid = result['success']
        image_settings_msg.status_message = result['message']

        image_settings_msg.brightness = result['brightness']
        image_settings_msg.brightness_min = metadata['brightness_min']
        image_settings_msg.brightness_max = metadata['brightness_max']

        image_settings_msg.contrast = result['contrast']
        image_settings_msg.contrast_min = metadata['contrast_min']
        image_settings_msg.contrast_max = metadata['contrast_max']

        image_settings_msg.saturation = result['saturation']
        image_settings_msg.saturation_min = metadata['saturation_min']
        image_settings_msg.saturation_max = metadata['saturation_max']

        image_settings_msg.white_balance = result['white_balance']
        image_settings_msg.white_balance_available = metadata['white_balance_available']

        image_settings_msg.day_night_available = metadata['day_night_available']
        image_settings_msg.is_night_mode_active = result['is_night_mode_active']
        image_settings_msg.day_night_shift_level = result['day_night_shift_level']
        image_settings_msg.day_night_shift_level_min = metadata['day_night_shift_level_min']
        image_settings_msg.day_night_shift_level_max = metadata['day_night_shift_level_max']

        if result['success']:
            rospy.loginfo('%s:getImageSettingsServiceCb: retrieved image settings', rospy.get_name())
        else:
            rospy.logerr('%s:getImageSettingsServiceCb: %s', rospy.get_name(), result['message'])
        return GetImageSettingsResponse(
            success=result['success'],
            message=result['message'],
            image_settings=image_settings_msg
        )

    def setFocusService(self, req):
        response = SetCameraFocusResponse()

        if not self.ptz_syncronized:
            response.ret = False
            response.message = 'PTZ not synchronized yet'
            rospy.logwarn('%s:setFocusService: %s', rospy.get_name(), response.message)
            return response

        if not self.focus_supported:
            response.ret = False
            response.message = 'Focus control not supported by this camera'
            rospy.logwarn('%s:setFocusService: %s', rospy.get_name(), response.message)
            return response

        if not req.auto and not math.isfinite(req.value):
            response.ret = False
            response.message = 'Invalid focus value'
            rospy.logwarn('%s:setFocusService: %s', rospy.get_name(), response.message)
            return response

        if not req.auto and (req.value < 0.0 or req.value > 100.0):
            response.ret = False
            response.message = 'Focus percentage must be within [0, 100]'
            rospy.logwarn('%s:setFocusService: %s', rospy.get_name(), response.message)
            return response

        focus_value = None if req.auto else self._focusPercentageToRaw(req.value)
        
        # Warn if requesting focus below detected effective minimum
        if not req.auto and self.effective_focus_min_percent is not None and req.value < self.effective_focus_min_percent:
            rospy.logwarn(
                '%s:setFocusService: Focus request %.1f%% is below effective minimum %.1f%%. '
                'Camera will likely clamp to %.1f%% in current conditions.',
                rospy.get_name(), req.value, self.effective_focus_min_percent, self.effective_focus_min_percent
            )
        
        control = self.controller.sendPTZCommand(focus=focus_value, autofocus=req.auto)
        if not self._commandSucceeded(control, 'focus'):
            response.ret = False
            response.message = self._formatCommandError(control, 'focus')
            return response

        self.desired_autofocus = req.auto
        if not req.auto:
            self.desired_focus = req.value
            self.last_commanded_focus_percent = req.value

        response.ret = True
        response.message = 'Focus command sent'
        return response

    def setIrisService(self, req):
        response = SetCameraIrisResponse()

        if not self.ptz_syncronized:
            response.ret = False
            response.message = 'PTZ not synchronized yet'
            rospy.logwarn('%s:setIrisService: %s', rospy.get_name(), response.message)
            return response

        if not self.iris_supported:
            response.ret = False
            response.message = 'Iris control not supported by this camera'
            rospy.logwarn('%s:setIrisService: %s', rospy.get_name(), response.message)
            return response

        if not req.auto and not math.isfinite(req.value):
            response.ret = False
            response.message = 'Invalid iris value'
            rospy.logwarn('%s:setIrisService: %s', rospy.get_name(), response.message)
            return response

        if not req.auto and (req.value < 0.0 or req.value > 100.0):
            response.ret = False
            response.message = 'Iris percentage must be within [0, 100]'
            rospy.logwarn('%s:setIrisService: %s', rospy.get_name(), response.message)
            return response

        iris_value = None if req.auto else self._irisPercentageToRaw(req.value)
        if not req.auto and self.iris_two_step_control:
            # Send autoiris=off first; some cameras (e.g. P5676-LE) revert to
            # auto-iris when the two commands arrive simultaneously.
            self.controller.sendPTZCommand(autoiris=False)
        control = self.controller.sendPTZCommand(iris=iris_value, autoiris=req.auto)
        if not self._commandSucceeded(control, 'iris'):
            response.ret = False
            response.message = self._formatCommandError(control, 'iris')
            return response

        self.desired_autoiris = req.auto
        if iris_value is not None:
            self.desired_iris = req.value  # store as percentage

        response.ret = True
        response.message = 'Iris command sent'
        return response

    def _commandSucceeded(self, control, feature_name):
        if control['status'] == 204 and not control['exception']:
            return True

        message = self._formatCommandError(control, feature_name)
        if control['exception']:
            rospy.logerr('%s:%s command failed: %s', rospy.get_name(), feature_name, message)
        else:
            rospy.logwarn('%s:%s command failed: %s', rospy.get_name(), feature_name, message)
        return False

    def _formatCommandError(self, control, feature_name):
        if control['exception']:
            return str(control['error_msg'])

        body = control.get('body', '')
        if body:
            return 'HTTP %s: %s' % (control['status'], body.strip())
        return 'HTTP %s while sending %s command to %s%s' % (control['status'], feature_name, self.hostname, control.get('url', ''))

    def _updateOptionalSupport(self, ptz_read):
        self.focus_supported = ptz_read.get('supports_focus', False)
        self.iris_supported = ptz_read.get('supports_iris', False)

        if not self.focus_supported and not self.focus_support_warned:
            rospy.logwarn('%s:getPTZState: camera does not report focus/autofocus support', rospy.get_name())
            self.focus_support_warned = True

        if not self.iris_supported and not self.iris_support_warned:
            rospy.logwarn('%s:getPTZState: camera does not report iris/autoiris support', rospy.get_name())
            self.iris_support_warned = True

    def controlPTZ(self):
        """
            Performs the control of the camera ptz
        """
        t_now = rospy.Time.now()

        if self.autotracking_active:
            return

        if self.control_mode == 'position':
            # Only if it's syncronized
            if self.ptz_syncronized:
                if self.send_constantly == True:
                    self.sendPTZCommand()
        
        elif self.control_mode == 'velocity':# Nothing for now
            '''if (t_now - self.t_last_command_sent) > self.t_control_loop:
                rospy.loginfo('controlPTZ: sending velocity command')
            ''' 
            # Only if it's syncronized
            if self.ptz_syncronized:
                if self.send_constantly == True:
                    self.sendPTZCommand()

        if (t_now - self.t_last_command_time) > self.t_last_command_watchdog:
            #rospy.loginfo_throttle(5, 'controlPTZ: watchdog timeout')
            # syncronize desired position to the current one every time the control is idle
            self.desired_pan = self.invert_pan*self.current_ptz.pan 
            self.desired_tilt = self.invert_tilt*self.current_ptz.tilt
            self.desired_zoom = self.current_ptz.zoom
            self.desired_focus = self.current_ptz.focus
            self.desired_autofocus = self.current_ptz.autofocus
            self.desired_iris = self.current_ptz.iris
            self.desired_autoiris = self.current_ptz.autoiris

        

    def isPTZinPosition(self):	
        """
            @return True if camera has the desired position / settings
        """
        if abs(self.current_ptz.pan - self.desired_pan) <= self.error_pos and abs(self.current_ptz.tilt - self.desired_tilt) <= self.error_pos and abs(self.current_ptz.zoom - self.desired_zoom) <= self.error_zoom:
            '''rospy.logwarn('isPTZinPosition: pan %.3lf vs %.3lf', self.current_ptz.pan, self.desired_pan)
            rospy.logwarn('isPTZinPosition: tilt %.3lf vs %.3lf', self.current_ptz.tilt, self.desired_tilt)
            rospy.logwarn('isPTZinPosition: zoom %.3lf vs %.3lf', self.current_ptz.zoom, self.desired_zoom)'''
            return True
        else:
            return False

    def sendPTZCommand(self, pan = None, tilt = None, zoom = None):
        """
            Sends the ptz to the camera
        """

        # Add offsets to the pan and tilt values
        if pan is None:
            pan_value = self.desired_pan + self.pan_offset
        else:
            pan_value = pan + self.pan_offset
        
        if tilt is None:
            tilt_value = self.desired_tilt + self.tilt_offset
        else:
            tilt_value = tilt + self.tilt_offset
        
        if zoom is None:
            zoom_value = self.desired_zoom
        else:
            zoom_value = zoom
        
        #rospy.loginfo('sendPTZCommand: desired_pan = %.3lf, pan_offset = %.3lf, pan = %.3lf', self.desired_pan, self.pan_offset, pan)
        #rospy.loginfo('sendPTZCommand: desired_tilt = %.3lf, tilt_offset = %.3lf, tilt = %.3lf',self.desired_tilt, self.tilt_offset, tilt)

        pan_value = math.degrees(pan_value)
        tilt_value = math.degrees(tilt_value)
        zoom_value = self.desired_zoom
        
        control = self.controller.sendPTZCommand(pan=pan_value, tilt=tilt_value, zoom=zoom_value)
        
        if control['status'] != 204 and not control['exception']:
            rospy.logerr('%s/sendPTZCommand: Error getting response. url = %s%s'% (rospy.get_name(), self.hostname, control['url']) )
        elif control['exception']:
            rospy.logerr('%s:sendPTZCommand: error connecting the camera: %s '%(rospy.get_name(), control['error_msg']))
        
        self.t_last_command_sent = rospy.Time.now()
            
    def getPTZState(self):
        """
            Gets the current ptz state/position of the camera
        """

        # First time saves the current values
        ptz_read = self.controller.getPTZState()
        if not ptz_read["error_reading"]:
            self._updateOptionalSupport(ptz_read)

            self.current_ptz.pan =  self.invert_pan * self.normalize_angle( ptz_read["pan"] - self.pan_offset)
            self.current_ptz.tilt = self.invert_tilt * (ptz_read["tilt"] - self.tilt_offset)
            
            self.current_ptz.zoom = ptz_read["zoom"]
            self.current_ptz.iris = self._irisRawToPercentage(ptz_read["iris"])
            self.current_ptz.autoiris = ptz_read["autoiris"]
            self.current_ptz.focus = self._focusRawToPercentage(ptz_read["focus"])
            self.current_ptz.autofocus = ptz_read["autofocus"]

            # If we just commanded focus, check if it was clipped
            if not ptz_read["autofocus"] and self.last_commanded_focus_percent is not None:
                self._updateEffectiveFocusFromCommand(
                    self.last_commanded_focus_percent,
                    self.current_ptz.focus,
                    self.current_ptz.autofocus
                )
                self._checkFocusClipping(self.last_commanded_focus_percent, self.current_ptz.focus)
        
            if not self.ptz_syncronized:
                self.desired_pan = self.invert_pan*self.current_ptz.pan 
                self.desired_tilt = self.invert_tilt*self.current_ptz.tilt
                self.desired_zoom = self.current_ptz.zoom
                self.desired_focus = self.current_ptz.focus
                self.desired_autofocus = self.current_ptz.autofocus
                self.desired_iris = self.current_ptz.iris
                self.desired_autoiris = self.current_ptz.autoiris
                rospy.loginfo('%s:getPTZState: PTZ state syncronized!', rospy.get_name())
                self.ptz_syncronized = True
            
            self.error_reading = ptz_read["error_reading"]
            self.error_reading_msg = ptz_read["error_reading_msg"]

            #rospy.loginfo_throttle(5, 'getPTZState read pan = %.3lf, current_ptz.pan = %.3lf,  read tilt = %.3lf, current_ptz.tilt = %.3lf, zoom = %.3lf', ptz_read["pan"], self.current_ptz.pan, ptz_read["tilt"], self.current_ptz.tilt, self.current_ptz.zoom)
        
        else:
            self.error_reading = ptz_read["error_reading"]
            self.error_reading_msg = ptz_read["error_reading_msg"]
            rospy.logerr('%s:getPTZState: received corrupted data: %s '%(rospy.get_name(),self.error_reading_msg))
            rospy.signal_shutdown('PTZ read error: %s' % self.error_reading_msg)
            
        #print('Get state')
        #self.axis.pub.publish(self.msg)
        
    def normalize_angle(self, angle_in_radians) -> float:
        """
        Normalizes an angle in radians to be between -π and π.
        """
        normalized_angle = angle_in_radians
        while normalized_angle > math.pi:
            normalized_angle -= 2 * math.pi
        while normalized_angle < -math.pi:
            normalized_angle += 2 * math.pi
        
        return normalized_angle
       
    def run(self):
        """
            Executes the thread
        """
        r = rospy.Rate(self.rate)
        
        while not rospy.is_shutdown():
            
            self.getPTZState()
            
            if(self.use_control_timeout):
                self.manageControl()
            
            # Performs interaction with the camera if it is enabled
            if self.run_control:
                self.controlPTZ()
                #print('Alive')
            # Publish ROS msgs
            self.publishROS()
                
            
            r.sleep()
        
        print('Bye!')


    def publishROS(self):
        """
            Publish to ROS server
        """
        # Publish the zoom parameters
        zoom_parameters = CameraParameters()
        zoom_parameters.min_zoom_step = int(self.min_zoom_step)
        zoom_parameters.zoom_lower_limit = int(self.min_zoom_augment)
        zoom_parameters.zoom_upperlimit = int(self.max_zoom_augment)
        zoom_parameters.zoom_augments = self.zoom_augments

        self.zoom_parameter_pub.publish(zoom_parameters)
        # Publishes the current PTZ values
        self.pub.publish(self.current_ptz)
        
        # Publish the joint state
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        
        msg.name = [self.pan_joint, self.tilt_joint, self.zoom_joint]
        normalized_zoom = (self.current_ptz.zoom - self.min_zoom_value) / (self.max_zoom_value - self.min_zoom_value) * (self.max_zoom_augment - 1) + 1
        msg.position = [self.current_ptz.pan, self.current_ptz.tilt, round(normalized_zoom) ]
        msg.velocity = [0.0, 0.0, 0.0]
        msg.effort = [0.0, 0.0, 0.0]
        
        self.joint_state_publisher.publish(msg)

        now = rospy.Time.now()
        if (now - self.t_last_image_settings_pub) >= self.image_settings_pub_period:
            self.publishImageSettings(now)
            self.t_last_image_settings_pub = now

    def publishImageSettings(self, now):
        result = self.controller.getImageSettings()
        metadata = self.image_settings_metadata

        msg = ImageSettings()
        msg.header.stamp = now
        msg.is_valid = result['success']
        msg.status_message = result['message']

        msg.brightness = result['brightness']
        msg.brightness_min = metadata['brightness_min']
        msg.brightness_max = metadata['brightness_max']

        msg.contrast = result['contrast']
        msg.contrast_min = metadata['contrast_min']
        msg.contrast_max = metadata['contrast_max']

        msg.saturation = result['saturation']
        msg.saturation_min = metadata['saturation_min']
        msg.saturation_max = metadata['saturation_max']

        msg.white_balance = result['white_balance']
        msg.white_balance_available = metadata['white_balance_available']

        msg.day_night_available = metadata['day_night_available']
        msg.is_night_mode_active = result['is_night_mode_active']
        msg.day_night_shift_level = result['day_night_shift_level']
        msg.day_night_shift_level_min = metadata['day_night_shift_level_min']
        msg.day_night_shift_level_max = metadata['day_night_shift_level_max']

        if not result['success']:
            is_new_error = (not self.image_settings_error_active) or (
                self.last_image_settings_error_message != result['message']
            )
            if is_new_error:
                rospy.logerr('%s:publishImageSettings: %s', rospy.get_name(), result['message'])
            self.image_settings_error_active = True
            self.last_image_settings_error_message = result['message']
        elif self.image_settings_error_active:
            rospy.loginfo('%s:publishImageSettings: image settings read recovered', rospy.get_name())
            self.image_settings_error_active = False
            self.last_image_settings_error_message = ''

        self.image_settings_pub.publish(msg)
        
        
    def get_data(self):
        return self.msg

    def stop_control(self):
        """
            Stops the control loop
        """
        self.run_control = False

    def start_control(self):
        """
            Starts the control loop
        """
        self.run_control = True
        
    def manageControl(self):
        """
            Gets/releases ptz control using a timeout
        """
        
        if(rospy.get_rostime() - self.last_command_time < self.command_timeout):
            if not self.run_control:
                self.start_control()
        else:
            if self.run_control:
                self.stop_control()	


    def peer_subscribe(self, topic_name, topic_publish, peer_publish):
        """
            Callback when a peer has subscribed from a topic
        """
        #print('Is control loop enabled? %s'%self.run_control)
        
        
        if not self.run_control:
            self.start_control()
            
    def peer_unsubscribe(self, topic_name, num_peers):
        """
            Callback when a peer has unsubscribed from a topic
        """
        #print('Num of peers = %d'%num_peers)
        
        if num_peers == 0:
            #print('Stopping control')
            self.stop_control()
            
    def getStateDiagnostic(self, stat):		
        """
        Callback to analyze the state of ptz the params read from the camera
        """
        
        if self.error_reading:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Error getting ptz data: %s" % self.error_reading_msg)
        else:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, "Reading ptz data")
        
        stat.add("rate", self.rate)
        stat.add("pan", self.current_ptz.pan)
        stat.add("tilt", self.current_ptz.tilt)
        stat.add("zoom", self.current_ptz.zoom)
        stat.add("focus_declared_range_percent", "[0.0, 100.0]")
        if self.effective_focus_min_percent is not None:
            stat.add("focus_effective_range_percent", "[%.1f, 100.0]" % self.effective_focus_min_percent)
        stat.add("focus_supported", self.focus_supported)
        stat.add("iris_declared_range_percent", "[0.0, 100.0]")
        stat.add("iris_supported", self.iris_supported)
        
        return stat

    def publishDiagnostics(self, event):
        """
                Publishes the diagnostics at the desired rate
        """
        # Updates diagnostics
        self.diagnostics_updater.update()
    
def main():

    rospy.init_node("axis_camera")

    axis_node_name = rospy.get_name()
    axis_node_namespace = rospy.get_namespace()

    print('namespace = %s, name = %s' % (axis_node_namespace, axis_node_name))

    # default params
    arg_defaults = {
        'hostname': '192.168.1.205',
        'username': 'root',
        'password': 'R0b0tn1K',
        'enable_auth': True,
        'camera_id': 'XXXX',  # internal id (if necessary)
        'camera_model': 'axis_m5525',
        'autoflip': False,
        'eflip': False,
        'pan_joint': 'pan',
        'tilt_joint': 'tilt',
        'zoom_joint': 'zoom',
        'min_pan_value': -2.97,
        'max_pan_value': 2.97,
        'min_tilt_value': 0,
        'max_tilt_value': 1.57,
        'max_zoom_value': 20000,
        'min_zoom_value': 0,
        'min_zoom_step': 1,
        'min_zoom_augment': 0.0,
        'max_zoom_augment': 30.0,
        'ptz_rate': 5.0,
        'error_pos': 0.02,
        'error_zoom': 99.0,
        'joint_states_topic': 'joint_states',
        'use_control_timeout': False,
        'control_timeout_value': 5.0,
        'invert_pan': False,
        'invert_tilt': False,
        'send_constantly': False,
        'pan_offset': 0.0,
        'tilt_offset': 0.0,
        'image_settings_pub_rate': 1.0,
        'iris_two_step_control': False
    }
    args = {}

    for name in arg_defaults:

        param_name = '%s%s' % (axis_node_namespace, name)

        if rospy.search_param(param_name):
            args[name] = rospy.get_param(param_name)
        else:
            args[name] = arg_defaults[name]

    rospy.loginfo('%s: args: %s' % (axis_node_name, args))

    axis = AxisPTZ(args)
    axis.rosSetup()
    rospy.loginfo('%s: starting' % axis_node_name)
    axis.start()
    axis.run()


if __name__ == "__main__":
	main()
