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
import diagnostic_updater
import diagnostic_msgs

from axis_camera.axis_lib.axis_control import ControlAxis

class AxisPTZ(threading.Thread):
    """
            Class interface to set the Pan Tilt Zoom of the camera
    """

    def __init__(self, args):
        self.hostname = args['hostname']
        self.camera_id = args['camera_id']
        self.camera_model = args['camera_model']
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
        self.error_reading = False
        self.error_reading_msg = ''

        # Timer to get/release ptz control
        if(self.use_control_timeout):
            self.last_command_time = rospy.Time(0)
            self.command_timeout = rospy.Duration(self.control_timeout_value)
        
        self.controller = ControlAxis(self.hostname)
        # Time to set when the last command was received
        self.t_last_command_time = rospy.Time(0)
        # Time to control when the last command was received
        self.t_last_command_watchdog = rospy.Duration(1.0)

        # position, velocity
        self.default_control_mode = 'position'
        self.control_mode = 'position'
        self.t_last_command_sent = rospy.Time(0)
        self.t_control_loop = 1 / self.rate

    def rosSetup(self):
        """
                Sets the ros connections
        """
        ns = rospy.get_namespace()
        self.pub = rospy.Publisher("~camera_params", AxisMsg, queue_size=10)
        self.sub = rospy.Subscriber("~ptz_command", ptz, self.commandPTZCb)
        # Publish the joint state of the pan & tilt
        self.joint_state_publisher = rospy.Publisher(self.joint_states_topic, JointState, queue_size=10)

        # Services
        self.home_service = rospy.Service('~home_ptz', Empty, self.homeService)

        # Diagnostic Updater
        self.diagnostics_updater = diagnostic_updater.Updater()
        self.diagnostics_updater.setHardwareID("%s-%s:%s" % (self.camera_model, self.camera_id, self.hostname))
        self.diagnostics_updater.add("Ptz state updater", self.getStateDiagnostic)
        # Creates a periodic callback to publish the diagnostics at desired freq
        self.diagnostics_timer = rospy.Timer(rospy.Duration(1.0), self.publishDiagnostics)

    def commandPTZCb(self, msg):
        """
            Command for ptz movements
        """
        self.t_last_command_time = rospy.Time.now()

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
            new_zoom = command.zoom + self.desired_zoom
            #rospy.loginfo('setCommandPTZ: new zoom = %.3lf +  %.3lf  = %.3lf', command.zoom, self.desired_zoom,new_zoom)
        else:
            new_pan = self.invert_pan*command.pan
            new_tilt = self.invert_tilt*command.tilt
            new_zoom = command.zoom
            
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
        elif pan < self.min_pan_value:
            pan = self.min_pan_value
        
        if tilt > self.max_tilt_value:
            tilt = self.max_tilt_value
        elif tilt < self.min_tilt_value:
            tilt = self.min_tilt_value
        
        if zoom > self.max_zoom_value:
            zoom = self.max_zoom_value
        elif zoom < self.min_zoom_value:
            zoom = self.min_zoom_value
            
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
        
        
    def controlPTZ(self):
        """
            Performs the control of the camera ptz
        """
        t_now = rospy.Time.now()

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
        
        control = self.controller.sendPTZCommand(pan_value, tilt_value, zoom_value)
        
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

            self.current_ptz.pan =  self.invert_pan * self.normalize_angle( ptz_read["pan"] - self.pan_offset)
            self.current_ptz.tilt = self.invert_tilt * (ptz_read["tilt"] - self.tilt_offset)
            
            self.current_ptz.zoom = ptz_read["zoom"]
            self.current_ptz.iris = ptz_read["iris"]
            self.current_ptz.autoiris = ptz_read["autoiris"]
            self.current_ptz.focus = ptz_read["focus"]
            self.current_ptz.autofocus = ptz_read["autofocus"]
        
            if not self.ptz_syncronized:
                self.desired_pan = self.invert_pan*self.current_ptz.pan 
                self.desired_tilt = self.invert_tilt*self.current_ptz.tilt
                self.desired_zoom = self.current_ptz.zoom
                rospy.loginfo('%s:getPTZState: PTZ state syncronized!', rospy.get_name())
                self.ptz_syncronized = True
            
            self.error_reading = ptz_read["error_reading"]
            self.error_reading_msg = ptz_read["error_reading_msg"]

            #rospy.loginfo_throttle(5, 'getPTZState read pan = %.3lf, current_ptz.pan = %.3lf,  read tilt = %.3lf, current_ptz.tilt = %.3lf, zoom = %.3lf', ptz_read["pan"], self.current_ptz.pan, ptz_read["tilt"], self.current_ptz.tilt, self.current_ptz.zoom)
        
        else:
            self.error_reading = ptz_read["error_reading"]
            self.error_reading_msg = ptz_read["error_reading_msg"]
            rospy.logerr('%s:getPTZState: received corrupted data: %s '%(rospy.get_name(),self.error_reading_msg))
            
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
        # Publishes the current PTZ values
        self.pub.publish(self.current_ptz)
        
        # Publish the joint state
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        
        msg.name = [self.pan_joint, self.tilt_joint, self.zoom_joint]
        msg.position = [self.current_ptz.pan, self.current_ptz.tilt, self.current_ptz.zoom]
        msg.velocity = [0.0, 0.0, 0.0]
        msg.effort = [0.0, 0.0, 0.0]
        
        self.joint_state_publisher.publish(msg)
        
        
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
        'tilt_offset': 0.0
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