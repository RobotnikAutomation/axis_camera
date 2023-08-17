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
try:
    import urllib
    import urllib2
except:
	import urllib.request, urllib.error, urllib.parse
try:
    import httplib
except:
    import http.client
import socket
import math

import rospy

from std_srvs.srv import Empty
from sensor_msgs.msg import JointState

from robotnik_msgs.msg import Axis as AxisMsg
from robotnik_msgs.msg import ptz
import diagnostic_updater
import diagnostic_msgs


class AxisPTZ(threading.Thread):
    """
            Class interface to set the Pan Tilt Zoom of the camera
    """

    def __init__(self, args):
        self.enable_auth = args['enable_auth']
        self.hostname = args['hostname']
        self.username = args['username']
        self.password = args['password']
        self.node_name = args['node_name']
        self.camera_id = args['camera_id']
        self.camera_model = args['camera_model']
        self.rate = args['ptz_rate']
        self.autoflip = args['autoflip']
        self.eflip = args['eflip']
        self.tilt_joint = args['tilt_joint']
        self.pan_joint = args['pan_joint']
        self.min_pan_value = args['min_pan_value']
        self.max_pan_value = args['max_pan_value']
        self.min_tilt_value = args['min_tilt_value']
        self.max_tilt_value = args['max_tilt_value']
        self.min_zoom_value = args['min_zoom_value']
        self.max_zoom_value = args['max_zoom_value']
        self.home_pan_value = args['home_pan_value']
        self.home_tilt_value = args['home_tilt_value']
        self.error_pos = args['error_pos']
        self.error_zoom = args['error_zoom']
        self.joint_states_topic = args['joint_states_topic']
        self.use_control_timeout = args['use_control_timeout']
        self.control_timeout_value = args['control_timeout_value']
        self.invert_ptz = args['invert_ptz']
        self.initialization_delay = args['initialization_delay']
        self.send_constantly = args['send_constantly']

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

    def rosSetup(self):
        """
                Sets the ros connections
        """
        ns = rospy.get_namespace()
        self.pub = rospy.Publisher("%s%s/camera_params" % (ns, self.node_name), AxisMsg, queue_size=10)
        #self.sub = rospy.Subscriber("cmd", Axis, self.cmd)
        self.sub = rospy.Subscriber("%s%s/ptz_command" % (ns, self.node_name), ptz, self.commandPTZCb)
        # Publish the joint state of the pan & tilt
        self.joint_state_publisher = rospy.Publisher(self.joint_states_topic, JointState, queue_size=10)

        # Services
        self.home_service = rospy.Service('%s%s/home_ptz' % (ns, self.node_name), Empty, self.homeService)

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
        #print(msg)
        self.setCommandPTZ(msg)
        if not self.send_constantly:
            self.sendPTZCommand()

        
    def setCommandPTZ(self, command):
        # Save time of requested command
        if(self.use_control_timeout):
            self.last_command_time = rospy.get_rostime()
            #rospy.loginfo("Last command time %i %i", self.last_command_time.secs, self.last_command_time.nsecs)
        if self.invert_ptz:
            invert_command = -1.0
        else:
            invert_command = 1.0
        # Need to convert from rad to degree
        # relative motion
        if command.relative:
            new_pan = invert_command*command.pan + self.desired_pan
            new_tilt = invert_command*command.tilt + self.desired_tilt
            new_zoom = self.desired_zoom + command.zoom

        else:
            new_pan = invert_command*command.pan
            new_tilt = invert_command*command.tilt
            # new_pan = math.degrees(invert_command*command.pan)
            # new_tilt = math.degrees(invert_command*command.tilt)
            new_zoom = command.zoom
        # Applies limit restrictions
        if new_pan > self.max_pan_value:
            new_pan = self.max_pan_value
        elif new_pan < self.min_pan_value:
            new_pan = self.min_pan_value
        if new_tilt > self.max_tilt_value:
            new_tilt = self.max_tilt_value
        elif new_tilt < self.min_tilt_value:
            new_tilt = self.min_tilt_value
        if new_zoom > self.max_zoom_value:
            new_zoom = self.max_zoom_value
        elif new_zoom < self.min_zoom_value:
            new_zoom = self.min_zoom_value
            
        self.desired_pan = new_pan
        self.desired_tilt = new_tilt
        self.desired_zoom = new_zoom
        
    def homeService(self, req):
        
        # Set home values
        home_command = ptz()
        home_command.relative = False
        home_command.pan = self.home_pan_value
        home_command.tilt = self.home_tilt_value
        home_command.zoom = 0
        
        self.setCommandPTZ(home_command)
        if not self.send_constantly:
            self.sendPTZCommand()
        
        return {}
        
        
    def controlPTZ(self):
        """
            Performs the control of the camera ptz
        """
        # Only if it's syncronized
        if self.ptz_syncronized:
            #if self.isPTZinPosition():
            self.sendPTZCommand()
            #else:
            #rospy.logwarn('controlPTZ: not in  position')
        

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

    def sendPTZCommand(self):
        """
            Sends the ptz to the camera
        """
        pan = math.degrees(self.desired_pan)
        tilt = math.degrees(self.desired_tilt)
        #pan = self.desired_pan
        #tilt = self.desired_tilt
        zoom = self.desired_zoom
        try:
            conn = httplib.HTTPConnection(self.hostname)
        except:
            conn = http.client.HTTPConnection(self.hostname)
        params = { 'pan': pan, 'tilt': tilt, 'zoom': zoom }
        
        try:		
            #rospy.loginfo("AxisPTZ::cmd_ptz: pan = %f, tilt = %f, zoom = %f",pan, tilt, zoom)
            try:
                url = "/axis-cgi/com/ptz.cgi?camera=1&%s" % urllib.urlencode(params)
            except:
                url = "/axis-cgi/com/ptz.cgi?camera=1&%s" % urllib.parse.urlencode(params)

            #rospy.loginfo("AxisPTZ::cmd_ptz: %s",url)
            conn.request("GET", url)
            if conn.getresponse().status != 204:
                rospy.logerr('%s/sendPTZCommand: Error getting response. url = %s%s'% (rospy.get_name(), self.hostname, url) )
            #print("%s/axis-cgi/com/ptz.cgi?camera=1&%s. Response =%d" % (rospy.get_name(), urllib.urlencode(params), conn.getresponse().status))
        except socket.error as e:
            rospy.logerr('%s:sendPTZCommand: error connecting the camera: %s '%(rospy.get_name(),e))
        except socket.timeout as e:
            rospy.logerr('%s:sendPTZCommand: error connecting the camera: %s '%(rospy.get_name(),e))
            
        
    def getPTZState(self):
        """
            Gets the current ptz state/position of the camera
        """
        try:
            conn = httplib.HTTPConnection(self.hostname)
        except:
            conn = http.client.HTTPConnection(self.hostname)

        params = { 'query':'position' }
        try:
            try:
                conn.request("GET", "/axis-cgi/com/ptz.cgi?%s" % urllib.urlencode(params))
            except:
                conn.request("GET", "/axis-cgi/com/ptz.cgi?%s" % urllib.parse.urlencode(params))
            response = conn.getresponse()
            if response.status == 200:
                body = response.read()
                try:
                    params = dict([s.split('=',2) for s in body.splitlines()])
                except:
                    params = dict([s.decode().split('=',2) for s in body.splitlines()])
                self.current_ptz.pan = math.radians(float(params['pan']))
                self.current_ptz.tilt = math.radians(float(params['tilt']))
                
                if 'zoom' in params:
                    self.current_ptz.zoom = float(params['zoom'])
                else:
                    self.current_ptz.zoom = 0.0
                # Optional params (depending on model)
                if 'iris' in params:
                    self.current_ptz.iris = float(params['iris'])
                else:
                    self.current_ptz.iris = 0.0
                if 'focus' in params:
                    self.current_ptz.focus = float(params['focus'])
                else:
                    self.current_ptz.focus = 0.0
                if 'autofocus' in params:
                    self.current_ptz.autofocus = (params['autofocus'] == 'on')
                else:
                    self.current_ptz.autofocus = False
                if 'autoiris' in params:
                    self.current_ptz.autoiris = (params['autoiris'] == 'on')
                else:
                    self.current_ptz.autoiris = False
            
            # First time saves the current values
            if not self.ptz_syncronized:
                self.desired_pan = self.current_ptz.pan 
                self.desired_tilt = self.current_ptz.tilt
                self.desired_zoom = self.current_ptz.zoom
                #rospy.loginfo('%s:getPTZState: PTZ state syncronized!', rospy.get_name())
                self.ptz_syncronized = True
            
            self.error_reading = False
            
        except socket.error as e:
            rospy.logerr('%s:getPTZState: error connecting the camera: %s '%(rospy.get_name(),e))
            self.error_reading = True
            self.error_reading_msg = e
        except socket.timeout as e:
            rospy.logerr('%s:getPTZState: error connecting the camera: %s '%(rospy.get_name(),e))
            self.error_reading = True
            self.error_reading_msg = e
        except ValueError as e:
            rospy.logerr('%s:getPTZState: received corrupted data: %s '%(rospy.get_name(),e))
            self.error_reading = True
            self.error_reading_msg = e
            
        #print('Get state')
        #self.axis.pub.publish(self.msg)
        
        
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
            if self.run_control and self.send_constantly:
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
        
        msg.name = [self.pan_joint, self.tilt_joint]
        msg.position = [self.current_ptz.pan, self.current_ptz.tilt]
        msg.velocity = [0.0, 0.0]
        msg.effort = [0.0, 0.0]
        
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
        'password': 'R0b0tn1K',
        'username': 'root',
        'node_name': 'axis_camera',
        'camera_id': 'XXXX',  # internal id (if necessary)
        'camera_model': 'axis_m5525',
        'enable_auth': True,
        'autoflip': False,
        'eflip': False,
        'pan_joint': 'pan',
        'tilt_joint': 'tilt',
        'min_pan_value': -2.97,
        'max_pan_value': 2.97,
        'min_tilt_value': 0,
        'max_tilt_value': 1.57,
        'max_zoom_value': 20000,
        'min_zoom_value': 0,
        'home_pan_value': 0.0,
        'home_tilt_value': 0.79,
        'ptz_rate': 5.0,
        'error_pos': 0.02,
        'error_zoom': 99.0,
        'joint_states_topic': 'joint_states',
        'use_control_timeout': False,
        'control_timeout_value': 5.0,
        'invert_ptz': False,
        'initialization_delay': 0.0,  # time waiting before running
        'send_constantly': True
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