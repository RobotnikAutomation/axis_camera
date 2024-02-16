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

import time
import datetime

import rospy

from sensor_msgs.msg import CompressedImage, CameraInfo
import camera_info_manager

import diagnostic_updater
import diagnostic_msgs
from axis_camera.axis_lib.axis_stream import StreamAxis


class Axis():
    """
            Class Axis. Intended to read video from the IP camera and publish to ROS
    """

    def __init__(self, args):
        """
                Init method.
                Initializes attributes and read passed args
        """
        self.enable_auth = args['enable_auth']
        self.hostname = args['hostname']
        self.username = 'root'
        self.password = args['password']
        self.camera_id = args['camera_id']
        self.camera_number = args['camera_number']
        self.camera_info_url = args['camera_info_url']
        self.camera_model = args['camera_model']
        self.fps = args['fps']
        self.compression = args['compression']
        self.axis_frame_id = args['frame']
        self.profile = args['profile']
        self.initialization_delay = args['initialization_delay']

        args_streamer = {
            'enable_auth' : self.enable_auth,
            'hostname' : self.hostname,
            'username' : self.username,
            'password' : self.password,
            'camera_number' : self.camera_number,
            'fps' : self.fps,
            'compression' : self.compression,
            'profile' : self.profile
        }

        # by default is stopped
        self.run_camera = False

        self.last_update = datetime.datetime.now()  # To control the data reception

        self.status = 'ERROR'

        # time between reconnections
        self.reconnection_time = 5

        self.subscribers = 0

        self.error_reading = False
        self.error_reading_msg = ''

        self.streamer = StreamAxis(args_streamer)

    def rosSetup(self):
        """
                Creates and setups ROS components
        """
        axis_node_name = rospy.get_name()
        self.cinfo = camera_info_manager.CameraInfoManager(
            cname=self.camera_model, url=self.camera_info_url, namespace=rospy.get_namespace()+axis_node_name)
        self.cinfo.loadCameraInfo()
        # Mirar de cambiar por ImageTransport
        self.compressed_image_publisher = rospy.Publisher(
            "%scompressed" % rospy.get_namespace(), CompressedImage, self, queue_size=10)
        self.caminfo_publisher = rospy.Publisher(
            "%scamera_info" % rospy.get_namespace(), CameraInfo, self, queue_size=10)

        # Sets the url
        self.url = self.streamer.getUrl()

        rospy.loginfo('Axis:rosSetup: Camera %s (%s:%d): url = %s' %
                      (self.camera_model, self.hostname, self.camera_number, self.url))

        # Diagnostic Updater
        self.diagnostics_updater = diagnostic_updater.Updater()
        self.diagnostics_updater.setHardwareID("%s-%s:%s" % (self.camera_model, self.camera_id, self.hostname))
        self.diagnostics_updater.add("Video stream updater", self.getStreamDiagnostic)
        #self.diagnostics_updater.add("Video stream frequency", self.getStreamFrequencyDiagnostic)

        if self.fps == 0:
            freq_bounds = {'min': 0.5, 'max': 100}
        else:
            freq_bounds = {'min': self.fps, 'max': self.fps}

        self.image_pub_freq = diagnostic_updater.HeaderlessTopicDiagnostic("%scompressed" % rospy.get_namespace(), self.diagnostics_updater,
                                                                           diagnostic_updater.FrequencyStatusParam(freq_bounds, 0.01, 1))

        # Creates a periodic callback to publish the diagnostics at desired freq
        self.diagnostics_timer = rospy.Timer(rospy.Duration(1.0), self.publishDiagnostics)

    def run(self):
        """
                Executes the main loop of the node
        """
        rospy.logwarn('%s:run: waiting %.3lf secs before running', rospy.get_name(), self.initialization_delay)
        time.sleep(self.initialization_delay)

        while not rospy.is_shutdown():

            try:
                if self.run_camera:
                    self.stream()

                # rospy.loginfo('stream')
            except:
                import traceback
                traceback.print_exc()

            #rospy.loginfo('Axis:run: Camera %s (%s:%d) reconnecting in %d seconds'%(self.camera_id, self.hostname, self.camera_number, self.reconnection_time))
            if not rospy.is_shutdown():
                time.sleep(self.reconnection_time)

    def peer_subscribe(self, topic_name, topic_publish, peer_publish):
        """
                Callback when a peer has subscribed from a topic
        """
        self.subscribers = self.subscribers + 1

        if not self.run_camera:
            rospy.loginfo('Axis:peer_subscribe: %s. Start reading from camera' % (rospy.get_name()))
            self.run_camera = True

    def peer_unsubscribe(self, topic_name, num_peers):
        """
                Callback when a peer has unsubscribed from a topic
        """
        self.subscribers = self.subscribers - 1

        if self.subscribers == 0 and self.run_camera:
            rospy.loginfo('Axis:peer_unsubscribe: %s. Stop reading from camera' % (rospy.get_name()))
            self.run_camera = False

    def stream(self):
        """
                Reads and process the streams from the camera
        """
        self.error_reading, self.error_reading_msg = self.streamer.stream()

        if self.error_reading:
            rospy.logerr('Axis:stream: Camera %s (%s:%d). Error: %s' %
                                    (self.camera_id, self.hostname, self.camera_number, self.error_reading_msg))
        else:
            self.runCamera()
    
    def runCamera(self):
        while self.run_camera and not rospy.is_shutdown():

            img = self.streamer.getImage()

            msg = CompressedImage()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = self.axis_frame_id
            msg.format = "jpeg"
            msg.data = img
            # publish image
            self.compressed_image_publisher.publish(msg)

            cimsg = self.cinfo.getCameraInfo()
            cimsg.header.stamp = msg.header.stamp
            cimsg.header.frame_id = self.axis_frame_id
            # publish camera info
            self.caminfo_publisher.publish(cimsg)

            #print(self.real_fps)
            self.last_update = datetime.datetime.now()
            self.error_reading = False
            self.image_pub_freq.tick()

    def publishDiagnostics(self, event):
        """
                Publishes the diagnostics at the desired rate
        """
        # Updates diagnostics
        self.diagnostics_updater.update()

    def rosShutdown(self):
        """
                Performs the shutdown of the ROS interfaces
        """
        pass

    def getStreamDiagnostic(self, stat):
        """
                Callback to analyze the reception of the video stream
        """
        if self.error_reading:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.ERROR,
                         "Error receiving video stream: %s" % self.error_reading_msg)
        else:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, "Getting video stream")
        stat.add("camera id", self.camera_id)
        stat.add("camera number", self.camera_number)
        stat.add("camera model", self.camera_model)
        stat.add("fps", str(self.fps))
        stat.add("compression", str(self.compression))
        stat.add("video_frame", self.axis_frame_id)
        stat.add("running", self.run_camera)

        return stat
    
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
        'enable_auth': True,
        'camera_number': 1,  # camera number
        'camera_id': 'XXXX',  # internal id (if necessary)
        'camera_model': 'axis_p5512',
        'profile': 'Test',
        'fps': 0,  # max
        'compression': 0,  # 0->100
        'camera_info_url': 'package://axis_camera/data/default_calibration.yaml',
        'frame': 'axis_camera1',
        'initialization_delay': 0.0,  # time waiting before running
    }
    args = {}

    for name in arg_defaults:

        param_name = '%s%s' % (axis_node_namespace, name)

        if rospy.search_param(param_name):
            args[name] = rospy.get_param(param_name)
        else:
            args[name] = arg_defaults[name]

    rospy.loginfo('%s: args: %s' % (axis_node_name, args))

    axis = Axis(args)
    axis.rosSetup()
    rospy.loginfo('%s: starting' % axis_node_name)

    axis.run()


if __name__ == "__main__":
	main()