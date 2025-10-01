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

import rclpy
import rclpy.time
from rclpy.node import Node

from axis_camera.axis_lib.axis_stream import StreamAxis
from camera_info_manager import CameraInfoManager, genCameraName
from sensor_msgs.msg import Image, CameraInfo, CompressedImage

from cv_bridge import CvBridge
import cv2
import numpy as np

class AxisStream(Node):
    """
        Class to handle the stream from the Axis camera.
        It reads the stream and publishes the images to a topic.
    """
    def __init__(self):
        # TODO: enable_rosout is set to False to avoid issues with zenoh #17
        super().__init__('axis_stream_node', enable_rosout=False)

        self.rosReadParams()
        self.streamer = StreamAxis({
            'hostname': self.hostname,
            'camera_number': self.camera_number,
            'fps': self.fps,
            'compression': self.compression,
            'profile': self.profile,
            'timeout': self.timeout,
            'videocodec': self.videocodec,
            'resolution': self.resolution
        })

        self.run_camera = False
        self.get_logger().info(f"Axis camera stream URL: {self.streamer.getUrl()}")
        self.publish_cam_info = False
        self.publish_img = False
        self.publish_compressed_img = False

        self.bridge = CvBridge()

        self.rosSetup()
        if self.initialization_delay > 0:
            self.get_logger().info("__init__:: Waiting for initialization delay of %.3lf seconds" % self.initialization_delay)
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=self.initialization_delay))

        self.create_timer(1/self.desired_freq, self.controlLoop)

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
        self.hostname = self.readParam('hostname', '192.168.0.185')
        self.camera_number = self.readParam('camera_number', 1)
        self.camera_id = self.readParam('camera_id', 'camera')
        self.camera_info_url = self.readParam('camera_info_url', 'package://axis_camera/data/default_calibration.yaml')
        self.fps = self.readParam('fps', 0)
        self.compression = self.readParam('compression', 0)
        self.axis_frame_id = self.readParam('axis_frame_id', 'axis_camera')
        self.profile = self.readParam('profile', 'Test')
        self.timeout = self.readParam('timeout', 5.0)
        self.videocodec = self.readParam('videocodec', 'jpeg')
        self.resolution = self.readParam('resolution', '1920x1080')
        self.initialization_delay = self.readParam('initialization_delay', 0.0)
        self.reconnection_time = self.readParam('reconnection_time', 5.0)
        self.desired_freq = self.readParam('desired_freq', 30.0)

    def rosSetup(self):
        """
        Sets up the ROS node, including subscribers and publishers.
        This method is called after the parameters are read.
        """
        self.camera_info = CameraInfoManager(self, cname = genCameraName(self.hostname), url = self.camera_info_url, namespace='/axis_stream')
        self.camera_info.loadCameraInfo()
        self.image_publisher = self.create_publisher(Image, '~/image_raw', 10)
        self.compressed_image_publisher = self.create_publisher(CompressedImage, '~/image_raw/compressed', 10)
        self.camera_info_publisher = self.create_publisher(CameraInfo, '~/camera_info', 10)

    def controlLoop(self):
        """
        Executes the control loop for the camera stream.
        """
        self.checkSubscriberCount()
        try:
            if self.run_camera:
                self.stream()

        except Exception as e:
            self.get_logger().error(f"controlLoop:: Error in Axis camera {self.camera_id} ({self.hostname}:{self.camera_number}): {e}")
            if rclpy.ok():
                self.get_clock().sleep_for(rclpy.duration.Duration(seconds=self.reconnection_time))

    def stream(self):
        error, error_msg = self.streamer.stream()
        if error:
            self.get_logger().error(f"stream:: Error streaming from Axis camera {self.camera_id} ({self.hostname}:{self.camera_number}): {error_msg}")
            return
        else:
            self.publishCamera()

    def publishCamera(self):
        stamp = self.get_clock().now().to_msg()

        if self.publish_compressed_img or self.publish_img:
            image = self.streamer.getImage()

            if self.publish_img:
                msg = self.convertToROSImage(image, encoding="bgr8")
                self.image_publisher.publish(msg)

            if self.publish_compressed_img:
                compressed_msg = CompressedImage()
                compressed_msg.header.stamp = stamp
                compressed_msg.header.frame_id = self.axis_frame_id
                compressed_msg.format = 'jpeg'
                compressed_msg.data = image
                self.compressed_image_publisher.publish(compressed_msg)

        if self.publish_cam_info:
            camera_info_msg = self.camera_info.getCameraInfo()
            camera_info_msg.header.stamp = stamp
            camera_info_msg.header.frame_id = self.axis_frame_id
            self.camera_info_publisher.publish(camera_info_msg)

    def convertToROSImage(self, image: bytes, encoding: str) -> Image:
        """
        Convert an OpenCV image to a ROS Image message.
        
        :param image: bytes array representing the image
        :param encoding: Encoding type (e.g., "bgr8", "mono8")
        :return: ROS Image message
        """
        np_array = np.frombuffer(image, np.uint8)
        cv_image = cv2.imdecode(np_array, cv2.IMREAD_COLOR)

        return self.bridge.cv2_to_imgmsg(cv_image, encoding = encoding)

    def checkSubscriberCount(self):
        """
        Checks the number of subscribers to the image topic.
        If there are subscribers, it starts the camera stream.
        If there are no subscribers, it stops the camera stream.
        """

        publish_img = self.image_publisher.get_subscription_count() > 0
        publish_compressed_img = self.compressed_image_publisher.get_subscription_count() > 0
        publish_cam_info = self.camera_info_publisher.get_subscription_count() > 0

        self.subscriberTransitionLogger(self.publish_img, publish_img, self.image_publisher.topic_name)
        self.subscriberTransitionLogger(self.publish_compressed_img, publish_compressed_img, self.compressed_image_publisher.topic_name)
        self.subscriberTransitionLogger(self.publish_cam_info, publish_cam_info, self.camera_info_publisher.topic_name)

        run_camera = publish_cam_info or publish_img or publish_compressed_img

        if not self.run_camera == run_camera:
            action = "Starting" if run_camera else "Stopping"
            self.get_logger().info(f"checkSubscriberCount:: {action} camera stream")

        self.run_camera = run_camera
        self.publish_cam_info = publish_cam_info
        self.publish_img = publish_img
        self.publish_compressed_img = publish_compressed_img

    def subscriberTransitionLogger(self, subs_before, subs_now, topic_name):
        if not subs_now == subs_before:
            string = "Subscribers" if subs_now else "No more subscribers"
            self.get_logger().info(f"subscriberTransitionLogger:: {string} detected on topic {topic_name}")

