#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
   Node to control an Axis PTZ camera in simulation

    Subcriptors:
        - color/command (robotnik_msgs/ptz): Command to move the camera
    Publishers:
        - status (std_msgs/String): Status of the component
        - status_stamped (robotnik_msgs/StringStamped): Status of the component with timestamp
        - top_ptz_camera_joint_pan_position_controller/command (std_msgs/Float64): Command to move the pan joint
        - top_ptz_camera_joint_tilt_position_controller/command (std_msgs/Float64): Command to move the tilt joint
        - top_ptz_camera_joint_zoom_position_controller/command (std_msgs/Float64): Command to move the zoom joint
    
    Services:
        - color/command (robotnik_msgs/set_ptz): Service to move the camera
        - home/command (robotnik_msgs/set_ptz_home): Service to move the camera to the home position  
"""

#pylint: disable=attribute-defined-outside-init
#pylint: disable=wildcard-import
#pylint: disable=useless-parent-delegation
#pylint: disable=unused-wildcard-import

from rcomponent.rcomponent import *

# Insert here general imports:
# import math


# Insert here msg and srv imports:
from std_msgs.msg import String, Float64
from robotnik_msgs.msg import StringStamped
#from robotnik_msgs.msg import Axis as AxisMsg
from robotnik_msgs.msg import ptz

from robotnik_msgs.srv import set_ptz, set_ptzRequest, set_ptzResponse
from robotnik_msgs.srv import set_ptz_home, set_ptz_homeRequest, set_ptz_homeResponse

class AxisPTZ(RComponent):
    """
    Interface to control an Axis PTZ camera in simulation
    """

    def __init__(self):
        super().__init__()

    def ros_read_params(self):
        """Gets params from param server"""
        super().ros_read_params()

        # get components params
        self._desired_freq = rospy.get_param('~desired_freq', 5.0)

        # Get the plublihsers topics names
        self.pan_joint_controller_topic = rospy.get_param('~pan_joint_controller_topic', 
                                                          'top_ptz_camera_joint_pan_position_controller/command')
        self.tilt_joint_controller_topic = rospy.get_param('~tilt_joint_controller_topic',
                                                            'top_ptz_camera_joint_tilt_position_controller/command')
        self.zoom_joint_controller_topic = rospy.get_param('~zoom_joint_controller_topic',
                                                            'top_ptz_camera_joint_zoom_position_controller/command')

        # Get the subscribers topics names
        self.subscriber_color_command_topic = rospy.get_param('~subscriber_rgb_comand_topic', 'color/command')

        # Get the services servers names
        self.server_color_comand_service = rospy.get_param('~server_rgb_comand_service', 'color/command')
        self.server_home_comand_service = rospy.get_param('~server_home_comand_service', 'home/command')

        # Get the camera limits
        self.pan_max = rospy.get_param('~max_pan_value', 3.1416)
        self.pan_min = rospy.get_param('~min_pan_value', -3.1416)
        self.tilt_max = rospy.get_param('~max_tilt_value', 1.57)
        self.tilt_min = rospy.get_param('~min_tilt_value', 0.0)
        self.zoom_max = rospy.get_param('~max_zoom_value', 30.0)
        self.zoom_min = rospy.get_param('~min_zoom_value', 0.0)

    def ros_setup(self):
        """Creates and inits ROS components"""

        super().ros_setup()

        # Publisher
        self.status_pub = rospy.Publisher('~status', String, queue_size=10)
        self.status_stamped_pub = rospy.Publisher('~status_stamped', StringStamped, queue_size=10)
        self.pub_command_pan_sim = rospy.Publisher(self.pan_joint_controller_topic, Float64, queue_size=10)
        self.pub_command_tilt_sim = rospy.Publisher(self.tilt_joint_controller_topic, Float64, queue_size=10)
        self.pub_command_zoom_sim = rospy.Publisher(self.zoom_joint_controller_topic, Float64, queue_size=10)

        # Subscriber
        self.subscriber_color_commands = rospy.Subscriber(self.subscriber_color_command_topic, ptz, 
                                                          self.sub_command_ptz_cb)

        # Services
        self.service_server_ptz_color_commands = rospy.Service(self.server_color_comand_service, set_ptz, 
                                                               self.command_service_cb)
        self.service_server_home_commands = rospy.Service(self.server_home_comand_service, set_ptz_home, 
                                                          self.home_service_cb)

        return 0

    def init_state(self):
        self.status = String()

        super().setup()
        return super().init_state()

    def ready_state(self):
        """Actions performed in ready state"""

        # Check topic health

        if self.check_topics_health() is False:
            self.switch_to_state(State.EMERGENCY_STATE)
            return super().ready_state()

        # Publish topic with status

        status_stamped = StringStamped()
        status_stamped.header.stamp = rospy.Time.now()
        status_stamped.string = self.status.data

        self.status_pub.publish(self.status)
        self.status_stamped_pub.publish(status_stamped)

        return super().ready_state()

    def emergency_state(self):
        if self.check_topics_health() is True :
            self.switch_to_state(State.READY_STATE)

    def shutdown(self):
        """Shutdowns device

        Return:
            0 : if it's performed successfully
            -1: if there's any problem or the component is running
        """

        return super().shutdown()

    def switch_to_state(self, new_state):
        """Performs the change of state"""

        return super().switch_to_state(new_state)

    def sub_command_ptz_cb(self, msg: ptz):
        """
        Callback to receive the command to move the camera
        """
        #self.tick_topics_health('example_sub')
        rospy.logwarn("Received ptz command to move the camera")

        # check limits of the camera
        if self.out_camera_joint_limits(msg.pan, msg.tilt, msg.zoom):
            return
        
        # perform the command
        self.perform_ptz_command(msg.pan, msg.tilt, msg.zoom)

    def command_service_cb(self, req: set_ptzRequest) -> bool:
        """
        Callback to receive the command to move the camera
        """
        rospy.logwarn("Received set_ptz srv request to move the camera.")

        response = set_ptzResponse()

        if self.out_camera_joint_limits(req.pan, req.tilt, req.zoom):
            response.ret = False
            return True

        # perform the command
        self.perform_ptz_command(req.pan, req.tilt, req.zoom)

        response.ret = True
        return True

    def home_service_cb(self, req: set_ptz_homeRequest) -> bool:
        """"
        Callback to receive the command to move the camera to the home position
        """
        rospy.logwarn("Received set_ptz_home srv request to move the camera.")
        response = set_ptz_homeResponse()

        # perform the command
        self.perform_ptz_command(0.0, 0.0, 0.0)

        response.ret = True
        return True

    def out_camera_joint_limits(self, pan: float, tilt: float, zoom: float) -> bool:
        """
        Check if the camera is out of the limits
        """
        if pan > self.pan_max or pan < self.pan_min:
            rospy.logwarn("Pan is out of limits")
            return True
        
        if tilt > self.tilt_max or tilt < self.tilt_min:
            rospy.logwarn("Tilt is out of limits")
            return True

        if zoom > self.zoom_max or zoom < self.zoom_min:
            rospy.logwarn("Zoom is out of limits")
            return True
        
        return False
    
    def perform_ptz_command(self, pan: float, tilt: float, zoom: float):
        """
        Perform the command to move the camera
        """
        self.pub_command_pan_sim.publish(pan)
        self.pub_command_tilt_sim.publish(tilt)
        self.pub_command_zoom_sim.publish(zoom)