#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
    Node to control an Axis PTZ camera in simulation
"""

import rospy
from axis_camera.axis_ptz_sim import AxisPTZSim

def main():
    """
    Main function to initialize the ROS node and start the AxisPTZSim node.
    This function initializes a ROS node named "axis_node_sim_node", creates an instance
    of the AxisPTZSim class, logs the start of the node, and starts the AxisPTZSim node.
    """

    rospy.init_node("axis_node_sim_node")

    rc_node = AxisPTZSim()

    rospy.loginfo(f"{rospy.get_name()}: starting")

    rc_node.start()


if __name__ == "__main__":
    main()
