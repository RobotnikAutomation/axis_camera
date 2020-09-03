#!/usr/bin/env python
#
# Basic PTZ node, based on documentation here:
#   http://www.axis.com/files/manuals/vapix_ptz_45621_en_1112.pdf
#

from robotnik_msgs.srv import set_digital_output
from robotnik_msgs.msg import inputs_outputs
import rospy
import os
import sys
import string
import time
import getopt
import threading
# import httplib, urllib
import httplib
import httplib2
import urllib
import urllib2

import base64
import roslib
roslib.load_manifest('axis_camera')


class AxisIO:
    def __init__(self, args):
        # def __init__(self, args):

        self.hostname = args['hostname']
        self.username = args['username']
        self.password = args['password']
        self.outputs = args['outputs']
        self.inputs = args['inputs']
        self.max_inputs_outputs = args['max_inputs_outputs']

        self.msg = inputs_outputs()
        self.msg.digital_outputs = []
        self.msg.digital_inputs = []
        self.msg.analog_outputs = []
        self.msg.analog_inputs = []
        self.encodedstring = base64.encodestring(self.username + ":" + self.password)[:-1]
        self.auth = "Basic %s" % self.encodedstring
        self.headers = {'Authorization': self.auth}
        self.url = 'http://%s/axis-cgi/io/port.cgi' % (self.hostname)
        rospy.logwarn("url = %s", self.url)
        for i in range(self.max_inputs_outputs):
            self.msg.digital_outputs.append(False)

        for i in range(self.max_inputs_outputs):
            self.msg.digital_inputs.append(False)

        rospy.loginfo("AxisIO: hostname=%s, outputs = %d, inputs = %d" %
                      (self.hostname, len(self.outputs), len(self.inputs)))

        self.pub = rospy.Publisher("%s/inputs_outputs" % rospy.get_name(), inputs_outputs)
        self.srv = rospy.Service('%s/set_digital_output' % rospy.get_name(),
                                 set_digital_output, self.srv_set_digital_outputs)

    def check_inputs_outputs(self):

        url = self.url + "?checkactive=1,2,3,4"
        # Axis param.cgi action
        try:
            req = urllib2.Request(url)
            ret = urllib2.urlopen(req)
        except urllib2.URLError, e:
            rospy.logerr('AxisIO::check_inputs_outputs: Exception %s' % (e))
            return
        except urllib2.HTTPError, e:
            rospy.logerr('AxisIO::check_inputs_outputs: Exception %s' % (e))
            return
        except Exception, e:
            rospy.logerr('AxisIO::check_inputs_outputs: Exception %s', e)
            return

        if ret.code == 200:
            msg = ret.read()
            msg2 = msg.split('\n')
            i = 0
            for m in msg2:
                if len(m) > 0:
                    msg3 = m.split('=')
                    if len(msg3) == 2:
                        port = msg3[0].split('port')
                        if len(port) == 2:
                            port_number = int(port[1])
                            if port_number in self.outputs:
                                self.msg.digital_outputs[port_number-1] = (msg3[1] == 'active')
                        # print "Port %d, value %s"%(i, str(msg3[1]))
                        # i = i + 1
                            elif port_number in self.inputs:
                                self.msg.digital_inputs[port_number-1] = (msg3[1] == 'active')

    def set_output(self, num_, val_):
        action = '\\'
        if val_ == True:
            action = '/'
        else:
            action = '\\'

        url_ = self.url + "?action=%d:%s" % (num_, action)

        try:
            req = urllib2.Request(url_)
            ret = urllib2.urlopen(req)
        except urllib2.URLError, e:
            rospy.logerr('AxisIO::set_output: Exception %s' % (e))
            return False
        except urllib2.HTTPError, e:
            rospy.logerr('AxisIO::set_output: Exception %s' % (e))
            return False
        except Exception, e:
            rospy.logerr('AxisIO::set_output: Exception %s', e)
            return False

        if ret.code == 200:
            return True
        else:
            return False

    # Service to set digital outputs

    def srv_set_digital_outputs(self, msg):
        if msg.output > self.max_inputs_outputs or msg.output <= 0:
            rospy.logerr('AxisIO::srv_set_digital_outputs: output %d out of range(%d)' %
                         (msg.output, self.max_inputs_outputs))
            return False

        if msg.output not in self.outputs:
            rospy.logerr('AxisIO::srv_set_digital_outputs: output %d is not configured as an output' %
                         (msg.output))
            return False

        return self.set_output(msg.output, msg.value)

    ##
    ##
    def run(self):

        r = rospy.Rate(2)

        while not rospy.is_shutdown():
            self.check_inputs_outputs()
            self.pub.publish(self.msg)
            r.sleep()


def main():
    rospy.init_node("axis_io")
    axis_name = rospy.get_name()
    arg_defaults = {
        'hostname': '192.168.1.102',
        'username': 'root',
        'password': '1234',
        'inputs': 0,
        'outputs': 0,
        'max_inputs_outputs': 4
    }
    args = {}

    for name in arg_defaults:
        param_name = '%s/%s' % (axis_name, name)

        if rospy.search_param(param_name):
            args[name] = rospy.get_param(param_name)
            # print 'arg %s = %s'%(name, args[name])
        else:
            args[name] = arg_defaults[name]

    # rospy.loginfo('axis_io: args =%s'%args)

    # print args
    axis = AxisIO(args)

    axis.run()


if __name__ == "__main__":
    main()
