#!/usr/bin/env python

import rospy

from std_msgs.msg import Header
from robotnik_msgs.msg import AxisMetadataDetection, AxisMetadataDetectionArray

from axis_camera.axis_lib.axis_detection import AxisDetectionClient


class AxisDetectionNode(object):
    """ROS node that publishes AXIS analytics metadata detections."""

    def __init__(self, args):
        self.hostname = args['hostname']
        self.enabled = args['detection_enabled']
        self.use_tls = args['detection_use_tls']
        self.ws_source = args['detection_ws_source']
        self.channel_filter = args['detection_channel_filter']
        self.enable_auth = args['enable_auth']
        self.username = args['username']
        self.password = args['password']
        self.frame_id = args['frame_id']
        self.rate = args['rate']
        self.detection_publishers = {}

        self.detection_client = AxisDetectionClient(
            hostname=self.hostname,
            enabled=self.enabled,
            use_tls=self.use_tls,
            ws_source=self.ws_source,
            channel_filter=self.channel_filter,
            enable_auth=self.enable_auth,
            username=self.username,
            password=self.password,
            sleep_fn=rospy.sleep,
        )

    def rosSetup(self):
        for channel in AxisDetectionClient.normalize_channel_filter(self.channel_filter):
            self._getDetectionPublisher(channel)
        rospy.on_shutdown(self.stop)

    def startDetection(self):
        self.detection_client.start(
            on_detections=self.publishDetections,
            on_connected=self.onConnected,
            on_error=self.onError,
            on_unsupported=self.onUnsupported,
        )

    def stop(self):
        self.detection_client.stop()

    def onConnected(self, url):
        rospy.loginfo('%s: metadata stream connected to %s', rospy.get_name(), url)

    def onError(self, message):
        rospy.logwarn_throttle(5, '%s: %s', rospy.get_name(), message)

    def onUnsupported(self, url):
        rospy.loginfo('%s: camera does not support analytics metadata endpoint %s -- detection disabled', rospy.get_name(), url)

    def publishDetections(self, detections):
        detections_by_channel = {}
        for detection in detections:
            channel = str(detection.get('channel') or 'unknown')
            detections_by_channel.setdefault(channel, []).append(detection)

        for channel, channel_detections in detections_by_channel.items():
            self.publishChannelDetections(channel, channel_detections)

    def publishChannelDetections(self, channel, detections):
        msg = AxisMetadataDetectionArray()
        msg.header = Header(stamp=rospy.Time.now(), frame_id=self.frame_id)

        for detection in detections:
            det = AxisMetadataDetection()
            det.header = msg.header
            det.track_id = detection['track_id']
            det.class_label = detection['class_label']
            det.score = detection['score']
            det.left = detection['left']
            det.top = detection['top']
            det.right = detection['right']
            det.bottom = detection['bottom']
            msg.detections.append(det)

        if msg.detections:
            self._getDetectionPublisher(channel).publish(msg)

    def _getDetectionPublisher(self, channel):
        channel = self._normalizeChannelName(channel)
        if channel not in self.detection_publishers:
            topic = "~detectors/%s/status" % channel
            self.detection_publishers[channel] = rospy.Publisher(topic, AxisMetadataDetectionArray, queue_size=10)
        return self.detection_publishers[channel]

    @staticmethod
    def _normalizeChannelName(channel):
        channel = str(channel or 'unknown').strip()
        channel = ''.join(char if char.isalnum() or char == '_' else '_' for char in channel)
        return channel or 'unknown'

    def run(self):
        self.startDetection()
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            rate.sleep()


def main():
    rospy.init_node("axis_detection")

    axis_node_name = rospy.get_name()

    arg_defaults = {
        'hostname': '192.168.1.205',
        'detection_enabled': True,
        'detection_use_tls': False,
        'detection_ws_source': 'analytics-scene-description',
        'detection_channel_filter': ['1'],
        'enable_auth': False,
        'username': 'root',
        'password': '',
        'frame_id': 'axis_camera',
        'rate': 1.0,
    }
    args = {}

    for name in arg_defaults:
        args[name] = rospy.get_param('~%s' % name, arg_defaults[name])

    log_args = dict(args)
    log_args['password'] = '***'
    rospy.loginfo('%s: args: %s' % (axis_node_name, log_args))

    axis_detection = AxisDetectionNode(args)
    axis_detection.rosSetup()
    rospy.loginfo('%s: starting' % axis_node_name)
    axis_detection.run()


if __name__ == "__main__":
    main()