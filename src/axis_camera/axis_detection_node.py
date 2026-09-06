#!/usr/bin/env python

import threading

import rospy

from std_msgs.msg import Header
from robotnik_msgs.msg import AxisMetadataDetection, AxisMetadataDetectionArray
from robotnik_msgs.msg import ReturnMessage
from robotnik_msgs.srv import GetStringList, GetStringListResponse
from object_detection_msgs.msg import DetectorState, DetectorsState
from object_detection_msgs.srv import ManageDetector, ManageDetectorResponse

from axis_camera.axis_lib.axis_detection import AxisDetectionClient


class AxisDetectionNode(object):
    """ROS node that publishes AXIS analytics metadata detections."""

    DETECTOR_NAMES = ('person_detector', 'vehicle_detector')
    CLASS_TO_DETECTOR = {
        'human': 'person_detector',
        'vehicle': 'vehicle_detector',
    }

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
        self.pending_detections = {}
        self.pending_detections_lock = threading.Lock()
        self.detector_states = {
            'person_detector': bool(args['person_detector_enabled']),
            'vehicle_detector': bool(args['vehicle_detector_enabled']),
        }
        self.detector_states_lock = threading.Lock()

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
        self.detectors_states_pub = rospy.Publisher(
            '~detectors_states', DetectorsState, queue_size=10)
        self.activate_detector_service = rospy.Service(
            '~activate_detector', ManageDetector, self.activateDetectorCb)
        self.get_detectors_name_list_service = rospy.Service(
            '~get_detectors_name_list', GetStringList, self.getDetectorsNameListCb)
        rospy.on_shutdown(self.stop)

    def startDetection(self):
        self.detection_client.start(
            on_detections=self.collectDetections,
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

    def activateDetectorCb(self, request):
        detector_names = self._requestedDetectorNames(request.name)
        if detector_names is None:
            return ManageDetectorResponse(
                success=False,
                message='Unknown detector "%s". Available detectors: %s' % (
                    request.name, ', '.join(self.DETECTOR_NAMES)))

        with self.detector_states_lock:
            for detector_name in detector_names:
                rospy.loginfo('%s: setting detector "%s" to %s', rospy.get_name(), detector_name, request.active)
                self.detector_states[detector_name] = bool(request.active)

        return ManageDetectorResponse(
            success=True,
            message='Detector state updated: %s=%s' % (
                ', '.join(detector_names), request.active))

    def getDetectorsNameListCb(self, request):
        requested_name = str(request.data or '').strip()
        if requested_name:
            if requested_name not in self.DETECTOR_NAMES:
                return GetStringListResponse(
                    strings=[],
                    ret=ReturnMessage(
                        success=False,
                        message='Unknown detector "%s". Available detectors: %s' % (
                            requested_name, ', '.join(self.DETECTOR_NAMES))))
            detector_names = [requested_name]
        else:
            detector_names = list(self.DETECTOR_NAMES)

        return GetStringListResponse(
            strings=detector_names,
            ret=ReturnMessage(success=True, message='Detector states retrieved'))

    def publishDetectorStates(self):
        with self.detector_states_lock:
            detector_states = dict(self.detector_states)

        msg = DetectorsState()
        for detector_name in self.DETECTOR_NAMES:
            state = DetectorState()
            state.name = detector_name
            state.active = detector_states[detector_name]
            msg.detectors.append(state)
        self.detectors_states_pub.publish(msg)

    def _requestedDetectorNames(self, requested_name):
        requested_name = str(requested_name or '').strip()
        if requested_name == 'all':
            return list(self.DETECTOR_NAMES)
        if requested_name in self.DETECTOR_NAMES:
            return [requested_name]
        return None

    def collectDetections(self, detections):
        with self.pending_detections_lock:
            for detection in detections:
                channel = self._normalizeChannelName(detection.get('channel'))
                self.pending_detections.setdefault(channel, []).append(detection)

    def publishStatus(self):
        with self.pending_detections_lock:
            pending_detections = self.pending_detections
            self.pending_detections = {}

        for channel in pending_detections:
            self._getDetectionPublisher(channel)

        for channel in list(self.detection_publishers):
            self.publishChannelDetections(channel, pending_detections.get(channel, []))
        self.publishDetectorStates()

    def publishChannelDetections(self, channel, detections):
        with self.detector_states_lock:
            detector_states = dict(self.detector_states)

        msg = AxisMetadataDetectionArray()
        msg.header = Header(stamp=rospy.Time.now(), frame_id=self.frame_id)

        for detection in detections:
            detector_name = self.CLASS_TO_DETECTOR.get(detection['class_label'])
            if detector_name is None or not detector_states[detector_name]:
                continue

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
            self.publishStatus()
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
        'person_detector_enabled': True,
        'vehicle_detector_enabled': True,
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