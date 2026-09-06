#!/usr/bin/env python

import os
import sys
import threading
import unittest
from types import SimpleNamespace
from unittest.mock import patch

import rospy


PACKAGE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
SRC_PATH = os.path.join(PACKAGE_ROOT, 'src')
if SRC_PATH not in sys.path:
    sys.path.insert(0, SRC_PATH)

from axis_camera.axis_lib.axis_detection import AxisDetectionClient
from axis_camera.axis_detection_node import AxisDetectionNode


class AxisDetectionClientTest(unittest.TestCase):

    def test_normalize_channel_filter_defaults_to_channel_one(self):
        self.assertEqual(['1'], AxisDetectionClient.normalize_channel_filter(None))
        self.assertEqual(['1'], AxisDetectionClient.normalize_channel_filter([]))

    def test_normalize_channel_filter_accepts_string_and_list(self):
        self.assertEqual(['2'], AxisDetectionClient.normalize_channel_filter('2'))
        self.assertEqual(['1', '3'], AxisDetectionClient.normalize_channel_filter([1, '3']))

    def test_build_configure_payload(self):
        client = AxisDetectionClient('192.168.0.10', ws_source='source', channel_filter=['2'])
        self.assertEqual({
            'apiVersion': '1.0',
            'method': 'source:configure',
            'params': {'channelFilter': ['2']}
        }, client.build_configure_payload())

    def test_extract_observations(self):
        observations = [{'track_id': 1}]
        message = {
            'params': {
                'notification': {
                    'message': {
                        'data': {
                            'frame': {
                                'observations': observations
                            }
                        }
                    }
                }
            }
        }
        self.assertEqual(observations, AxisDetectionClient.extract_observations(message))
        self.assertIsNone(AxisDetectionClient.extract_observations({}))

    def test_normalize_class_label(self):
        self.assertEqual('human', AxisDetectionClient.normalize_class_label('Person'))
        self.assertEqual('vehicle', AxisDetectionClient.normalize_class_label('motor-bike'))
        self.assertEqual('unknown', AxisDetectionClient.normalize_class_label('tree'))

    def test_parse_detections_filters_invalid_observations(self):
        client = AxisDetectionClient('192.168.0.10')
        observations = [
            {
                'track_id': 42,
                'class': {'type': 'person', 'score': 0.75},
                'bounding_box': {'left': 0.1, 'top': 0.2, 'right': 0.3, 'bottom': 0.4}
            },
            {
                'track_id': 43,
                'class': {'type': 'tree', 'score': 0.9},
                'bounding_box': {'left': 0.1, 'top': 0.2, 'right': 0.3, 'bottom': 0.4}
            },
            {
                'track_id': 44,
                'class': {'type': 'car', 'score': 0.8},
                'bounding_box': {'left': 0.1, 'top': 0.2, 'right': 0.3}
            }
        ]

        self.assertEqual([
            {
                'channel': '1',
                'track_id': '42',
                'class_label': 'human',
                'score': 0.75,
                'left': 0.1,
                'top': 0.2,
                'right': 0.3,
                'bottom': 0.4,
            }
        ], client.parse_detections(observations, channel='1'))

    def test_extract_channel_from_frame(self):
        message = {
            'params': {
                'notification': {
                    'message': {
                        'data': {
                            'frame': {
                                'channel': 2,
                                'observations': []
                            }
                        }
                    }
                }
            }
        }
        self.assertEqual('2', AxisDetectionClient.extract_channel(message))

    def test_extract_channel_from_source_simple_item(self):
        message = {
            'params': {
                'notification': {
                    'message': {
                        'source': {
                            'simpleItem': [
                                {'name': 'VideoSourceToken', 'value': '3'}
                            ]
                        }
                    }
                }
            }
        }
        self.assertEqual('3', AxisDetectionClient.extract_channel(message))

    def test_parse_digest_challenge(self):
        challenge = 'Digest realm="AXIS", nonce="abc", algorithm=MD5, qop="auth"'
        self.assertEqual({
            'realm': 'AXIS',
            'nonce': 'abc',
            'algorithm': 'MD5',
            'qop': 'auth'
        }, AxisDetectionClient.parse_digest_challenge(challenge))

    def test_build_digest_authorization_header(self):
        client = AxisDetectionClient(
            '192.168.0.10',
            ws_source='analytics-scene-description',
            enable_auth=True,
            username='root',
            password='secret'
        )

        header = client._build_digest_authorization(
            'Digest realm="AXIS", nonce="abc", algorithm=MD5, qop="auth"'
        )

        self.assertTrue(header.startswith('Digest '))
        self.assertIn('username="root"', header)
        self.assertIn('realm="AXIS"', header)
        self.assertIn('nonce="abc"', header)
        self.assertIn('uri="/vapix/ws-data-stream?sources=analytics-scene-description"', header)
        self.assertIn('qop=auth', header)
        self.assertIn('nc=00000001', header)
        self.assertIn('response="', header)

    def test_detector_names_and_class_mapping(self):
        self.assertEqual(
            ('person_detector', 'vehicle_detector'),
            AxisDetectionNode.DETECTOR_NAMES)
        self.assertEqual('person_detector', AxisDetectionNode.CLASS_TO_DETECTOR['human'])
        self.assertEqual('vehicle_detector', AxisDetectionNode.CLASS_TO_DETECTOR['vehicle'])

    def test_requested_detector_names(self):
        self.assertEqual(
            ['person_detector', 'vehicle_detector'],
            AxisDetectionNode._requestedDetectorNames(
                object.__new__(AxisDetectionNode), 'all'))
        self.assertEqual(
            ['person_detector'],
            AxisDetectionNode._requestedDetectorNames(
                object.__new__(AxisDetectionNode), 'person_detector'))
        self.assertIsNone(
            AxisDetectionNode._requestedDetectorNames(
                object.__new__(AxisDetectionNode), 'unknown_detector'))

    def test_activate_detector_callback_supports_single_and_all(self):
        node = object.__new__(AxisDetectionNode)
        node.detector_states = {
            'person_detector': True,
            'vehicle_detector': True,
        }
        node.detector_states_lock = threading.Lock()

        response = node.activateDetectorCb(SimpleNamespace(
            name='person_detector', active=False))
        self.assertTrue(response.success)
        self.assertFalse(node.detector_states['person_detector'])
        self.assertTrue(node.detector_states['vehicle_detector'])

        response = node.activateDetectorCb(SimpleNamespace(name='all', active=False))
        self.assertTrue(response.success)
        self.assertEqual(
            {'person_detector': False, 'vehicle_detector': False},
            node.detector_states)

    def test_get_detector_states_callback_filters_by_request_data(self):
        node = object.__new__(AxisDetectionNode)
        node.detector_states = {
            'person_detector': True,
            'vehicle_detector': False,
        }
        node.detector_states_lock = threading.Lock()

        response = node.getDetectorsNameListCb(SimpleNamespace(data=''))
        self.assertTrue(response.ret.success)
        self.assertEqual(['person_detector=True', 'vehicle_detector=False'], response.strings)

        response = node.getDetectorsNameListCb(SimpleNamespace(data='vehicle_detector'))
        self.assertTrue(response.ret.success)
        self.assertEqual(['vehicle_detector=False'], response.strings)

        response = node.getDetectorsNameListCb(SimpleNamespace(data='unknown_detector'))
        self.assertFalse(response.ret.success)
        self.assertEqual([], response.strings)

    def test_publish_detector_states_and_filter_channel_detections(self):
        class PublisherStub(object):
            def __init__(self):
                self.messages = []

            def publish(self, message):
                self.messages.append(message)

        node = object.__new__(AxisDetectionNode)
        node.detector_states = {
            'person_detector': False,
            'vehicle_detector': True,
        }
        node.detector_states_lock = threading.Lock()
        node.detectors_states_pub = PublisherStub()
        node.detection_publishers = {'1': PublisherStub()}
        node.frame_id = 'axis_camera'

        node.publishDetectorStates()
        state_message = node.detectors_states_pub.messages[0]
        self.assertEqual(['person_detector', 'vehicle_detector'], [
            detector.name for detector in state_message.detectors])
        self.assertEqual([False, True], [
            detector.active for detector in state_message.detectors])

        with patch('rospy.Time.now', return_value=rospy.Time(0)):
            node.publishChannelDetections('1', [
                {'channel': '1', 'track_id': '1', 'class_label': 'human', 'score': 0.9,
                 'left': 0.1, 'top': 0.1, 'right': 0.2, 'bottom': 0.2},
                {'channel': '1', 'track_id': '2', 'class_label': 'vehicle', 'score': 0.8,
                 'left': 0.2, 'top': 0.2, 'right': 0.3, 'bottom': 0.3},
            ])
        detection_message = node.detection_publishers['1'].messages[0]
        self.assertEqual(['vehicle'], [
            detection.class_label for detection in detection_message.detections])


if __name__ == '__main__':
    unittest.main()