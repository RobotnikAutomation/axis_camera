#!/usr/bin/env python

import os
import sys
import unittest


PACKAGE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
SRC_PATH = os.path.join(PACKAGE_ROOT, 'src')
if SRC_PATH not in sys.path:
    sys.path.insert(0, SRC_PATH)

from axis_camera.axis_lib.axis_detection import AxisDetectionClient


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


if __name__ == '__main__':
    unittest.main()