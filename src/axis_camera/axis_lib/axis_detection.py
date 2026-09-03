#!/usr/bin/env python

import hashlib
import json
import os
import re
import ssl
import threading

try:
    from urllib.parse import urlsplit
except ImportError:
    from urlparse import urlsplit

try:
    import websocket
except ImportError:
    websocket = None
else:
    if not hasattr(websocket, 'WebSocketApp'):
        try:
            from websocket._app import WebSocketApp
        except ImportError:
            websocket = None
        else:
            websocket.WebSocketApp = WebSocketApp


class AxisDetectionClient(object):
    """Client for AXIS analytics metadata over VAPIX WebSocket."""

    def __init__(self, hostname, enabled=True, use_tls=False,
                 ws_source='analytics-scene-description', channel_filter=None,
                 enable_auth=False, username='', password='', sleep_fn=None, logger=None):
        self.hostname = hostname
        self.enabled = enabled
        self.use_tls = bool(use_tls)
        self.ws_source = ws_source
        self.channel_filter = self.normalize_channel_filter(channel_filter)
        self.enable_auth = bool(enable_auth)
        self.username = username
        self.password = password
        self.sleep_fn = sleep_fn
        self.logger = logger

        self.on_detections = None
        self.on_connected = None
        self.on_error = None
        self.on_unsupported = None

        self._stop = threading.Event()
        self._thread = None
        self._ws = None
        self._unsupported = False
        self._connected = False
        self._authorization_header = None
        self._digest_nonce_count = 0

        scheme = 'wss' if self.use_tls else 'ws'
        self.url = '%s://%s/vapix/ws-data-stream?sources=%s' % (
            scheme,
            self.hostname,
            self.ws_source
        )

    @staticmethod
    def parse_digest_challenge(challenge):
        if not challenge or not challenge.lower().startswith('digest '):
            return None

        values = {}
        challenge = challenge[len('Digest '):]
        for match in re.finditer(r'(\w+)=("(?:[^"\\]|\\.)*"|[^,]+)', challenge):
            key = match.group(1)
            value = match.group(2).strip()
            if value.startswith('"') and value.endswith('"'):
                value = value[1:-1]
            values[key] = value
        return values

    @staticmethod
    def normalize_channel_filter(raw_filter):
        if isinstance(raw_filter, str):
            raw_filter = [raw_filter]
        if not isinstance(raw_filter, list) or len(raw_filter) == 0:
            raw_filter = ['1']
        return [str(channel) for channel in raw_filter]

    @staticmethod
    def extract_observations(msg_obj):
        try:
            return msg_obj['params']['notification']['message']['data']['frame']['observations']
        except Exception:
            return None

    @staticmethod
    def extract_channel(msg_obj):
        candidates = [
            ('params', 'channel'),
            ('params', 'notification', 'channel'),
            ('params', 'notification', 'message', 'channel'),
            ('params', 'notification', 'message', 'data', 'channel'),
            ('params', 'notification', 'message', 'data', 'frame', 'channel'),
        ]

        for path in candidates:
            value = AxisDetectionClient._get_nested_value(msg_obj, path)
            if value is not None and str(value).strip() != '':
                return str(value)

        source = AxisDetectionClient._get_nested_value(msg_obj, ('params', 'notification', 'message', 'source'))
        return AxisDetectionClient._extract_channel_from_source(source)

    @staticmethod
    def _get_nested_value(obj, path):
        current = obj
        for key in path:
            if not isinstance(current, dict):
                return None
            current = current.get(key)
        return current

    @staticmethod
    def _extract_channel_from_source(source):
        if not isinstance(source, dict):
            return None

        for key in ('channel', 'Channel', 'video_channel', 'VideoChannel'):
            value = source.get(key)
            if value is not None and str(value).strip() != '':
                return str(value)

        simple_items = source.get('simpleItem') or source.get('simpleItems') or source.get('SimpleItem')
        if isinstance(simple_items, dict):
            simple_items = [simple_items]
        if not isinstance(simple_items, list):
            return None

        channel_names = {'channel', 'videochannel', 'videosource', 'videosourcetoken', 'source'}
        for item in simple_items:
            if not isinstance(item, dict):
                continue
            name = str(item.get('name', item.get('Name', ''))).replace('_', '').lower()
            value = item.get('value', item.get('Value'))
            if name in channel_names and value is not None and str(value).strip() != '':
                return str(value)
        return None

    @staticmethod
    def extract_raw_class_label(observation):
        class_obj = observation.get('class', {})
        if isinstance(class_obj, dict):
            for key in ('type', 'label', 'name', 'object'):
                value = class_obj.get(key)
                if value is not None and str(value).strip() != '':
                    return str(value)
        elif class_obj is not None and str(class_obj).strip() != '':
            return str(class_obj)
        return ''

    @staticmethod
    def normalize_class_label(raw_label):
        label = str(raw_label or '').strip().lower().replace('-', '_').replace(' ', '_')

        human_labels = {
            'human', 'person', 'pedestrian', 'people', 'man', 'woman'
        }
        vehicle_labels = {
            'vehicle', 'car', 'truck', 'bus', 'van', 'motorcycle', 'motorbike', 'motor_bike', 'bike', 'bicycle'
        }

        if label in human_labels:
            return 'human'
        if label in vehicle_labels:
            return 'vehicle'
        return 'unknown'

    def build_configure_payload(self):
        return {
            'apiVersion': '1.0',
            'method': '%s:configure' % self.ws_source,
            'params': {
                'channelFilter': self.channel_filter,
            }
        }

    def parse_detections(self, observations, channel=None):
        detections = []
        if not isinstance(observations, list):
            return detections

        for observation in observations:
            detection = self._parse_detection_observation(observation, channel)
            if detection is not None:
                detections.append(detection)

        return detections

    def start(self, on_detections=None, on_connected=None, on_error=None, on_unsupported=None):
        self.on_detections = on_detections
        self.on_connected = on_connected
        self.on_error = on_error
        self.on_unsupported = on_unsupported

        if not self.enabled:
            self._log('info', 'metadata stream disabled')
            return False
        if websocket is None:
            self._emit_error('metadata stream disabled: python websocket-client not installed or wrong websocket package installed')
            return False
        if self._thread is not None:
            return True

        self._stop.clear()
        self._thread = threading.Thread(target=self._spin)
        self._thread.daemon = True
        self._thread.start()
        return True

    def stop(self):
        self._stop.set()
        try:
            if self._ws is not None:
                self._ws.close()
        except Exception:
            pass

    def is_connected(self):
        return self._connected

    def is_unsupported(self):
        return self._unsupported

    def _parse_detection_observation(self, observation, channel=None):
        if not isinstance(observation, dict):
            return None

        bbox = observation.get('bounding_box', {})
        if not isinstance(bbox, dict):
            return None

        left = bbox.get('left')
        top = bbox.get('top')
        right = bbox.get('right')
        bottom = bbox.get('bottom')
        if None in (left, top, right, bottom):
            return None

        class_obj = observation.get('class', {})
        if not isinstance(class_obj, dict):
            class_obj = {}

        class_label = self.normalize_class_label(self.extract_raw_class_label(observation))
        if class_label not in ('human', 'vehicle'):
            return None

        return {
            'channel': self._get_detection_channel(observation, channel),
            'track_id': str(observation.get('track_id', '')),
            'class_label': class_label,
            'score': float(class_obj.get('score', 0.0) or 0.0),
            'left': float(left),
            'top': float(top),
            'right': float(right),
            'bottom': float(bottom),
        }

    def _on_open(self, ws):
        self._unsupported = False
        self._connected = True
        ws.send(json.dumps(self.build_configure_payload()))
        if self.on_connected is not None:
            self.on_connected(self.url)

    def _on_message(self, ws, message):
        try:
            msg_obj = json.loads(message)
        except Exception:
            return

        error = msg_obj.get('error')
        if isinstance(error, dict):
            self._emit_error('metadata ws error: %s' % error)
            return

        method = msg_obj.get('method')
        if method == '%s:configure' % self.ws_source:
            return

        channel = self.extract_channel(msg_obj) or self._default_channel()
        detections = self.parse_detections(self.extract_observations(msg_obj), channel)
        if detections and self.on_detections is not None:
            self.on_detections(detections)

    def _on_error(self, ws, error):
        error_text = str(error)
        if '404' in error_text or 'Not Found' in error_text:
            if not self._unsupported:
                self._unsupported = True
                self._stop.set()
                if self.on_unsupported is not None:
                    self.on_unsupported(self.url)
            return

        if self._handle_unauthorized_error(error):
            return

        self._emit_error('metadata ws error: %s' % error)

    def _on_close(self, ws, code, reason):
        self._connected = False
        if not self._stop.is_set() and not self._unsupported:
            self._emit_error('metadata ws closed (%s, %s)' % (code, reason))

    def _spin(self):
        while not self._stop.is_set():
            self._ws = websocket.WebSocketApp(
                self.url,
                header=self._build_headers(),
                on_open=self._on_open,
                on_message=self._on_message,
                on_error=self._on_error,
                on_close=self._on_close,
            )
            try:
                if self.use_tls:
                    self._ws.run_forever(
                        sslopt={'cert_reqs': ssl.CERT_NONE},
                        ping_interval=20,
                        ping_timeout=10,
                    )
                else:
                    self._ws.run_forever(ping_interval=20, ping_timeout=10)
            except Exception as exc:
                self._connected = False
                self._emit_error('metadata ws exception: %s' % exc)

            if self._stop.is_set():
                break

            if self.sleep_fn is not None:
                self.sleep_fn(2.0)

    def _build_headers(self):
        if self._authorization_header is None:
            return None
        return {'Authorization': self._authorization_header}

    def _default_channel(self):
        if len(self.channel_filter) == 1:
            return self.channel_filter[0]
        return None

    @staticmethod
    def _get_detection_channel(observation, fallback_channel=None):
        for key in ('channel', 'video_channel', 'source'):
            value = observation.get(key)
            if value is not None and str(value).strip() != '':
                return str(value)
        return fallback_channel

    def _handle_unauthorized_error(self, error):
        if not self.enable_auth:
            return False

        status_code = getattr(error, 'status_code', None)
        if status_code != 401:
            return False

        response_headers = getattr(error, 'resp_headers', {}) or {}
        challenge = response_headers.get('www-authenticate') or response_headers.get('WWW-Authenticate')
        authorization = self._build_digest_authorization(challenge)
        if authorization is None:
            return False

        self._authorization_header = authorization
        self._log('info', 'metadata ws received Digest challenge, retrying with credentials')
        return True

    def _build_digest_authorization(self, challenge):
        values = self.parse_digest_challenge(challenge)
        if not values:
            return None

        realm = values.get('realm')
        nonce = values.get('nonce')
        if not realm or not nonce:
            return None

        algorithm = values.get('algorithm', 'MD5').upper()
        if algorithm not in ('MD5', 'MD5-SESS'):
            return None

        method = 'GET'
        uri = self._get_digest_uri()
        qop = self._select_digest_qop(values.get('qop'))
        opaque = values.get('opaque')
        cnonce = hashlib.md5(os.urandom(16)).hexdigest()

        ha1 = hashlib.md5(('%s:%s:%s' % (self.username, realm, self.password)).encode('utf-8')).hexdigest()
        if algorithm == 'MD5-SESS':
            ha1 = hashlib.md5(('%s:%s:%s' % (ha1, nonce, cnonce)).encode('utf-8')).hexdigest()
        ha2 = hashlib.md5(('%s:%s' % (method, uri)).encode('utf-8')).hexdigest()

        response_values = [
            'username="%s"' % self.username,
            'realm="%s"' % realm,
            'nonce="%s"' % nonce,
            'uri="%s"' % uri,
            'algorithm=%s' % algorithm,
        ]

        if opaque:
            response_values.append('opaque="%s"' % opaque)

        if qop:
            self._digest_nonce_count += 1
            nonce_count = '%08x' % self._digest_nonce_count
            response = hashlib.md5(('%s:%s:%s:%s:%s:%s' % (
                ha1, nonce, nonce_count, cnonce, qop, ha2
            )).encode('utf-8')).hexdigest()
            response_values.extend([
                'qop=%s' % qop,
                'nc=%s' % nonce_count,
                'cnonce="%s"' % cnonce,
                'response="%s"' % response,
            ])
        else:
            response = hashlib.md5(('%s:%s:%s' % (ha1, nonce, ha2)).encode('utf-8')).hexdigest()
            response_values.append('response="%s"' % response)

        return 'Digest %s' % ', '.join(response_values)

    def _get_digest_uri(self):
        parsed_url = urlsplit(self.url)
        uri = parsed_url.path or '/'
        if parsed_url.query:
            uri = '%s?%s' % (uri, parsed_url.query)
        return uri

    @staticmethod
    def _select_digest_qop(qop_value):
        if not qop_value:
            return None
        qop_options = [value.strip() for value in qop_value.split(',')]
        if 'auth' in qop_options:
            return 'auth'
        return None

    def _emit_error(self, message):
        self._log('warn', message)
        if self.on_error is not None:
            self.on_error(message)

    def _log(self, level, message):
        if self.logger is None:
            return
        log_method = getattr(self.logger, level, None)
        if log_method is not None:
            log_method(message)