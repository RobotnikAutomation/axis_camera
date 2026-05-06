#!/usr/bin/env python

import json
import ssl
import threading

import rospy
from std_msgs.msg import Header
from std_msgs.msg import String

from axis_camera.msg import AxisMetadataDetection, AxisMetadataDetectionArray
from axis_camera.srv import SetDetectionFilter, SetDetectionFilterResponse

try:
    import websocket
except ImportError:
    websocket = None


class AxisMetadataDetectionNode(object):
    VALID_FILTERS = ('all', 'human', 'vehicle')

    def __init__(self):
        if websocket is None:
            raise RuntimeError('python websocket-client is required: pip install websocket-client')

        self.hostname = rospy.get_param('~hostname', '192.168.1.205')
        self.use_tls = bool(rospy.get_param('~use_tls', False))
        self.ws_source = rospy.get_param('~ws_source', 'analytics-scene-description')
        self.channel_filter = rospy.get_param('~channel_filter', ['1'])
        self.filter_class = str(rospy.get_param('~filter_class', 'all')).strip().lower()

        self.channel_filter = self._normalize_channel_filter(self.channel_filter)

        self.filter_class = self._normalize_filter_class(self.filter_class)

        # Publish initial value so rosparam get works from startup
        rospy.set_param('~filter_class', self.filter_class)

        self.pub_all = rospy.Publisher('~metadata_all', AxisMetadataDetectionArray, queue_size=10)
        self.pub_human = rospy.Publisher('~metadata_human', AxisMetadataDetectionArray, queue_size=10)
        self.pub_vehicle = rospy.Publisher('~metadata_vehicle', AxisMetadataDetectionArray, queue_size=10)
        self.pub_filtered = rospy.Publisher('~metadata_filtered', AxisMetadataDetectionArray, queue_size=10)
        self.pub_filter_status = rospy.Publisher('~filter_status', String, queue_size=1, latch=True)
        self.set_filter_srv = rospy.Service('~set_filter', SetDetectionFilter, self._set_filter_service_cb)

        scheme = 'wss' if self.use_tls else 'ws'
        self.ws_url = '%s://%s/vapix/ws-data-stream?sources=%s' % (scheme, self.hostname, self.ws_source)

        self._configured = threading.Event()
        self._stop = threading.Event()
        self._ws = None
        self._last_runtime_param_check = 0.0

        self._publish_filter_status()

    def _normalize_filter_class(self, raw_filter_class):
        normalized = str(raw_filter_class or '').strip().lower()
        if normalized not in self.VALID_FILTERS:
            rospy.logwarn('%s: invalid ~filter_class=%s, using all', rospy.get_name(), raw_filter_class)
            return 'all'
        return normalized

    def _publish_filter_status(self):
        self.pub_filter_status.publish(String(data=self.filter_class))

    def _set_filter_class(self, new_filter_class, source):
        normalized = str(new_filter_class or '').strip().lower()
        if normalized not in self.VALID_FILTERS:
            return False, 'invalid filter_class=%s, expected one of: %s' % (new_filter_class, ', '.join(self.VALID_FILTERS))

        if normalized == self.filter_class:
            rospy.set_param('~filter_class', self.filter_class)
            self._publish_filter_status()
            return True, 'filter already set to %s' % self.filter_class

        old_filter = self.filter_class
        self.filter_class = normalized
        rospy.set_param('~filter_class', self.filter_class)
        self._publish_filter_status()
        rospy.loginfo('%s: %s changed filter_class %s -> %s', rospy.get_name(), source, old_filter, self.filter_class)
        return True, 'filter changed from %s to %s' % (old_filter, self.filter_class)

    def _set_filter_service_cb(self, req):
        success, message = self._set_filter_class(req.filter_class, 'service')
        return SetDetectionFilterResponse(
            success=success,
            applied_filter=self.filter_class,
            message=message,
        )

    def _normalize_channel_filter(self, raw_filter):
        if isinstance(raw_filter, str):
            raw_filter = [raw_filter]
        if not isinstance(raw_filter, list) or len(raw_filter) == 0:
            raw_filter = ['1']
        return [str(ch) for ch in raw_filter]

    def _reload_runtime_params(self):
        now = rospy.Time.now().to_sec()
        if now - self._last_runtime_param_check < 1.0:
            return
        self._last_runtime_param_check = now

        new_filter_class = str(rospy.get_param('~filter_class', self.filter_class)).strip().lower()
        if new_filter_class not in self.VALID_FILTERS:
            rospy.logwarn_throttle(10, '%s: ignoring invalid ~filter_class=%s', rospy.get_name(), new_filter_class)
        elif new_filter_class != self.filter_class:
            self._set_filter_class(new_filter_class, 'param')

        raw_channel_filter = rospy.get_param('~channel_filter', self.channel_filter)
        new_channel_filter = self._normalize_channel_filter(raw_channel_filter)
        if new_channel_filter != self.channel_filter:
            self.channel_filter = new_channel_filter
            rospy.loginfo('%s: runtime channel_filter changed to %s', rospy.get_name(), self.channel_filter)
            try:
                if self._ws is not None:
                    self._ws.send(json.dumps(self._build_configure_payload()))
                    rospy.loginfo('%s: configure resent with channelFilter=%s', rospy.get_name(), self.channel_filter)
            except Exception as exc:
                rospy.logwarn('%s: failed to resend configure payload: %s', rospy.get_name(), exc)

    def _allowed_class(self, cls):
        normalized = str(cls or '').strip().lower()
        if self.filter_class == 'all':
            return normalized in ('human', 'vehicle')
        return normalized == self.filter_class

    def _build_configure_payload(self):
        return {
            'apiVersion': '1.0',
            'method': '%s:configure' % self.ws_source,
            'params': {
                'channelFilter': self.channel_filter,
            }
        }

    def _publish_detections(self, observations):
        header = Header(stamp=rospy.Time.now(), frame_id='axis_camera')

        out_all = AxisMetadataDetectionArray()
        out_all.header = header

        out_human = AxisMetadataDetectionArray()
        out_human.header = header

        out_vehicle = AxisMetadataDetectionArray()
        out_vehicle.header = header

        out_filtered = AxisMetadataDetectionArray()
        out_filtered.header = header

        for obs in observations:
            if not isinstance(obs, dict):
                continue

            bbox = obs.get('bounding_box', {})
            if not isinstance(bbox, dict):
                continue

            cls_obj = obs.get('class', {})
            if not isinstance(cls_obj, dict):
                cls_obj = {}

            class_label = str(cls_obj.get('type', 'unknown')).strip().lower()
            if class_label not in ('human', 'vehicle'):
                continue

            left = bbox.get('left')
            top = bbox.get('top')
            right = bbox.get('right')
            bottom = bbox.get('bottom')
            if None in (left, top, right, bottom):
                continue

            det = AxisMetadataDetection()
            det.header = header
            det.track_id = str(obs.get('track_id', ''))
            det.class_label = class_label if class_label else 'unknown'
            det.score = float(cls_obj.get('score', 0.0) or 0.0)
            det.left = float(left)
            det.top = float(top)
            det.right = float(right)
            det.bottom = float(bottom)

            out_all.detections.append(det)
            if class_label == 'human':
                out_human.detections.append(det)
            elif class_label == 'vehicle':
                out_vehicle.detections.append(det)

            if self._allowed_class(class_label):
                out_filtered.detections.append(det)

        if out_all.detections:
            self.pub_all.publish(out_all)
        if out_human.detections:
            self.pub_human.publish(out_human)
        if out_vehicle.detections:
            self.pub_vehicle.publish(out_vehicle)
        if out_filtered.detections:
            self.pub_filtered.publish(out_filtered)

    def _extract_observations(self, msg_obj):
        try:
            return msg_obj['params']['notification']['message']['data']['frame']['observations']
        except Exception:
            return None

    def _on_open(self, ws):
        rospy.loginfo('%s: connected to %s', rospy.get_name(), self.ws_url)
        payload = self._build_configure_payload()
        ws.send(json.dumps(payload))
        rospy.loginfo('%s: configure sent with channelFilter=%s', rospy.get_name(), self.channel_filter)

    def _on_message(self, ws, msg):
        self._reload_runtime_params()

        try:
            obj = json.loads(msg)
        except Exception:
            return

        error = obj.get('error')
        if isinstance(error, dict):
            rospy.logwarn_throttle(10, '%s: metadata ws error: %s', rospy.get_name(), error)
            return

        method = obj.get('method')
        if method == '%s:configure' % self.ws_source:
            self._configured.set()
            rospy.loginfo('%s: metadata configure accepted', rospy.get_name())
            return

        observations = self._extract_observations(obj)
        if isinstance(observations, list):
            self._publish_detections(observations)

    def _on_error(self, ws, err):
        rospy.logwarn_throttle(5, '%s: metadata ws error: %s', rospy.get_name(), err)

    def _on_close(self, ws, code, reason):
        rospy.logwarn('%s: metadata ws closed (%s, %s)', rospy.get_name(), code, reason)

    def spin(self):
        while not rospy.is_shutdown() and not self._stop.is_set():
            self._configured.clear()
            self._ws = websocket.WebSocketApp(
                self.ws_url,
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
                rospy.logwarn('%s: metadata ws exception: %s', rospy.get_name(), exc)

            if rospy.is_shutdown() or self._stop.is_set():
                break

            rospy.sleep(2.0)

    def stop(self):
        self._stop.set()
        try:
            if self._ws is not None:
                self._ws.close()
        except Exception:
            pass


def main():
    rospy.init_node('axis_detection_node')
    node = AxisMetadataDetectionNode()
    rospy.on_shutdown(node.stop)
    node.spin()


if __name__ == '__main__':
    main()
