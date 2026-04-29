try:
    import urllib.parse as urllib_parse
except:
	import urllib as urllib_parse #Not tested in pyhton2

try:
    import urllib.request as urllib_request
    import urllib.error as urllib_error
except:
    import urllib2 as urllib_request
    import urllib2 as urllib_error

try:
    import httplib
except:
    import http.client as httplib

import socket
import math
import xml.etree.ElementTree as ET
import json

class ControlAxis():
    def __init__(self, hostname, username='root', password=''):
        self.hostname = hostname
        self._username = username
        self._password = password
        self._sensor_parameter_names = None
        self._white_balance_parameter_path = None
        self._white_balance_supported_modes = None
        self._day_night_parameter_path = None
        self._day_night_supported_modes = None
        self._parameter_int_ranges = {}
        self._parameter_enum_values = {}
        self._last_known_image_settings = {
            'brightness': None,
            'contrast': None,
            'saturation': None,
            'white_balance': None,
            'is_night_mode_active': None,
            'day_night_shift_level': None
        }

    def _get_parameter_int_range_from_definitions(self, group, parameter_name):
        cache_key = '%s.%s' % (group, parameter_name)
        if cache_key in self._parameter_int_ranges:
            return self._parameter_int_ranges[cache_key]

        try:
            opener = self._get_digest_opener()
            params = urllib_parse.urlencode({
                'action': 'listdefinitions',
                'listformat': 'xmlschema',
                'group': group
            })
            url = 'http://%s/axis-cgi/admin/param.cgi?%s' % (self.hostname, params)
            response = opener.open(url, timeout=5)
            xml_text = response.read().decode('utf-8', errors='replace')
            root = ET.fromstring(xml_text)
            ns = {'ax': 'http://www.axis.com/ParameterDefinitionsSchema'}

            xpath = './/ax:parameter[@name="%s"]' % parameter_name
            parameter_node = root.find(xpath, ns)
            if parameter_node is None:
                self._parameter_int_ranges[cache_key] = (None, None)
                return (None, None)

            int_node = parameter_node.find('.//ax:int', ns)
            if int_node is None:
                self._parameter_int_ranges[cache_key] = (None, None)
                return (None, None)

            min_value = int_node.get('min')
            max_value = int_node.get('max')
            if min_value is None or max_value is None:
                self._parameter_int_ranges[cache_key] = (None, None)
                return (None, None)

            parsed = (int(min_value), int(max_value))
            self._parameter_int_ranges[cache_key] = parsed
            return parsed
        except Exception:
            self._parameter_int_ranges[cache_key] = (None, None)
            return (None, None)

    def _get_parameter_enum_values_from_definitions(self, group, parameter_name):
        cache_key = '%s.%s' % (group, parameter_name)
        if cache_key in self._parameter_enum_values:
            return self._parameter_enum_values[cache_key]

        try:
            opener = self._get_digest_opener()
            params = urllib_parse.urlencode({
                'action': 'listdefinitions',
                'listformat': 'xmlschema',
                'group': group
            })
            url = 'http://%s/axis-cgi/admin/param.cgi?%s' % (self.hostname, params)
            response = opener.open(url, timeout=5)
            xml_text = response.read().decode('utf-8', errors='replace')
            root = ET.fromstring(xml_text)
            ns = {'ax': 'http://www.axis.com/ParameterDefinitionsSchema'}

            xpath = './/ax:parameter[@name="%s"]' % parameter_name
            parameter_node = root.find(xpath, ns)
            if parameter_node is None:
                self._parameter_enum_values[cache_key] = None
                return None

            entries = parameter_node.findall('.//ax:enum/ax:entry', ns)
            if not entries:
                self._parameter_enum_values[cache_key] = None
                return None

            values = set(entry.get('value') for entry in entries if entry.get('value') is not None)
            result = values if values else None
            self._parameter_enum_values[cache_key] = result
            return result
        except Exception:
            self._parameter_enum_values[cache_key] = None
            return None

    def getImageSettingsMetadata(self):
        metadata = {
            'brightness_min': -100,
            'brightness_max': 100,
            'contrast_min': -100,
            'contrast_max': 100,
            'saturation_min': -100,
            'saturation_max': 100,
            'white_balance_available': False,
            'day_night_available': False,
            'day_night_shift_level_min': 0,
            'day_night_shift_level_max': 100
        }

        brightness_range = self._get_parameter_int_range_from_definitions('ImageSource.I0.Sensor', 'Brightness')
        if brightness_range[0] is not None:
            metadata['brightness_min'] = brightness_range[0]
            metadata['brightness_max'] = brightness_range[1]

        contrast_range = self._get_parameter_int_range_from_definitions('ImageSource.I0.Sensor', 'Contrast')
        if contrast_range[0] is not None:
            metadata['contrast_min'] = contrast_range[0]
            metadata['contrast_max'] = contrast_range[1]

        saturation_range = self._get_parameter_int_range_from_definitions('ImageSource.I0.Sensor', 'Saturation')
        if saturation_range[0] is None:
            saturation_range = self._get_parameter_int_range_from_definitions('ImageSource.I0.Sensor', 'ColorLevel')
        if saturation_range[0] is not None:
            metadata['saturation_min'] = saturation_range[0]
            metadata['saturation_max'] = saturation_range[1]

        shift_range = self._get_parameter_int_range_from_definitions('ImageSource.I0.DayNight', 'ShiftLevel')
        if shift_range[0] is not None:
            metadata['day_night_shift_level_min'] = shift_range[0]
            metadata['day_night_shift_level_max'] = shift_range[1]

        wb_path, _ = self._get_white_balance_parameter_path()
        metadata['white_balance_available'] = wb_path is not None

        dn_path, _ = self._get_day_night_parameter_path()
        metadata['day_night_available'] = dn_path is not None

        return metadata

    def _get_digest_opener(self):
        password_mgr = urllib_request.HTTPPasswordMgrWithDefaultRealm()
        password_mgr.add_password(None, 'http://' + self.hostname, self._username, self._password)
        auth_handler = urllib_request.HTTPDigestAuthHandler(password_mgr)
        return urllib_request.build_opener(auth_handler)

    def _list_sensor_parameters(self):
        if self._sensor_parameter_names is not None:
            return self._sensor_parameter_names

        opener = self._get_digest_opener()
        params = urllib_parse.urlencode({
            'action': 'list',
            'group': 'ImageSource.I0.Sensor'
        })
        url = 'http://%s/axis-cgi/admin/param.cgi?%s' % (self.hostname, params)
        response = opener.open(url, timeout=5)
        body = response.read().decode('utf-8').splitlines()

        parameter_names = set()
        for line in body:
            if '=' not in line:
                continue
            key = line.split('=', 1)[0].strip()
            prefix = 'root.ImageSource.I0.Sensor.'
            if key.startswith(prefix):
                parameter_names.add(key[len(prefix):])

        self._sensor_parameter_names = parameter_names
        return self._sensor_parameter_names

    def _update_sensor_parameter(self, parameter_name, value, success_label):
        ret = {
            'success': False,
            'message': ''
        }

        params = urllib_parse.urlencode({
            'action': 'update',
            'ImageSource.I0.Sensor.%s' % parameter_name: value
        })
        url = 'http://%s/axis-cgi/admin/param.cgi?%s' % (self.hostname, params)

        try:
            opener = self._get_digest_opener()
            response = opener.open(url, timeout=5)
            body = response.read().decode('utf-8').strip()

            if body == 'OK':
                ret['success'] = True
                ret['message'] = '%s set to %s' % (success_label, value)
            else:
                ret['message'] = 'camera rejected update: %s' % body

        except urllib_error.HTTPError as e:
            ret['message'] = 'HTTP error %d: %s' % (e.code, e.reason)
        except urllib_error.URLError as e:
            ret['message'] = 'connection error: %s' % e.reason
        except socket.timeout:
            ret['message'] = 'connection timeout'

        return ret

    def _normalize_parameter_value(self, value):
        if value is None:
            return None
        return str(value).strip()

    def _verify_and_rollback_parameter_update(self, parameter_path, requested_value, previous_value, result, success_label):
        current_value = self._normalize_parameter_value(self._read_parameter_value(parameter_path))
        requested_value = self._normalize_parameter_value(requested_value)
        previous_value = self._normalize_parameter_value(previous_value)

        # If the camera does not let us read back, keep original result.
        if current_value is None:
            return result

        # Requested value is exactly the one applied.
        if current_value == requested_value:
            return result

        # Value differs from requested one (firmware clamp or transform). Try rollback.
        if previous_value is not None and current_value != previous_value:
            rollback_result = self._update_parameter_path(parameter_path, previous_value, success_label)
            rollback_value = self._normalize_parameter_value(self._read_parameter_value(parameter_path))

            if rollback_result['success'] and rollback_value == previous_value:
                result['message'] = (
                    'camera applied %s=%s instead of requested %s; restored previous value %s'
                ) % (success_label, current_value, requested_value, previous_value)
            else:
                result['message'] = (
                    'camera applied %s=%s instead of requested %s and rollback failed'
                ) % (success_label, current_value, requested_value)
        else:
            result['message'] = 'camera applied %s=%s instead of requested %s' % (
                success_label,
                current_value,
                requested_value
            )

        result['success'] = False
        return result

    def _get_saturation_parameter_name(self):
        # First check the sensor parameter list
        try:
            parameter_names = self._list_sensor_parameters()
        except urllib_error.HTTPError as e:
            return None, 'HTTP error %d: %s' % (e.code, e.reason)
        except urllib_error.URLError as e:
            return None, 'connection error: %s' % e.reason
        except socket.timeout:
            return None, 'connection timeout'

        if 'Saturation' in parameter_names:
            return 'Saturation', ''
        if 'ColorLevel' in parameter_names:
            return 'ColorLevel', ''

        # If not in list, some firmwares still accept these parameters
        # Try Saturation first
        return 'Saturation', ''

    def _list_group_lines(self, group):
        opener = self._get_digest_opener()
        params = urllib_parse.urlencode({
            'action': 'list',
            'group': group
        })
        url = 'http://%s/axis-cgi/admin/param.cgi?%s' % (self.hostname, params)
        response = opener.open(url, timeout=5)
        return response.read().decode('utf-8').splitlines()

    def _group_exists(self, group):
        try:
            lines = self._list_group_lines(group)
        except Exception:
            return False

        if len(lines) == 0:
            # Some firmwares return an empty body for valid leaf groups.
            return True

        if lines[0].startswith('# Error:'):
            return False

        return True

    def _read_parameter_value(self, parameter_path):
        """Read a single parameter value from VAPIX. Returns the value or None if not found."""
        try:
            opener = self._get_digest_opener()
            params = urllib_parse.urlencode({
                'action': 'list',
                'group': parameter_path
            })
            url = 'http://%s/axis-cgi/admin/param.cgi?%s' % (self.hostname, params)
            response = opener.open(url, timeout=5)
            body = response.read().decode('utf-8').strip()
            
            if not body or body.startswith('# Error'):
                return None
            
            # If returns a single line with the parameter
            lines = body.splitlines()
            for line in lines:
                if '=' in line and not line.startswith('#'):
                    value = line.split('=', 1)[1].strip()
                    return value
            return None
        except Exception:
            return None

    def _ptz_brightness_to_service_range(self, ptz_brightness):
        """Convert PTZ brightness range (typically 1..9999) to normalized range [0, 100]."""
        min_val = 1
        max_val = 9999

        min_str = self._read_parameter_value('PTZ.Limit.L1.MinBrightness')
        max_str = self._read_parameter_value('PTZ.Limit.L1.MaxBrightness')
        try:
            if min_str is not None:
                min_val = int(min_str)
            if max_str is not None:
                max_val = int(max_str)
        except ValueError:
            pass

        if max_val <= min_val:
            return 0

        normalized = (float(ptz_brightness) - float(min_val)) / float(max_val - min_val)
        mapped = normalized * 100.0
        if mapped < 0.0:
            mapped = 0.0
        if mapped > 100.0:
            mapped = 100.0
        return int(round(mapped))

    def _get_white_balance_parameter_path(self):
        if self._white_balance_parameter_path is not None:
            return self._white_balance_parameter_path, ''

        try:
            sensor_names = self._list_sensor_parameters()
        except urllib_error.HTTPError as e:
            return None, 'HTTP error %d: %s' % (e.code, e.reason)
        except urllib_error.URLError as e:
            return None, 'connection error: %s' % e.reason
        except socket.timeout:
            return None, 'connection timeout'

        if 'WhiteBalance' in sensor_names or self._group_exists('ImageSource.I0.Sensor.WhiteBalance'):
            self._white_balance_parameter_path = 'ImageSource.I0.Sensor.WhiteBalance'
            return self._white_balance_parameter_path, ''

        if self._group_exists('Image.I0.Appearance.WhiteBalance'):
            self._white_balance_parameter_path = 'Image.I0.Appearance.WhiteBalance'
            return self._white_balance_parameter_path, ''

        try:
            appearance_lines = self._list_group_lines('Image.I0.Appearance')
        except Exception:
            appearance_lines = []

        for line in appearance_lines:
            if line.startswith('root.Image.I0.Appearance.WhiteBalance='):
                self._white_balance_parameter_path = 'Image.I0.Appearance.WhiteBalance'
                return self._white_balance_parameter_path, ''

        return None, 'camera does not expose a supported white balance parameter'

    def _get_day_night_parameter_path(self):
        if self._day_night_parameter_path is not None:
            return self._day_night_parameter_path, ''

        # Try the parameter specified in the VAPIX task first
        if self._group_exists('ImageSource.I0.DayNight.DayNightShift'):
            self._day_night_parameter_path = 'ImageSource.I0.DayNight.DayNightShift'
            return self._day_night_parameter_path, ''

        # Fall back to IrCutFilter (used on some firmware versions)
        if self._group_exists('ImageSource.I0.DayNight.IrCutFilter'):
            self._day_night_parameter_path = 'ImageSource.I0.DayNight.IrCutFilter'
            return self._day_night_parameter_path, ''

        return None, 'camera does not expose a supported day/night parameter'

    def _get_supported_white_balance_modes(self, parameter_path):
        if self._white_balance_supported_modes is not None:
            return self._white_balance_supported_modes, ''

        parts = parameter_path.rsplit('.', 1)
        if len(parts) == 2:
            group, parameter_name = parts
            modes = self._get_parameter_enum_values_from_definitions(group, parameter_name)
            if modes:
                self._white_balance_supported_modes = modes
                return self._white_balance_supported_modes, ''

        # Fallback if XML definitions are unavailable
        self._white_balance_supported_modes = set(['auto', 'fixed_indoor', 'fixed_outdoor', 'hold'])
        return self._white_balance_supported_modes, ''

    def _get_supported_day_night_modes(self, parameter_path):
        if self._day_night_supported_modes is not None:
            return self._day_night_supported_modes, ''

        parts = parameter_path.rsplit('.', 1)
        if len(parts) == 2:
            group, parameter_name = parts
            modes = self._get_parameter_enum_values_from_definitions(group, parameter_name)
            if modes:
                self._day_night_supported_modes = modes
                return self._day_night_supported_modes, ''

        # Fallback if XML definitions are unavailable
        if 'DayNightShift' in parameter_path:
            self._day_night_supported_modes = set(['auto', 'day', 'night'])
        else:  # IrCutFilter
            self._day_night_supported_modes = set(['auto', 'yes', 'no'])
        return self._day_night_supported_modes, ''

    def _supports_day_night_mode(self):
        try:
            property_lines = self._list_group_lines('Properties.ImageSource')
        except urllib_error.HTTPError as e:
            return None, 'HTTP error %d: %s' % (e.code, e.reason)
        except urllib_error.URLError as e:
            return None, 'connection error: %s' % e.reason
        except socket.timeout:
            return None, 'connection timeout'

        for line in property_lines:
            if line.startswith('root.Properties.ImageSource.DayNight='):
                value = line.split('=', 1)[1].strip().lower()
                return value == 'yes', ''

        return True, ''

    def _update_parameter_path(self, parameter_path, value, success_label):
        ret = {
            'success': False,
            'message': ''
        }

        params = urllib_parse.urlencode({
            'action': 'update',
            parameter_path: value
        })
        url = 'http://%s/axis-cgi/admin/param.cgi?%s' % (self.hostname, params)

        try:
            opener = self._get_digest_opener()
            response = opener.open(url, timeout=5)
            body = response.read().decode('utf-8').strip()

            if body == 'OK':
                ret['success'] = True
                ret['message'] = '%s set to %s' % (success_label, value)
            else:
                ret['message'] = 'camera rejected update: %s' % body

        except urllib_error.HTTPError as e:
            ret['message'] = 'HTTP error %d: %s' % (e.code, e.reason)
        except urllib_error.URLError as e:
            ret['message'] = 'connection error: %s' % e.reason
        except socket.timeout:
            ret['message'] = 'connection timeout'

        return ret

    def _update_sensor_parameter_verified(self, parameter_name, value, success_label):
        parameter_path = 'ImageSource.I0.Sensor.%s' % parameter_name
        previous_value = self._read_parameter_value(parameter_path)
        result = self._update_sensor_parameter(parameter_name, value, success_label)
        return self._verify_and_rollback_parameter_update(
            parameter_path,
            value,
            previous_value,
            result,
            success_label
        )

    def _update_parameter_path_verified(self, parameter_path, value, success_label):
        previous_value = self._read_parameter_value(parameter_path)
        result = self._update_parameter_path(parameter_path, value, success_label)
        return self._verify_and_rollback_parameter_update(
            parameter_path,
            value,
            previous_value,
            result,
            success_label
        )

    def getPTZLimits(self):
        """
            Gets the PTZ limits reported by the camera.
        """
        ptz_limits = {
            'focus_min': None,
            'focus_max': None,
            'iris_min': None,
            'iris_max': None,
            'error_reading': False,
            'error_reading_msg': ''
        }
        conn = httplib.HTTPConnection(self.hostname)
        params = {
            'query': 'limits',
            'camera': 1
        }

        try:
            conn.request("GET", "/axis-cgi/com/ptz.cgi?%s" % urllib_parse.urlencode(params))
            response = conn.getresponse()
            if response.status == 200:
                body = response.read()
                try:
                    parsed_params = dict([s.split('=', 1) for s in body.splitlines()])
                except:
                    parsed_params = dict([s.decode().split('=', 1) for s in body.splitlines()])

                if 'MinFocus' in parsed_params:
                    ptz_limits['focus_min'] = float(parsed_params['MinFocus'])
                if 'MaxFocus' in parsed_params:
                    ptz_limits['focus_max'] = float(parsed_params['MaxFocus'])
                if 'MinIris' in parsed_params:
                    ptz_limits['iris_min'] = float(parsed_params['MinIris'])
                if 'MaxIris' in parsed_params:
                    ptz_limits['iris_max'] = float(parsed_params['MaxIris'])
        except socket.error as e:
            ptz_limits['error_reading'] = True
            ptz_limits['error_reading_msg'] = e
        except socket.timeout as e:
            ptz_limits['error_reading'] = True
            ptz_limits['error_reading_msg'] = e
        except ValueError as e:
            ptz_limits['error_reading'] = True
            ptz_limits['error_reading_msg'] = e
        finally:
            conn.close()

        return ptz_limits

    def sendPTZCommand(self, pan=None, tilt=None, zoom=None, focus=None, autofocus=None, iris=None, autoiris=None):
        ret = {
            'exception': False,
            'error_msg': '',
            'status': 0,
            'body': '',
            'url': ''
        }

        conn = httplib.HTTPConnection(self.hostname)
        params = {}
        if pan is not None:
            params['pan'] = pan
        if tilt is not None:
            params['tilt'] = tilt
        if zoom is not None:
            params['zoom'] = zoom
        if focus is not None:
            params['focus'] = focus
        if autofocus is not None:
            params['autofocus'] = 'on' if autofocus else 'off'
        if iris is not None:
            params['iris'] = iris
        if autoiris is not None:
            params['autoiris'] = 'on' if autoiris else 'off'

        try:		   
            url = "/axis-cgi/com/ptz.cgi?camera=1&%s" % urllib_parse.urlencode(params)

            conn.request("GET", url)
            response = conn.getresponse()
            ret['status'] = response.status
            body = response.read()
            if body:
                try:
                    ret['body'] = body.decode()
                except AttributeError:
                    ret['body'] = body
            ret['url'] = url

        except socket.error as e:
            ret['exception'] = True
            ret['error_msg'] = e
        except socket.timeout as e:
            ret['exception'] = True
            ret['error_msg'] = e
        finally:
            conn.close()
        return ret

    def _try_autotracking_vapix(self, opener, enable):
        """
        Tries to set autotracking via official VAPIX PTZ Autotracking API.
        Available on cameras with firmware >= 10.x that expose the CGI natively.
        Returns (success, message) tuple. success=None means 404 => try fallback.
        """
        url = 'http://%s/axis-cgi/ptz-autotracking/operator.cgi' % self.hostname
        payload = {
            'apiVersion': '1.0',
            'method': 'setAutotrackingState',
            'params': {'enabled': bool(enable)}
        }
        try:
            request = urllib_request.Request(
                url,
                data=json.dumps(payload).encode('utf-8'),
                headers={'Content-Type': 'application/json'}
            )
            response = opener.open(request, timeout=5)
            body = response.read().decode('utf-8', errors='replace').strip()
            if response.getcode() in (200, 204):
                if body:
                    try:
                        body_json = json.loads(body)
                    except Exception:
                        body_json = None
                    if isinstance(body_json, dict) and isinstance(body_json.get('error'), dict):
                        return False, 'VAPIX operator.cgi error: %s' % str(body_json['error'].get('code'))
                return True, 'Auto-tracking %s via VAPIX operator.cgi' % ('enabled' if enable else 'disabled')
            return False, 'VAPIX operator.cgi HTTP %d' % response.getcode()
        except urllib_error.HTTPError as e:
            if e.code == 404:
                return None, 'VAPIX operator.cgi not found (404)'  # None = try fallback
            return False, 'VAPIX operator.cgi HTTP error %d: %s' % (e.code, e.reason)
        except (urllib_error.URLError, socket.timeout) as e:
            return False, 'VAPIX operator.cgi connection error: %s' % str(e)

    def _try_autotracking_admin(self, opener, enable):
        """
        Tries to set autotracking via VAPIX PTZ Autotracking admin endpoint.
        Returns (success, message) tuple. success=None means 404 => try fallback.
        """
        url = 'http://%s/axis-cgi/ptz-autotracking/admin.cgi' % self.hostname
        payload = {
            'apiVersion': '1.0',
            'method': 'setAutotrackingState',
            'params': {'enabled': bool(enable)}
        }
        try:
            request = urllib_request.Request(
                url,
                data=json.dumps(payload).encode('utf-8'),
                headers={'Content-Type': 'application/json', 'Accept': 'application/json'}
            )
            response = opener.open(request, timeout=5)
            body = response.read().decode('utf-8', errors='replace').strip()
            if response.getcode() in (200, 204):
                if body:
                    try:
                        body_json = json.loads(body)
                    except Exception:
                        body_json = None
                    if isinstance(body_json, dict) and isinstance(body_json.get('error'), dict):
                        return False, 'VAPIX admin.cgi error: %s' % str(body_json['error'].get('code'))
                return True, 'Auto-tracking %s via VAPIX admin.cgi' % ('enabled' if enable else 'disabled')
            return False, 'VAPIX admin.cgi HTTP %d' % response.getcode()
        except urllib_error.HTTPError as e:
            if e.code == 404:
                return None, 'VAPIX admin.cgi not found (404)'
            return False, 'VAPIX admin.cgi HTTP error %d: %s' % (e.code, e.reason)
        except (urllib_error.URLError, socket.timeout) as e:
            return False, 'VAPIX admin.cgi connection error: %s' % str(e)

    def _try_autotracking_acap(self, opener, enable):
        """
        Tries to set autotracking via the PTZ Autotracker ACAP app local endpoint.
        This is the API the app exposes regardless of firmware version, and is the
        same endpoint used by the camera web UI. Works on cameras like P5676-LE.
        Returns (success, message) tuple.
        """
        url = 'http://%s/local/axis-ptz-autotracking/settings.fcgi' % self.hostname
        payload = {
            'apiVersion': '1.0',
            'method': 'setAutotrackingState',
            'params': {'enabled': bool(enable)}
        }
        try:
            request = urllib_request.Request(
                url,
                data=json.dumps(payload).encode('utf-8'),
                headers={'Content-Type': 'application/json'}
            )
            response = opener.open(request, timeout=5)
            body = response.read().decode('utf-8', errors='replace').strip()
            if response.getcode() in (200, 204):
                if body:
                    try:
                        body_json = json.loads(body)
                    except Exception:
                        body_json = None
                    if isinstance(body_json, dict) and isinstance(body_json.get('error'), dict):
                        return False, 'ACAP settings.fcgi error: %s' % str(body_json['error'].get('code'))
                return True, 'Auto-tracking %s via ACAP settings.fcgi' % ('enabled' if enable else 'disabled')
            return False, 'ACAP settings.fcgi HTTP %d' % response.getcode()
        except urllib_error.HTTPError as e:
            if e.code == 404:
                return False, 'ACAP settings.fcgi not found (404) - PTZ Autotracker app may not be installed'
            return False, 'ACAP settings.fcgi HTTP error %d: %s' % (e.code, e.reason)
        except (urllib_error.URLError, socket.timeout) as e:
            return False, 'ACAP settings.fcgi connection error: %s' % str(e)

    def setAutoTracking(self, enable):
        """
        Enables or disables auto-tracking on Axis PTZ cameras.

          Uses a three-tier approach to support a wide range of camera models:
             1. VAPIX PTZ Autotracking admin API (/axis-cgi/ptz-autotracking/admin.cgi)
                 - Newer firmware endpoint.
             2. VAPIX PTZ Autotracking operator API (/axis-cgi/ptz-autotracking/operator.cgi)
                 - Legacy endpoint.
             3. PTZ Autotracker ACAP app local API (/local/axis-ptz-autotracking/settings.fcgi)
             - The app's own REST endpoint, used by the web UI, available on any
               camera with the PTZ Autotracker ACAP app installed (e.g. P5676-LE)

        Returns a dict with keys: success (bool), message (str).
        """
        ret = {'success': False, 'message': ''}
        opener = self._get_digest_opener()

        # 1. Try VAPIX admin endpoint first
        success, message = self._try_autotracking_admin(opener, enable)
        if success is True:
            ret['success'] = True
            ret['message'] = message
            return ret
        if success is False:
            ret['message'] = message
            return ret

        # 2. Try legacy VAPIX endpoint
        success, message = self._try_autotracking_vapix(opener, enable)
        if success is True:
            ret['success'] = True
            ret['message'] = message
            return ret
        if success is False:
            ret['message'] = message
            return ret

        # success is None => 404, fall through to ACAP app endpoint
        # 3. Fall back to ACAP app local endpoint
        success, message = self._try_autotracking_acap(opener, enable)
        ret['success'] = success
        ret['message'] = message
        return ret

    def _extract_autotracking_enabled(self, payload):
        """
        Extracts autotracking enabled state from different JSON response shapes.
        Returns True/False when found, otherwise None.
        """
        if isinstance(payload, bool):
            return payload

        if not isinstance(payload, dict):
            return None

        # Fast path for common direct keys
        for key in ('enabled', 'active', 'autotracking', 'autotrack'):
            value = payload.get(key)
            if isinstance(value, bool):
                return value

        # Recursive walk for nested structures (data/result/params/etc.)
        stack = [payload]
        while stack:
            current = stack.pop()
            if not isinstance(current, dict):
                continue

            for key, value in current.items():
                if key in ('enabled', 'active', 'autotracking', 'autotrack'):
                    if isinstance(value, bool):
                        return value
                    if isinstance(value, str):
                        lowered = value.strip().lower()
                        if lowered in ('1', 'true', 'on', 'enabled', 'yes'):
                            return True
                        if lowered in ('0', 'false', 'off', 'disabled', 'no'):
                            return False
                if isinstance(value, dict):
                    stack.append(value)

        return None

    def _read_autotracking_state_endpoint(self, opener, url):
        """
        Reads autotracking state from one endpoint trying common getter methods.
        Returns (success, enabled, message).
        """
        methods = ('getAutotrackingState', 'getAutotrackerState', 'getState')
        for method in methods:
            payload = {
                'apiVersion': '1.0',
                'method': method,
                'params': {}
            }
            try:
                request = urllib_request.Request(
                    url,
                    data=json.dumps(payload).encode('utf-8'),
                    headers={'Content-Type': 'application/json'}
                )
                response = opener.open(request, timeout=5)
                body = response.read().decode('utf-8', errors='replace').strip()
                if response.getcode() not in (200, 204):
                    continue
                if not body:
                    continue

                try:
                    body_json = json.loads(body)
                except Exception:
                    continue

                if isinstance(body_json, dict) and isinstance(body_json.get('error'), dict):
                    continue

                enabled = self._extract_autotracking_enabled(body_json)
                if enabled is not None:
                    return True, enabled, 'read via %s' % url
            except urllib_error.HTTPError as e:
                if e.code == 404:
                    return False, None, '%s not found (404)' % url
            except (urllib_error.URLError, socket.timeout) as e:
                return False, None, 'connection error on %s: %s' % (url, str(e))

        return False, None, 'no readable state on %s' % url

    def getAutoTrackingState(self):
        """
        Reads current autotracking state from camera APIs.
        Returns a dict: success (bool), enabled (bool or None), message (str).
        """
        opener = self._get_digest_opener()
        urls = (
            'http://%s/axis-cgi/ptz-autotracking/admin.cgi' % self.hostname,
            'http://%s/axis-cgi/ptz-autotracking/operator.cgi' % self.hostname,
            'http://%s/local/axis-ptz-autotracking/settings.fcgi' % self.hostname,
        )

        errors = []
        for url in urls:
            success, enabled, message = self._read_autotracking_state_endpoint(opener, url)
            if success:
                return {'success': True, 'enabled': enabled, 'message': message}
            errors.append(message)

        return {
            'success': False,
            'enabled': None,
            'message': '; '.join(errors)
        }

    _AOA_CONTROL_PATH = '/local/objectanalytics/control.cgi'
    _AOA_MODE_TO_CLASSES = {
        # Confirmed from AOA UI payloads:
        # motion  -> objectClassifications omitted
        # person  -> [{'type': 'human'}]
        # vehicle -> [{'type': 'vehicle'}]
        'motion': tuple(),
        'person': ('human',),
        'vehicle': ('vehicle',),
    }

    def _normalize_tracking_mode(self, mode):
        if mode is None:
            return 'auto'
        lowered = str(mode).strip().lower()
        alias = {
            'human': 'person',
            'people': 'person',
            'car': 'vehicle',
            'movement': 'motion',
        }
        return alias.get(lowered, lowered)

    def _call_aoa(self, opener, method, params):
        url = 'http://%s%s' % (self.hostname, self._AOA_CONTROL_PATH)
        payload = {
            'apiVersion': '1.3',
            'context': 'AOA_NATIVE_UI',
            'method': method,
            'params': params,
        }
        request = urllib_request.Request(
            url,
            data=json.dumps(payload).encode('utf-8'),
            headers={
                'Content-Type': 'application/json',
                'Accept': 'application/json',
                'X-Requested-With': 'XMLHttpRequest',
            }
        )
        response = opener.open(request, timeout=5)
        body = response.read().decode('utf-8', errors='replace').strip()
        return response.getcode(), json.loads(body) if body else {}

    def _get_aoa_configuration(self, opener):
        try:
            status, body = self._call_aoa(opener, 'getConfiguration', {})
            if status not in (200, 204):
                return None, 'AOA getConfiguration HTTP %d' % status
            if isinstance(body.get('error'), dict):
                return None, 'AOA getConfiguration error: %s' % body['error'].get('message', body['error'])
            data = body.get('data')
            if not isinstance(data, dict):
                return None, 'AOA getConfiguration returned no data'
            return data, 'AOA configuration read'
        except urllib_error.HTTPError as e:
            if e.code == 404:
                return None, 'AOA control.cgi not found (404)'
            return None, 'AOA getConfiguration HTTP error %d: %s' % (e.code, e.reason)
        except (urllib_error.URLError, socket.timeout) as e:
            return None, 'AOA getConfiguration connection error: %s' % str(e)
        except Exception as e:
            return None, 'AOA getConfiguration parse error: %s' % str(e)

    def _get_aoa_configuration_capabilities(self, opener):
        try:
            status, body = self._call_aoa(opener, 'getConfigurationCapabilities', {})
            if status not in (200, 204):
                return None, 'AOA getConfigurationCapabilities HTTP %d' % status
            if isinstance(body.get('error'), dict):
                return None, 'AOA getConfigurationCapabilities error: %s' % body['error'].get('message', body['error'])
            data = body.get('data')
            if not isinstance(data, dict):
                return None, 'AOA getConfigurationCapabilities returned no data'
            return data, 'AOA configuration capabilities read'
        except urllib_error.HTTPError as e:
            if e.code == 404:
                return None, 'AOA control.cgi not found (404)'
            return None, 'AOA getConfigurationCapabilities HTTP error %d: %s' % (e.code, e.reason)
        except (urllib_error.URLError, socket.timeout) as e:
            return None, 'AOA getConfigurationCapabilities connection error: %s' % str(e)
        except Exception as e:
            return None, 'AOA getConfigurationCapabilities parse error: %s' % str(e)

    def _extract_supported_modes_from_aoa_capabilities(self, capabilities):
        supported_modes = set(['motion'])

        scenarios = capabilities.get('scenarios', {}) if isinstance(capabilities, dict) else {}
        supported_scenarios = scenarios.get('supportedScenarios', []) if isinstance(scenarios, dict) else []
        if isinstance(supported_scenarios, list) and 'motion' not in supported_scenarios:
            supported_modes.discard('motion')

        def walk_classifications(items):
            if not isinstance(items, list):
                return
            for item in items:
                if not isinstance(item, dict):
                    continue
                item_type = str(item.get('type', '')).strip().lower()
                if item_type == 'human':
                    supported_modes.add('person')
                elif item_type == 'vehicle':
                    supported_modes.add('vehicle')
                walk_classifications(item.get('subTypes', []))

        walk_classifications(capabilities.get('objectClassifications', []))

        if not supported_modes:
            supported_modes.add('motion')

        ordered = [mode for mode in ('motion', 'person', 'vehicle') if mode in supported_modes]
        return ordered if ordered else ['motion']

    def _mode_from_aoa_configuration(self, config):
        scenarios = config.get('scenarios', []) if isinstance(config, dict) else []
        if not scenarios:
            return 'motion'

        saw_motion_pattern = False
        classes = set()
        for scenario in scenarios:
            if not isinstance(scenario, dict):
                continue
            object_classifications = scenario.get('objectClassifications')
            if object_classifications is None:
                saw_motion_pattern = True
                continue
            if not isinstance(object_classifications, list) or len(object_classifications) == 0:
                saw_motion_pattern = True
                continue
            for item in object_classifications:
                if not isinstance(item, dict):
                    continue
                t = str(item.get('type', '')).strip().lower()
                if t == 'human':
                    classes.add('person')
                elif t == 'vehicle':
                    classes.add('vehicle')

        if saw_motion_pattern:
            return 'motion'
        if classes == {'person'}:
            return 'person'
        if classes == {'vehicle'}:
            return 'vehicle'
        return 'motion'

    def _prepare_aoa_configuration_for_mode(self, config, mode):
        updated = json.loads(json.dumps(config))
        scenarios = updated.get('scenarios', [])
        if not isinstance(scenarios, list):
            return updated

        if mode == 'motion':
            for scenario in scenarios:
                if isinstance(scenario, dict) and 'objectClassifications' in scenario:
                    del scenario['objectClassifications']
            return updated

        desired = [{'type': t} for t in self._AOA_MODE_TO_CLASSES[mode]]
        for scenario in scenarios:
            if not isinstance(scenario, dict):
                continue
            scenario['objectClassifications'] = list(desired)
        return updated

    def _set_aoa_mode(self, opener, mode):
        config, message = self._get_aoa_configuration(opener)
        if config is None:
            if 'not found (404)' in message:
                return None, message
            return False, message

        new_config = self._prepare_aoa_configuration_for_mode(config, mode)
        try:
            status, body = self._call_aoa(opener, 'setConfiguration', new_config)
            if status not in (200, 204):
                return False, 'AOA setConfiguration HTTP %d' % status
            if isinstance(body.get('error'), dict):
                return False, 'AOA setConfiguration error: %s' % body['error'].get('message', body['error'])

            verify_config, verify_message = self._get_aoa_configuration(opener)
            if verify_config is None:
                return False, 'AOA setConfiguration accepted but verify failed: %s' % verify_message
            effective_mode = self._mode_from_aoa_configuration(verify_config)
            if effective_mode != mode:
                return False, 'AOA setConfiguration accepted but effective mode is %s (requested %s)' % (effective_mode, mode)
            return True, 'mode %s applied via AOA control.cgi' % mode
        except urllib_error.HTTPError as e:
            if e.code == 404:
                return None, 'AOA control.cgi not found (404)'
            return False, 'AOA setConfiguration HTTP error %d: %s' % (e.code, e.reason)
        except (urllib_error.URLError, socket.timeout) as e:
            return False, 'AOA setConfiguration connection error: %s' % str(e)

    def setAutoTrackingWithMode(self, enabled, mode='auto'):
        """
        Enables/disables autotracking and applies tracking mode when requested.
        Supported modes: motion | person | vehicle | auto
        """
        normalized_mode = self._normalize_tracking_mode(mode)
        if normalized_mode not in ('auto', 'motion', 'person', 'vehicle'):
            return {
                'success': False,
                'applied_mode': 'motion',
                'fallback_applied': False,
                'message': 'unsupported mode "%s"; expected motion|person|vehicle|auto' % str(mode)
            }

        toggle_result = self.setAutoTracking(enabled)
        if not toggle_result.get('success'):
            return {
                'success': False,
                'applied_mode': 'motion',
                'fallback_applied': False,
                'message': toggle_result.get('message', 'failed to toggle autotracking')
            }

        opener = self._get_digest_opener()
        capabilities = self.getAutoTrackingCapabilities()
        supported_modes = capabilities.get('supported_modes', ['motion'])

        if normalized_mode == 'auto':
            current_mode = 'motion'
            config, _ = self._get_aoa_configuration(opener)
            if config is not None:
                current_mode = self._mode_from_aoa_configuration(config)
            return {
                'success': True,
                'applied_mode': current_mode,
                'fallback_applied': False,
                'message': '%s; mode kept as auto' % toggle_result['message']
            }

        if normalized_mode not in supported_modes:
            if 'motion' in supported_modes:
                return {
                    'success': True,
                    'applied_mode': 'motion',
                    'fallback_applied': True,
                    'message': '%s; mode "%s" not supported by this device, fallback to motion'
                               % (toggle_result['message'], normalized_mode)
                }
            return {
                'success': False,
                'applied_mode': 'motion',
                'fallback_applied': False,
                'message': '%s; mode "%s" not supported and motion fallback unavailable'
                           % (toggle_result['message'], normalized_mode)
            }

        mode_success, mode_message = self._set_aoa_mode(opener, normalized_mode)
        if normalized_mode == 'motion' and mode_success is None:
            return {
                'success': True,
                'applied_mode': 'motion',
                'fallback_applied': False,
                'message': '%s; motion mode assumed because AOA is not available' % toggle_result['message']
            }
        if mode_success is True:
            return {
                'success': True,
                'applied_mode': normalized_mode,
                'fallback_applied': False,
                'message': '%s; %s' % (toggle_result['message'], mode_message)
            }

        if normalized_mode != 'motion':
            fallback_success, fallback_message = self._set_aoa_mode(opener, 'motion')
            if fallback_success is True or fallback_success is None:
                return {
                    'success': True,
                    'applied_mode': 'motion',
                    'fallback_applied': True,
                    'message': '%s; mode "%s" not supported (%s), fallback to motion (%s)'
                               % (toggle_result['message'], normalized_mode, mode_message, fallback_message)
                }

        return {
            'success': False,
            'applied_mode': 'motion',
            'fallback_applied': False,
            'message': '%s; failed to apply mode "%s": %s' % (toggle_result['message'], normalized_mode, mode_message)
        }

    def getAutoTrackingCapabilities(self):
        """
        Returns supported modes and current autotracking status.
        """
        opener = self._get_digest_opener()

        supported_modes = ['motion']
        current_mode = 'motion'

        capabilities, _ = self._get_aoa_configuration_capabilities(opener)
        config, _ = self._get_aoa_configuration(opener)
        if capabilities is not None:
            supported_modes = self._extract_supported_modes_from_aoa_capabilities(capabilities)
        if config is not None:
            current_mode = self._mode_from_aoa_configuration(config)

        state = self.getAutoTrackingState()
        enabled = bool(state.get('enabled')) if state.get('enabled') is not None else False

        return {
            'success': True,
            'supported_modes': supported_modes,
            'current_mode': current_mode,
            'enabled': enabled,
            'message': state.get('message', '')
        }

    def getPTZState(self):
        """
            Gets the current ptz state/position of the camera
        """
        ptz_read = {}
        conn = httplib.HTTPConnection(self.hostname)
        params = { 'query':'position' }
        try:
            conn.request("GET", "/axis-cgi/com/ptz.cgi?%s" % urllib_parse.urlencode(params))
            response = conn.getresponse()
            if response.status == 200:
                body = response.read()
                try:
                    params = dict([s.split('=',2) for s in body.splitlines()])
                except:
                    params = dict([s.decode().split('=',2) for s in body.splitlines()])
                pan = math.radians(float(params['pan']))
                tilt = math.radians(float(params['tilt']))
                
                if 'zoom' in params:
                    zoom = float(params['zoom'])
                else:
                    zoom = 0.0
                # Optional params (depending on model)
                if 'iris' in params:
                    iris = float(params['iris'])
                else:
                    iris = 0.0
                if 'focus' in params:
                    focus = float(params['focus'])
                else:
                    focus = 0.0
                if 'autofocus' in params:
                    autofocus = (params['autofocus'] == 'on')
                else:
                    autofocus = False
                if 'autoiris' in params:
                    autoiris = (params['autoiris'] == 'on')
                else:
                    autoiris = False

                ptz_read = {
                    "pan" : pan,
                    "tilt" : tilt,
                    "zoom" : zoom,
                    "focus" : focus,
                    "autofocus" : autofocus,
                    "iris" : iris,
                    "autoiris" : autoiris,
                    "supports_focus" : ('focus' in params or 'autofocus' in params),
                    "supports_iris" : ('iris' in params or 'autoiris' in params),
                    "error_reading" : False,
                    "error_reading_msg" : ''
                }    
                        
        except socket.error as e:
            ptz_read["error_reading"]= True
            ptz_read["error_reading_msg"] = e
        except socket.timeout as e:
            ptz_read["error_reading"]= True
            ptz_read["error_reading_msg"] = e
        except ValueError as e:
            ptz_read["error_reading"]= True
            ptz_read["error_reading_msg"] = e
        except KeyError as e:
            ptz_read["error_reading"]= True
            ptz_read["error_reading_msg"] = "Missing expected field in PTZ response: %s" % e
        finally:
            conn.close()
        
        return ptz_read

    def setBrightness(self, brightness):
        br_range = self._get_parameter_int_range_from_definitions('ImageSource.I0.Sensor', 'Brightness')
        min_br, max_br = br_range if br_range[0] is not None else (-100, 100)
        if brightness < min_br or brightness > max_br:
            return {
                'success': False,
                'message': 'brightness value %d is out of range [%d, %d]' % (brightness, min_br, max_br)
            }

        result = self._update_sensor_parameter_verified('Brightness', brightness, 'brightness')
        if result['success']:
            self._last_known_image_settings['brightness'] = brightness
        return result

    def setContrast(self, contrast):
        ct_range = self._get_parameter_int_range_from_definitions('ImageSource.I0.Sensor', 'Contrast')
        min_ct, max_ct = ct_range if ct_range[0] is not None else (-100, 100)
        if contrast < min_ct or contrast > max_ct:
            return {
                'success': False,
                'message': 'contrast value %d is out of range [%d, %d]' % (contrast, min_ct, max_ct)
            }

        result = self._update_sensor_parameter_verified('Contrast', contrast, 'contrast')
        if result['success']:
            self._last_known_image_settings['contrast'] = contrast
        return result

    def setSaturation(self, saturation):
        parameter_name, error_message = self._get_saturation_parameter_name()
        if parameter_name is None:
            return {
                'success': False,
                'message': error_message
            }

        # Known VAPIX ranges per parameter name as fallback when listdefinitions is unavailable.
        _KNOWN_SATURATION_RANGES = {
            'Saturation': (-100, 100),
            'ColorLevel': (0, 100),
        }
        sat_range = self._get_parameter_int_range_from_definitions('ImageSource.I0.Sensor', parameter_name)
        if sat_range[0] is not None:
            min_sat, max_sat = sat_range
        else:
            min_sat, max_sat = _KNOWN_SATURATION_RANGES.get(parameter_name, (-100, 100))

        if saturation < min_sat or saturation > max_sat:
            return {
                'success': False,
                'message': 'saturation value %d is out of range [%d, %d]' % (saturation, min_sat, max_sat)
            }

        # Try the detected/default parameter name
        result = self._update_sensor_parameter_verified(parameter_name, saturation, 'saturation')
        if result['success']:
            self._last_known_image_settings['saturation'] = saturation
            return result

        # If Saturation failed, try ColorLevel as fallback — but validate its range first.
        if parameter_name == 'Saturation':
            cl_range = self._get_parameter_int_range_from_definitions('ImageSource.I0.Sensor', 'ColorLevel')
            cl_min = cl_range[0] if cl_range[0] is not None else 0
            cl_max = cl_range[1] if cl_range[1] is not None else 100
            if cl_min <= saturation <= cl_max:
                result = self._update_sensor_parameter_verified('ColorLevel', saturation, 'saturation')
                if result['success']:
                    self._last_known_image_settings['saturation'] = saturation
                    return result
            else:
                return {
                    'success': False,
                    'message': 'saturation value %d is out of range [%d, %d] for ColorLevel' % (saturation, cl_min, cl_max)
                }

        return result

    def getWhiteBalanceModes(self):
        parameter_path, error_message = self._get_white_balance_parameter_path()
        if parameter_path is None:
            return {
                'success': False,
                'message': error_message,
                'modes': []
            }

        enum_modes = None
        parts = parameter_path.rsplit('.', 1)
        if len(parts) == 2:
            enum_modes = self._get_parameter_enum_values_from_definitions(parts[0], parts[1])

        supported_modes, error_message = self._get_supported_white_balance_modes(parameter_path)
        if supported_modes is None:
            return {
                'success': False,
                'message': error_message,
                'modes': []
            }

        modes = sorted(enum_modes) if enum_modes else sorted(supported_modes)
        return {
            'success': True,
            'message': 'white_balance modes retrieved',
            'modes': modes
        }

    def setWhiteBalance(self, white_balance):
        white_balance = white_balance.strip()
        if not white_balance:
            return {
                'success': False,
                'message': 'white_balance mode cannot be empty'
            }

        parameter_path, error_message = self._get_white_balance_parameter_path()
        if parameter_path is None:
            return {
                'success': False,
                'message': error_message
            }

        enum_modes = None
        parts = parameter_path.rsplit('.', 1)
        if len(parts) == 2:
            enum_modes = self._get_parameter_enum_values_from_definitions(parts[0], parts[1])

        # If camera exposes enum capabilities, validate strictly against those values.
        if enum_modes:
            if white_balance not in enum_modes:
                modes_text = ', '.join(sorted(enum_modes))
                return {
                    'success': False,
                    'message': 'unsupported white_balance mode "%s". Supported modes: %s' % (white_balance, modes_text)
                }

        supported_modes, error_message = self._get_supported_white_balance_modes(parameter_path)
        if supported_modes is None:
            return {
                'success': False,
                'message': error_message
            }

        # If enum capabilities are unavailable, supported_modes may come from fallback.
        # In that case, allow write-through and let camera return the authoritative result.
        if enum_modes and white_balance not in supported_modes:
            modes_text = ', '.join(sorted(supported_modes))
            return {
                'success': False,
                'message': 'unsupported white_balance mode "%s". Supported modes: %s' % (white_balance, modes_text)
            }

        result = self._update_parameter_path(parameter_path, white_balance, 'white_balance')
        if result['success']:
            self._last_known_image_settings['white_balance'] = white_balance
        return result

    def setDayNightMode(self, day_night_mode):
        normalized_mode = day_night_mode.strip().lower()

        supports_day_night, error_message = self._supports_day_night_mode()
        if supports_day_night is None:
            return {
                'success': False,
                'message': error_message
            }
        if not supports_day_night:
            return {
                'success': False,
                'message': 'camera does not support manual day/night mode control'
            }

        parameter_path, error_message = self._get_day_night_parameter_path()
        if parameter_path is None:
            return {
                'success': False,
                'message': error_message
            }

        supported_modes, error_message = self._get_supported_day_night_modes(parameter_path)
        if supported_modes is None:
            return {
                'success': False,
                'message': error_message
            }

        # Map user-friendly aliases based on parameter type
        if 'DayNightShift' in parameter_path:
            # DayNightShift: day/night → day/night (no mapping needed)
            vapix_mode = normalized_mode
            friendly_modes = 'auto, day, night'
        else:
            # IrCutFilter: day/night → yes/no
            MODE_ALIAS = {'day': 'yes', 'night': 'no'}
            vapix_mode = MODE_ALIAS.get(normalized_mode, normalized_mode)
            friendly_modes = 'auto, day (yes), night (no)'

        if vapix_mode not in supported_modes:
            return {
                'success': False,
                'message': 'unsupported day_night_mode "%s". Supported modes: %s' % (day_night_mode, friendly_modes)
            }

        result = self._update_parameter_path(parameter_path, vapix_mode, 'day_night_mode')
        if result['success']:
            if 'DayNightShift' in parameter_path:
                if vapix_mode == 'night':
                    self._last_known_image_settings['is_night_mode_active'] = True
                elif vapix_mode == 'day':
                    self._last_known_image_settings['is_night_mode_active'] = False
            else:
                if vapix_mode == 'no':
                    self._last_known_image_settings['is_night_mode_active'] = True
                elif vapix_mode == 'yes':
                    self._last_known_image_settings['is_night_mode_active'] = False
        return result

    def setDayNightShiftLevel(self, shift_level):
        shift_range = self._get_parameter_int_range_from_definitions('ImageSource.I0.DayNight', 'ShiftLevel')
        if shift_range[0] is not None:
            min_shift, max_shift = shift_range
        else:
            min_shift, max_shift = 0, 100

        if shift_level < min_shift or shift_level > max_shift:
            return {
                'success': False,
                'message': 'shift_level value %d is out of range [%d, %d]' % (shift_level, min_shift, max_shift)
            }

        result = self._update_parameter_path_verified('ImageSource.I0.DayNight.ShiftLevel', shift_level, 'day_night_shift_level')
        if result['success']:
            self._last_known_image_settings['day_night_shift_level'] = shift_level
        return result

    def getImageSettings(self):
        ret = {
            'success': False,
            'brightness': 0,
            'contrast': 0,
            'saturation': 0,
            'white_balance': '',
            'is_night_mode_active': False,
            'day_night_shift_level': 0,
            'message': ''
        }
        readable_fields = 0
        cached_fields = 0

        # Read brightness, contrast directly from parameters
        brightness_val = self._read_parameter_value('ImageSource.I0.Sensor.Brightness')
        if brightness_val:
            try:
                ret['brightness'] = int(brightness_val)
                readable_fields += 1
            except ValueError:
                pass
        elif self._last_known_image_settings['brightness'] is not None:
            ret['brightness'] = self._last_known_image_settings['brightness']
            cached_fields += 1

        # Fallback: some cameras expose live brightness via PTZ position only.
        if not brightness_val and self._last_known_image_settings['brightness'] is None:
            ptz_state = self.getPTZState()
            if (not ptz_state.get('error_reading', True)) and (ptz_state.get('brightness') is not None):
                ret['brightness'] = self._ptz_brightness_to_service_range(ptz_state['brightness'])
                readable_fields += 1

        contrast_val = self._read_parameter_value('ImageSource.I0.Sensor.Contrast')
        if contrast_val:
            try:
                ret['contrast'] = int(contrast_val)
                readable_fields += 1
            except ValueError:
                pass
        elif self._last_known_image_settings['contrast'] is not None:
            ret['contrast'] = self._last_known_image_settings['contrast']
            cached_fields += 1

        # Try saturation (try both Saturation and ColorLevel)
        saturation_val = self._read_parameter_value('ImageSource.I0.Sensor.Saturation')
        if not saturation_val:
            saturation_val = self._read_parameter_value('ImageSource.I0.Sensor.ColorLevel')
        if saturation_val:
            try:
                ret['saturation'] = int(saturation_val)
                readable_fields += 1
            except ValueError:
                pass
        elif self._last_known_image_settings['saturation'] is not None:
            ret['saturation'] = self._last_known_image_settings['saturation']
            cached_fields += 1

        # Read white balance
        white_balance_path, _ = self._get_white_balance_parameter_path()
        if white_balance_path:
            white_balance_val = self._read_parameter_value(white_balance_path)
            if white_balance_val:
                ret['white_balance'] = white_balance_val
                readable_fields += 1
            elif self._last_known_image_settings['white_balance'] is not None:
                ret['white_balance'] = self._last_known_image_settings['white_balance']
                cached_fields += 1

        # Read day/night parameters
        ir_cut_val = self._read_parameter_value('ImageSource.I0.DayNight.IrCutFilter')
        day_night_shift_val = self._read_parameter_value('ImageSource.I0.DayNight.DayNightShift')
        
        if ir_cut_val:
            ret['is_night_mode_active'] = (ir_cut_val.lower() == 'no')
            readable_fields += 1
        elif day_night_shift_val:
            ret['is_night_mode_active'] = (day_night_shift_val.lower() == 'night')
            readable_fields += 1
        elif self._last_known_image_settings['is_night_mode_active'] is not None:
            ret['is_night_mode_active'] = self._last_known_image_settings['is_night_mode_active']
            cached_fields += 1

        # Read shift level
        shift_level_val = self._read_parameter_value('ImageSource.I0.DayNight.ShiftLevel')
        if shift_level_val:
            try:
                ret['day_night_shift_level'] = int(shift_level_val)
                readable_fields += 1
            except ValueError:
                pass
        elif self._last_known_image_settings['day_night_shift_level'] is not None:
            ret['day_night_shift_level'] = self._last_known_image_settings['day_night_shift_level']
            cached_fields += 1

        if readable_fields == 6:
            ret['success'] = True
            ret['message'] = 'ok'
        elif readable_fields > 0 or cached_fields > 0:
            ret['success'] = False
            ret['message'] = (
                'read failed for %d/6 fields; publishing fallback values '
                '(readable=%d, cached=%d)'
            ) % (6 - readable_fields, readable_fields, cached_fields)
        else:
            ret['success'] = False
            ret['message'] = 'camera does not expose readable image settings on this firmware'
        return ret
