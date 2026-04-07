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

    def _extract_modes_from_value(self, value_text):
        if ',' in value_text:
            return [item.strip() for item in value_text.split(',') if item.strip()]
        return []

    def _get_supported_white_balance_modes(self, parameter_path):
        if self._white_balance_supported_modes is not None:
            return self._white_balance_supported_modes, ''

        modes = set()

        try:
            lines = self._list_group_lines(parameter_path)
        except urllib_error.HTTPError as e:
            return None, 'HTTP error %d: %s' % (e.code, e.reason)
        except urllib_error.URLError as e:
            return None, 'connection error: %s' % e.reason
        except socket.timeout:
            return None, 'connection timeout'

        for line in lines:
            if '=' not in line:
                continue
            value = line.split('=', 1)[1].strip()
            for mode in self._extract_modes_from_value(value):
                modes.add(mode)
            if value and ',' not in value:
                modes.add(value)

        baseline_modes = set(['auto', 'fixed_indoor', 'fixed_outdoor', 'hold'])
        if not modes or len(modes) == 1:
            # Some firmwares only return the currently active mode in list().
            modes = modes.union(baseline_modes)

        self._white_balance_supported_modes = modes
        return self._white_balance_supported_modes, ''

    def _get_supported_day_night_modes(self, parameter_path):
        if self._day_night_supported_modes is not None:
            return self._day_night_supported_modes, ''

        modes = set()

        try:
            lines = self._list_group_lines(parameter_path)
        except urllib_error.HTTPError as e:
            return None, 'HTTP error %d: %s' % (e.code, e.reason)
        except urllib_error.URLError as e:
            return None, 'connection error: %s' % e.reason
        except socket.timeout:
            return None, 'connection timeout'

        for line in lines:
            if '=' not in line:
                continue
            value = line.split('=', 1)[1].strip()
            for mode in self._extract_modes_from_value(value):
                modes.add(mode)
            if value and ',' not in value:
                modes.add(value)

        # Choose baseline modes based on parameter type
        if 'DayNightShift' in parameter_path:
            baseline_modes = set(['auto', 'day', 'night'])
        else:  # IrCutFilter
            baseline_modes = set(['auto', 'yes', 'no'])
        
        if not modes or len(modes) == 1:
            modes = modes.union(baseline_modes)

        self._day_night_supported_modes = modes
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

    def sendPTZCommand(self, pan, tilt, zoom):
        ret = {
            'exception': False,
            'error_msg': '',
            'status': 0
        }

        conn = httplib.HTTPConnection(self.hostname)
        params = { 'pan': pan, 'tilt': tilt, 'zoom': zoom }
        
        try:		   
            url = "/axis-cgi/com/ptz.cgi?camera=1&%s" % urllib_parse.urlencode(params)

            conn.request("GET", url)
            ret['status'] = conn.getresponse().status
            ret['url'] = url

        except socket.error as e:
            ret['exception'] = True
            ret['error_msg'] = e
        except socket.timeout as e:
            ret['exception'] = True
            ret['error_msg'] = e
        return ret

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
        
        return ptz_read

    def setBrightness(self, brightness):
        if brightness < -100 or brightness > 100:
            return {
                'success': False,
                'message': 'brightness value %d is out of range [-100, 100]' % brightness
            }

        return self._update_sensor_parameter('Brightness', brightness, 'brightness')

    def setContrast(self, contrast):
        if contrast < -100 or contrast > 100:
            return {
                'success': False,
                'message': 'contrast value %d is out of range [-100, 100]' % contrast
            }

        return self._update_sensor_parameter('Contrast', contrast, 'contrast')

    def setSaturation(self, saturation):
        if saturation < -100 or saturation > 100:
            return {
                'success': False,
                'message': 'saturation value %d is out of range [-100, 100]' % saturation
            }

        parameter_name, error_message = self._get_saturation_parameter_name()
        if parameter_name is None:
            return {
                'success': False,
                'message': error_message
            }

        # Try the detected/default parameter name
        result = self._update_sensor_parameter(parameter_name, saturation, 'saturation')
        if result['success']:
            return result

        # If Saturation failed, try ColorLevel as fallback
        if parameter_name == 'Saturation':
            result = self._update_sensor_parameter('ColorLevel', saturation, 'saturation')
            if result['success']:
                return result

        return result

    def setWhiteBalance(self, white_balance):
        parameter_path, error_message = self._get_white_balance_parameter_path()
        if parameter_path is None:
            return {
                'success': False,
                'message': error_message
            }

        supported_modes, error_message = self._get_supported_white_balance_modes(parameter_path)
        if supported_modes is None:
            return {
                'success': False,
                'message': error_message
            }

        if white_balance not in supported_modes:
            modes_text = ', '.join(sorted(supported_modes))
            return {
                'success': False,
                'message': 'unsupported white_balance mode "%s". Supported modes: %s' % (white_balance, modes_text)
            }

        return self._update_parameter_path(parameter_path, white_balance, 'white_balance')

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

        return self._update_parameter_path(parameter_path, vapix_mode, 'day_night_mode')

