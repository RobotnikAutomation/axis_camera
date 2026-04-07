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
                if 'brightness' in params:
                    brightness = float(params['brightness'])
                else:
                    brightness = None

                ptz_read = {
                    "pan" : pan,
                    "tilt" : tilt,
                    "zoom" : zoom,
                    "focus" : focus,
                    "autofocus" : autofocus,
                    "iris" : iris,
                    "autoiris" : autoiris,
                    "brightness" : brightness,
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

        result = self._update_sensor_parameter('Brightness', brightness, 'brightness')
        if result['success']:
            self._last_known_image_settings['brightness'] = brightness
        return result

    def setContrast(self, contrast):
        if contrast < -100 or contrast > 100:
            return {
                'success': False,
                'message': 'contrast value %d is out of range [-100, 100]' % contrast
            }

        result = self._update_sensor_parameter('Contrast', contrast, 'contrast')
        if result['success']:
            self._last_known_image_settings['contrast'] = contrast
        return result

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
            self._last_known_image_settings['saturation'] = saturation
            return result

        # If Saturation failed, try ColorLevel as fallback
        if parameter_name == 'Saturation':
            result = self._update_sensor_parameter('ColorLevel', saturation, 'saturation')
            if result['success']:
                self._last_known_image_settings['saturation'] = saturation
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

        if readable_fields > 0:
            ret['success'] = True
            if readable_fields < 6:
                ret['message'] = 'partial read: %d/6 fields read from camera; remaining fields are fallback values' % readable_fields
        elif cached_fields > 0:
            ret['success'] = True
            ret['message'] = 'camera read unavailable on this firmware; returning %d/6 cached values from successful set_* calls' % cached_fields
        else:
            ret['success'] = False
            ret['message'] = 'camera does not expose readable image settings via VAPIX list on this firmware'
        return ret
