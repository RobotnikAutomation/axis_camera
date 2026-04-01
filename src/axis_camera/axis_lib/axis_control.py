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
        ret = {
            'success': False,
            'message': ''
        }

        if brightness < -100 or brightness > 100:
            ret['message'] = 'brightness value %d is out of range [-100, 100]' % brightness
            return ret

        params = urllib_parse.urlencode({
            'action': 'update',
            'ImageSource.I0.Sensor.Brightness': brightness
        })
        url = 'http://%s/axis-cgi/admin/param.cgi?%s' % (self.hostname, params)

        try:
            password_mgr = urllib_request.HTTPPasswordMgrWithDefaultRealm()
            password_mgr.add_password(None, 'http://' + self.hostname, self._username, self._password)
            auth_handler = urllib_request.HTTPDigestAuthHandler(password_mgr)
            opener = urllib_request.build_opener(auth_handler)

            response = opener.open(url, timeout=5)
            body = response.read().decode('utf-8').strip()

            if body == 'OK':
                ret['success'] = True
                ret['message'] = 'brightness set to %d' % brightness
            else:
                ret['message'] = 'camera rejected update: %s' % body

        except urllib_error.HTTPError as e:
            ret['message'] = 'HTTP error %d: %s' % (e.code, e.reason)
        except urllib_error.URLError as e:
            ret['message'] = 'connection error: %s' % e.reason
        except socket.timeout as e:
            ret['message'] = 'connection timeout'

        return ret

