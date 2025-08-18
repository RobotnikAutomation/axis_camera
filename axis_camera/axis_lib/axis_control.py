try:
    import urllib.parse as urllib_parse
except:
	import urllib as urllib_parse #Not tested in pyhton2

try:
    import httplib
except:
    import http.client as httplib

import socket
import math

class ControlAxis():
    def __init__(self, hostname, camera_id = 1, timeout = 1000):
        self.hostname = hostname
        self.camera_id = camera_id
        self.timeout = timeout

    # TODO: Handle when connection is not available

    def getPTZStatus(self):
        params = {}
        conn = httplib.HTTPConnection(self.hostname, timeout = self.timeout)
        query = { 'query':'status' }
        try:
            conn.request("GET", "/axis-cgi/com/ptz.cgi?%s" % urllib_parse.urlencode(query))
            response = conn.getresponse()
            if response.status == 200:
                body = response.read()
                body_lines = body.splitlines()
                for line in body_lines:
                    try:
                        decoded_line = line.split('=', 2)
                    except:
                        decoded_line = line.decode().split('=', 2)
                    if len(decoded_line) == 2:
                        params[decoded_line[0].strip()] = decoded_line[1].strip()

            params["error_reading"] = False
            params["error_reading_msg"] = ''

        except socket.timeout as e:
            params["error_reading"]= True
            params["error_reading_msg"] = e
        except socket.error as e:
            params["error_reading"]= True
            params["error_reading_msg"] = e
        except ValueError as e:
            params["error_reading"]= True
            params["error_reading_msg"] = e
        except Exception as e:
            params["error_reading"]= True
            params["error_reading_msg"] = e

        return params

    def getPTZInfo(self):
        params = {}
        conn = httplib.HTTPConnection(self.hostname, timeout = self.timeout)
        query = { 'info':'' }
        try:
            conn.request("GET", "/axis-cgi/com/ptz.cgi?%s" % urllib_parse.urlencode(query))
            response = conn.getresponse()
            if response.status == 200:
                body = response.read()
                body_lines = body.splitlines()
                for line in body_lines:
                    try:
                        decoded_line = line.split('=', 2)
                    except:
                        decoded_line = line.decode().split('=', 2)
                    if len(decoded_line) == 2:
                        params[decoded_line[0].strip()] = decoded_line[1].strip()

            params["error_reading"] = False
            params["error_reading_msg"] = ''

        except socket.timeout as e:
            params["error_reading"]= True
            params["error_reading_msg"] = e
        except socket.error as e:
            params["error_reading"]= True
            params["error_reading_msg"] = e
        except ValueError as e:
            params["error_reading"]= True
            params["error_reading_msg"] = e
        except Exception as e:
            params["error_reading"]= True
            params["error_reading_msg"] = e

        return params

    def sendPTZCommand(self, pan, tilt, zoom):
        ret = {
            'exception': False,
            'error_msg': '',
            'status': 0
        }

        conn = httplib.HTTPConnection(self.hostname, timeout = self.timeout)
        params = { 'pan': pan, 'tilt': tilt, 'zoom': zoom }
        
        try:		   
            url = "/axis-cgi/com/ptz.cgi?camera=%s&%s" % (self.camera_id, urllib_parse.urlencode(params))

            conn.request("GET", url)
            ret['status'] = conn.getresponse().status
            ret['url'] = url

        except socket.timeout as e:
            ret['exception'] = True
            ret['error_msg'] = e
        except socket.error as e:
            ret['exception'] = True
            ret['error_msg'] = e
        
        return ret

    def sendPTZVelocityCommand(self, pan, tilt, zoom):
        """
            Sends the PTZ velocity command to the camera.
            pan, tilt and zoom are in radians
        """

        """
        Extracted from AXIS M5525-E PTZ Dome 
        pan tilt speeds are in degrees per second
        speeds = (
            "value=1|speed=1.8,"
            "value=28|speed=5,"
            "value=40|speed=11,"
            "value=44|speed=14,"
            "value=62|speed=37,"
            "value=66|speed=44,"
            "value=73|speed=59,"
            "value=79|speed=75,"
            "value=84|speed=90,"
            "value=88|speed=103,"
            "value=93|speed=121,"
            "value=97|speed=137,"
            "value=100|speed=150"
        )
        """
        ret = {
            'exception': False,
            'error_msg': '',
            'status': 0
        }

        conn = httplib.HTTPConnection(self.hostname, timeout = self.timeout)
        params = { 'continuouspantiltmove': f'{pan},{tilt}', 'continuouszoommove': zoom }

        try:
            url = "/axis-cgi/com/ptz.cgi?camera=%s&%s" % (self.camera_id, urllib_parse.urlencode(params))

            conn.request("GET", url)
            ret['status'] = conn.getresponse().status
            ret['url'] = url

        except socket.timeout as e:
            ret['exception'] = True
            ret['error_msg'] = e
        except socket.error as e:
            ret['exception'] = True
            ret['error_msg'] = e

        return ret

    def getPTZState(self):
        """
            Gets the current ptz state/position of the camera
        """
        ptz_read = {}
        conn = httplib.HTTPConnection(self.hostname, timeout = self.timeout)
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

        except socket.timeout as e:
            ptz_read["error_reading"]= True
            ptz_read["error_reading_msg"] = e         
        except socket.error as e:
            ptz_read["error_reading"]= True
            ptz_read["error_reading_msg"] = e
        except ValueError as e:
            ptz_read["error_reading"]= True
            ptz_read["error_reading_msg"] = e
        
        return ptz_read

        