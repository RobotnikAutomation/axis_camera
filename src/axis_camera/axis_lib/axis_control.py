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
    def __init__(self, hostname):
        self.hostname = hostname

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
                    parsed_params = dict([s.split('=', 2) for s in body.splitlines()])
                except:
                    parsed_params = dict([s.decode().split('=', 2) for s in body.splitlines()])

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

        