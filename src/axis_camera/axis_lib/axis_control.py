try:
    import urllib
except:
	import urllib.request, urllib.error, urllib.parse
try:
    import httplib
except:
    import http.client
import socket
import math

class ControlAxis():
    def __init__(self, hostname):
        self.hostname = hostname

    def sendPTZCommand(self, pan, tilt, zoom):
        ret = {
            'exception': False,
            'error_msg': '',
            'status': 0
        }
        try:
            conn = httplib.HTTPConnection(self.hostname)
        except:
            conn = http.client.HTTPConnection(self.hostname)
        params = { 'pan': pan, 'tilt': tilt, 'zoom': zoom }
        
        try:		
            try:
                url = "/axis-cgi/com/ptz.cgi?camera=1&%s" % urllib.urlencode(params)
            except:
                url = "/axis-cgi/com/ptz.cgi?camera=1&%s" % urllib.parse.urlencode(params)

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
        try:
            conn = httplib.HTTPConnection(self.hostname)
        except:
            conn = http.client.HTTPConnection(self.hostname)

        params = { 'query':'position' }
        try:
            try:
                conn.request("GET", "/axis-cgi/com/ptz.cgi?%s" % urllib.urlencode(params))
            except:
                conn.request("GET", "/axis-cgi/com/ptz.cgi?%s" % urllib.parse.urlencode(params))
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

        