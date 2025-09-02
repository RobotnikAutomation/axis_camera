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
    def __init__(self, hostname, camera_number = 1, timeout = 1000):
        self.hostname = hostname
        self.camera_number = camera_number
        self.timeout = timeout

    # TODO: Handle when connection is not available. Added timeout

    def queryPTZ(self, query : dict):
        """
            Queries the PTZ camera with the given query parameters.
            Returns a dictionary with the status and parameters.
            It always returns the HTTP status code, the URL used, and if there was an error.
        """
        params = {}
        conn = httplib.HTTPConnection(self.hostname, timeout = self.timeout)
        try:
            url = "/axis-cgi/com/ptz.cgi?camera=%s&%s" % (self.camera_number, urllib_parse.urlencode(query))
            conn.request("GET", url)
            response = conn.getresponse()
            params["status"] = response.status
            params["url"] = url
            if response.status == 200:
                body = response.read()
                params["raw"] = body
                body_lines = body.splitlines()
                for line in body_lines:
                    try:
                        decoded_line = line.split('=', 2)
                    except:
                        decoded_line = line.decode().split('=', 2)
                    if len(decoded_line) == 2:
                        params[decoded_line[0].strip()] = decoded_line[1].strip()

            params["error"] = False
            params["error_msg"] = response.reason

        except socket.timeout as e:
            params["error"]= True
            params["error_msg"] = e
        except socket.error as e:
            params["error"]= True
            params["error_msg"] = e
        except ValueError as e:
            params["error"]= True
            params["error_msg"] = e
        except Exception as e:
            params["error"]= True
            params["error_msg"] = e

        return params

    def getPTZStatus(self):
        """
            Gets the current status of the PTZ camera.
            Returns a dictionary with the status parameters.
        """
        return self.queryPTZ({'query' : 'status'})

    def getPTZPosition(self):
        """
            Gets the current position of the PTZ camera.
            Returns a dictionary with the position parameters.
        """
        response = self.queryPTZ({'query' : 'position'})
        if response["error"]:
            return response

        if response["status"] != 200:
            response["error"] = True
            return response

        pan = math.radians(float(response['pan']))
        tilt = math.radians(float(response['tilt']))

        if 'zoom' in response:
            zoom = float(response['zoom'])
        else:
            zoom = 0.0
        # Optional params (depending on model)
        if 'iris' in response:
            iris = float(response['iris'])
        else:
            iris = 0.0
        if 'focus' in response:
            focus = float(response['focus'])
        else:
            focus = 0.0
        if 'autofocus' in response:
            autofocus = (response['autofocus'] == 'on')
        else:
            autofocus = False
        if 'autoiris' in response:
            autoiris = (response['autoiris'] == 'on')
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
            "error" : False,
            "error_msg" : ''
        }

        return ptz_read

    def getPTZInfo(self):
        """
            Gets the PTZ information of the camera.
            Returns a dictionary with the PTZ information parameters.
        """
        return self.queryPTZ({'info' : ''})

    def sendPTZCommand(self, pan, tilt, zoom):
        """
            Sends the PTZ command to the camera.
            pan and tilt are in radians, zoom is a float value.
        """

        params = { 'pan': pan, 'tilt': tilt, 'zoom': zoom }
        return self.queryPTZ(params)

    def sendPTZVelocityCommand(self, pan, tilt, zoom):
        """
            Sends the PTZ velocity command to the camera.
            pan and tilt are in radians, zoom is a float value.
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

        params = { 'continuouspantiltmove': f'{pan},{tilt}', 'continuouszoommove': zoom }
        return self.queryPTZ(params)
