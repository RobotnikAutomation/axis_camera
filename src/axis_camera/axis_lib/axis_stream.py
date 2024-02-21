try:
    import urllib2 as urllib_request #Not tested in pyhton2
    import urllib2 as urllib_error
except:
    import urllib.request as urllib_request
    import urllib.error as urllib_error
import base64
import socket

class StreamAxis():
    def __init__(self, args):
        self.enable_auth = args['enable_auth']
        self.hostname = args['hostname']
        self.username = args['username']
        self.password = args['password']
        self.camera_number = args['camera_number']
        self.fps = args['fps']
        self.compression = args['compression']
        self.profile = args['profile']

        self._url = 'http://%s/axis-cgi/mjpg/video.cgi?streamprofile=%s&camera=%d&fps=%d&compression=%d' % (
            self.hostname, self.profile, self.camera_number, self.fps, self.compression)
        try:
            encodedstring = base64.encodestring(self.username + ":" + str(self.password))[:-1]
        except:
            encodedstring = base64.encodebytes((self.username + ":" + str(self.password)).encode())[:-1]
        self.auth = "Basic %s" % encodedstring

        # timeout when calling urlopen
        self.timeout = 5
    
        self.videocodec = 'mpeg4'  # h264, mpeg4        

    def getUrl(self):
        return self._url

    def authenticate(self):
        # create a password manager
        password_mgr = urllib_request.HTTPPasswordMgrWithDefaultRealm()

        # Add the username and password, use default realm.
        top_level_url = "http://" + self.hostname
        password_mgr.add_password(None, top_level_url, self.username,
                                                            self.password)
        handler = urllib_request.HTTPBasicAuthHandler(password_mgr)

       # create "opener" (OpenerDirector instance)
        opener = urllib_request.build_opener(handler)

        # ...and install it globally so it can be used with urlopen.
        urllib_request.install_opener(opener)

    def stream(self): #Change msgs
        """
                Reads and process the streams from the camera
        """
        error_reading = False
        error_reading_msg = ''
        try:
            # If flag self.enable_auth is 'True' then use the user/password to access the camera. Otherwise use only self.url
            try:
                if self.enable_auth:
                    req = urllib_request.Request(self._url, None, {"Authorization": self.auth})
                else:
                    req = urllib_request.Request(self._url)
                self.fp = urllib_request.urlopen(req, timeout=self.timeout)
            
            except:
                req = urllib_request.Request(self._url)
                if self.enable_auth:
                    self.authenticate()
                self.fp = urllib_request.urlopen(req)

        except urllib_error.URLError as e:
            error_reading = True
            error_reading_msg = e
        except urllib_error.HTTPError as e:
            error_reading = True
            error_reading_msg = e
        except socket.timeout as e:
            error_reading = True
            error_reading_msg = e

        return error_reading, error_reading_msg

    def getImage(self):
        boundary = self.readLine()
        line = self.readLine()
        header = {}
        while not line == "\r\n":
            # print('read line %s'%line)
            line = line.strip()
            parts = line.split(": ", 1)
            header[parts[0]] = parts[1]
            line = self.readLine()

        content_length = int(header['Content-Length'])
        #print('Length = %d'%content_length)
        img = self.fp.read(content_length)
        line = self.readLine()
        return img

    def readLine(self):
        try:
            line = self.fp.readline().decode()
        except:
            line = self.fp.readline()
        return line