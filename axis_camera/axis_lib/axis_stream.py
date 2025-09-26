try:
    import urllib2 as urllib_request #Not tested in pyhton2
except:
    import urllib.request as urllib_request

import requests

class StreamAxis():
    def __init__(self, args):
        self.url_response = None
        self.hostname = args['hostname']
        self.camera_number = args['camera_number']
        self.fps = args['fps']
        self.compression = args['compression']
        self.profile = args['profile']
        if 'timeout' not in args.keys():
            args['timeout'] = 5
        self.timeout = args['timeout']
        if 'videocodec' not in args.keys():
            args['videocodec'] = 'mpeg4'
        self.videocodec = args['videocodec'] # h264, mpeg4
        video_formats = self.getSupportedImageFormats()
        if self.videocodec not in video_formats:
            print("Video codec %s not supported. Supported videocodecs are %s. Using %s" % (self.videocodec, video_formats, video_formats[0]))
            self.videocodec = video_formats[0]

        self._url = 'http://%s/axis-cgi/mjpg/video.cgi?streamprofile=%s&camera=%d&fps=%d&compression=%d&videocodec=%s' % (
            self.hostname, self.profile, self.camera_number, self.fps, self.compression, self.videocodec)

        self.resolution = args["resolution"]
        resolution_options = self.getSupportedResolutions()
        if not resolution_options:
            print("Could not get supported resolutions. Using default.")
        else:
            if self.resolution not in resolution_options:
                print("Resolution %s not supported.  Suported resolutions are %s. Using %s" % (self.resolution, resolution_options, resolution_options[0]))
                self.resolution = resolution_options[0]
            self._url += '&resolution=%s' % self.resolution

    def getSupportedImageFormats(self):
        query = {}
        url = "http://%s/axis-cgi/param.cgi?action=list&group=Properties.Image.Format&camera=%s" % (self.hostname, self.camera_number)
        try:
            response = requests.get(url, params=query, timeout=self.timeout)
            for line in response.text.splitlines():
                split_line = line.split('=', 2)
                if len(split_line) == 2:
                    options = split_line[1].strip().split(',')

        except Exception as e:
            options = ['mjpeg', 'h264']

        return options

    def getSupportedResolutions(self):
        query = {}
        url = "http://%s/axis-cgi/param.cgi?action=list&group=Properties.Image.Resolution&camera=%s" % (self.hostname, self.camera_number)
        try:
            response = requests.get(url, params=query, timeout=self.timeout)
            for line in response.text.splitlines():
                split_line = line.split('=', 2)
                if len(split_line) == 2:
                    options = split_line[1].strip().split(',')

        except Exception as e:
            options = []

        return options

    def getUrl(self):
        return self._url


    def stream(self):
        """
                Reads and process the streams from the camera
        """
        error = False
        error_msg = ''
        try:
            req = urllib_request.Request(self._url)
            self.url_response = urllib_request.urlopen(req, timeout=self.timeout)
    
        except Exception as e:
            error = True
            error_msg = e

        return error, error_msg

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
        img = self.url_response.read(content_length)
        line = self.readLine()
        return img

    def readLine(self):
        try:
            line = self.url_response.readline().decode()
        except:
            line = self.url_response.readline()
        return line