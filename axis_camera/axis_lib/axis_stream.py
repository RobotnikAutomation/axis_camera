try:
    import urllib2 as urllib_request #Not tested in pyhton2
except:
    import urllib.request as urllib_request

import requests
import time

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

        # Connection state management
        self.is_connected = False
        self.reconnection_attempts = 0
        self.max_reconnection_delay = 60.0  # Max backoff delay in seconds
        self.base_reconnection_delay = 1.0  # Base delay for exponential backoff
        self.last_reconnection_attempt = 0
        
        # Max buffering time parameter (default 1 second)
        self.max_buffering_time = args.get('max_buffering_time', 1.0)

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


    def connect(self):
        """
        Establishes connection to the camera stream.
        Uses exponential backoff for reconnection attempts.
        Returns True if connection is successful, False otherwise.
        """
        # Check if already connected
        if self.is_connected and self.url_response is not None:
            return True
        
        # Exponential backoff logic
        current_time = time.time()
        if self.reconnection_attempts > 0:
            # Calculate exponential backoff delay
            backoff_delay = min(
                self.base_reconnection_delay * (2 ** (self.reconnection_attempts - 1)),
                self.max_reconnection_delay
            )
            
            # Check if enough time has passed since last attempt
            if current_time - self.last_reconnection_attempt < backoff_delay:
                return False
        
        self.last_reconnection_attempt = current_time
        
        try:
            req = urllib_request.Request(self._url)
            self.url_response = urllib_request.urlopen(req, timeout=self.timeout)
            self.is_connected = True
            self.reconnection_attempts = 0  # Reset on successful connection
            return True
    
        except Exception as e:
            self.is_connected = False
            self.reconnection_attempts += 1
            print(f"Connection attempt {self.reconnection_attempts} failed: {e}")
            return False

    def disconnect(self):
        """
        Closes the connection to the camera stream.
        """
        if self.url_response is not None:
            try:
                self.url_response.close()
            except:
                pass
            self.url_response = None
        self.is_connected = False

    def stream(self):
        """
        Maintains the stream connection (legacy method for compatibility).
        Returns error status and message.
        """
        error = not self.connect()
        error_msg = '' if not error else f"Failed to connect after {self.reconnection_attempts} attempts"
        return error, error_msg

    def getImage(self):
        """
        Reads an image from the stream.
        Ensures that buffering time does not exceed max_buffering_time.
        Returns the image bytes or None if connection failed.
        """
        if not self.is_connected:
            if not self.connect():
                return None
        
        try:
            start_time = time.time()
            
            # Read boundary
            boundary = self.readLine()
            if not boundary:
                self.disconnect()
                return None
            
            # Read headers
            line = self.readLine()
            header = {}
            while line and not line == "\r\n":
                line = line.strip()
                if ": " in line:
                    parts = line.split(": ", 1)
                    header[parts[0]] = parts[1]
                line = self.readLine()
                
                # Check if we're exceeding max buffering time
                if time.time() - start_time > self.max_buffering_time:
                    print(f"Warning: Header reading exceeded max buffering time ({self.max_buffering_time}s)")
                    self.disconnect()
                    return None
            
            if 'Content-Length' not in header:
                self.disconnect()
                return None
            
            content_length = int(header['Content-Length'])
            
            # Read image data
            img = self.url_response.read(content_length)
            
            # Read trailing newline
            line = self.readLine()
            
            # Check total buffering time
            total_time = time.time() - start_time
            if total_time > self.max_buffering_time:
                print(f"Warning: Image retrieval took {total_time:.3f}s, exceeding max buffering time ({self.max_buffering_time}s)")
            
            return img
            
        except Exception as e:
            print(f"Error reading image: {e}")
            self.disconnect()
            return None

    def readLine(self):
        """
        Reads a line from the stream.
        Returns None if connection is not available.
        """
        if self.url_response is None:
            return None
        try:
            line = self.url_response.readline().decode()
        except:
            try:
                line = self.url_response.readline()
            except:
                return None
        return line