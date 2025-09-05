import time

class Timer():
    def __init__(self, duration : float, start_immediately=True):
        if start_immediately:
            self.start()
        else:
            self._init_time = None
        self._duration = duration
    
    def start(self):
        self._init_time = time.time()
    
    def reset(self):
        self.start()
    
    def stop(self):
        self._init_time = None

    def isFinished(self):
        if self.isStopped():
            return False
        return (time.time() - self._init_time) >= self._duration
    
    def isStopped(self):
        return self._init_time is None

    def getDuration(self):
        return self._duration

    def setDuration(self, duration : float):
        self._duration = duration
    
    def getInitTime(self):
        return self._init_time
    
    
