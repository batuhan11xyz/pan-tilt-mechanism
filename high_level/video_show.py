from threading import Thread
import cv2
import threading

class VideoShow:

    def __init__(self, frame=None):
        self.frame = frame
        self.stopped = False

    def start(self):
        Thread(target=self.show, args=()).start()
        return self

    def show(self):
        while not self.stopped:
            cv2.imshow("Video", self.frame)
            if cv2.waitKey(1) == ord("q"):
                self.stopped = True

    def stop(self):
        self.stopped = True
    
    def get_id(self):
            if hasattr(self, "_thread_id"):
                return self._thread_id
            for id_ , thread in threading._active.items():
                if thread is self:
                    return id_