from video_get import VideoGet
from video_show import VideoShow

import cv2 as cv

import logging
import numpy as np

from time import time, sleep
from imutils import grab_contours

from math import tan, atan
from math import pi

from math import radians
import serial

class Color_Controller(object):
    #constructor of ColorController
    def __init__(self, source=0) -> None:
        #Logger settings
        self.logger = logging.getLogger("Color_Controller")
        self.logger.setLevel(logging.DEBUG)
        formatter = logging.Formatter("%(asctime)s:%(levelname)s:%(name)s:%(funcName)s = %(message)s")
        stream_handler = logging.StreamHandler()
        stream_handler.setFormatter(formatter)
        self.logger.addHandler(stream_handler)
        self.logger.info("!color_controller detect!")
        
        #Camera settings
        self.stopped = False
        self.source = 0
        self.FoV = 62.2
        self.FoV_y = 48.8
        
        #width
        self.width_fov = 0
        self.target_angle_width = 0
        self.width_fov_radians = radians(self.FoV) 
        self.camera_width_pixels = 640 
        self.horizontal_focal_length_pixels = 0
        
        #height
        self.height_fov = 0
        self.target_angle_height = 0
        self.height_fov_radians = radians(self.FoV_y)  
        self.camera_height_pixels = 480 
        self.vertical_focal_length_pixels = 0
       
        #Pan-tilt settings
        self.pan_angle = 0
        self.tilt_angle = 0
        
        #Ardunio settings
        self.baudrate = 9600
        self.delay = 0.0
        self.num_frames_processed = 0 
        self.ardunio = serial.Serial("/dev/ttyUSB0", baudrate=self.baudrate , timeout=1)

        
    def open(self,source):
        try:
            self.video_getter = VideoGet(source).start()
            self.video_shower = VideoShow(self.video_getter.frame).start()
        except Exception:
            self.logger.error("Could not find video source",exc_info=True)
            raise RuntimeError("Could not open video source!")

    def close(self):
        try:
            self.video_getter.stop()
            self.video_shower.stop()
        except Exception:  
            self.logger.error("Could not find to close video source",exc_info=True)
            raise RuntimeError("Could not close video source!")
    
    def controller(self, frame):
        frame = self.find_window_centre_point(frame)
        lowerBound = np.array([0,146,135])
        upperBound = np.array([179,255,255])
        x , y = 0 , 0
        try:
            blurred = cv.GaussianBlur(frame, (11, 11), 0)
            frameHSV = cv.cvtColor(blurred, cv.COLOR_BGR2HSV)
            mask = cv.inRange(frameHSV, lowerBound, upperBound)
            morph1 = cv.erode(mask, None, iterations=4)
            morph2 = cv.dilate(morph1, None, iterations=4)
            cnts = cv.findContours(morph2.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
            cnts = grab_contours(cnts)
            if len(cnts) > 0:
                c = max(cnts, key=cv.contourArea)
                ((x, y), radius) = cv.minEnclosingCircle(c)
                M = cv.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                if radius > 10:
                    cv.circle(frame, (int(x), int(y)), int(radius),(0, 255, 255), 2)
                    cv.circle(frame, center, 5, (0, 255, 0), -1)
            return (int(x), int(y))
        except Exception:
            self.logger.error("Could not clearly using filters!",exc_info=True)
            raise RuntimeError("Error with filters!!")
    
    def find_window_centre_point(self, frame):
        try:
            height, width, _ = frame.shape # 640 w , 480 h
            center = (int(width/2), int(height/2)) 
            frame = cv.circle(frame, center, 5, (0,0,0), 1)
            frame = cv.line(frame, ((320), 0), (320,height), (0, 255, 0), 1)
            frame = cv.line(frame, (0, 240), (width, 240), (0, 255, 0), 1)
            return frame
        except Exception:
            self.logger.error("Could not find clearly center point!")
            raise RuntimeError("Error finding centre point!!")
    
    def get_position(self, frame):
        try:
            x, y = self.controller(frame)
            return[x, y]
        except Exception:
            self.logger.error("Could not clearly using position controller!",exc_info=True)
            raise RuntimeError("Error with positions!!")
    
    def position_controller(self, frame):
        try:
            x, y = (self.get_position(frame))
            height, width, _ = frame.shape
            diff_width_x  = int( x - width  / 2 )
            diff_height_y = int( y - height / 2 )
            self.width_fov  = int ( width  / self.FoV )
            self.height_fov = int ( height / self.FoV_y)          
            self.horizontal_focal_length_pixels = width / (2 * tan(self.width_fov_radians / 2))
            self.width_fov  = self.horizontal_focal_length_pixels * 2 * atan (width / (2 * self.horizontal_focal_length_pixels))
            self.target_angle_width =  int (diff_width_x / self.width_fov * self.FoV)
            self.vertical_focal_length_pixels = height / (2 * tan(self.height_fov_radians / 2))
            self.height_fov = self.vertical_focal_length_pixels * 2 * atan (height / (2 * self.vertical_focal_length_pixels))
            self.target_angle_height = int (diff_height_y / self.height_fov * self.FoV_y)                     
            return (int(self.target_angle_width), int(self.target_angle_height))
        except ZeroDivisionError:
            self.logger.error("Could not calculate position!")
            raise RuntimeError("Error calculating position!!")

    def angle_generator(self, frame:int):
        try:
            diff_x, diff_y, diff_angle, diff_angle1 = self.position_controller(frame)
            # x, y = (self.get_position(frame))
            #max x distance 230, max y distance 315
            #print(x , y)
            #define region
            if (diff_x > 0 and diff_x < 600) and (diff_y > 0 and diff_y < 465):  # +x  +y 
                print("Region 1!")
                print("|| CURRENT ANGLE:" , diff_angle, "||", diff_angle1, "||")
                return diff_angle,diff_angle1
                #diff_x, diff_y, diff_angle, diff_angle1 = self.position_controller(frame)
                #print(f"|| PAN: {self.pan_angle} || TILT: {self.pan_tilt} ||")
            elif (diff_x < 0 and diff_x > -600) and (diff_y > 0 and diff_y < 465): # -x  +y
                print("Region 2!")
                print("|| CURRENT ANGLE:" , diff_angle, "||", diff_angle1, "||")
                return diff_angle,diff_angle1
                #diff_x, diff_y, diff_angle, diff_angle1= self.position_controller(frame)
                #print(f"|| PAN: {self.pan_angle} || TILT: {self.pan_tilt} ||")
            elif (diff_x < 0 and diff_x > -600) and (diff_y < 0 and diff_y > -465):  # -x  -y
                print("Region 3!")
                print("|| CURRENT ANGLE:" , diff_angle, "||", diff_angle1, "||")
                return diff_angle,diff_angle1
                #diff_x, diff_y, diff_angle, diff_angle1= self.position_controller(frame)
                #print(f"|| PAN: {self.pan_angle} || TILT: {self.pan_tilt} ||")
            elif (diff_x > 0 and diff_x < 600) and (diff_y < 0 and diff_y > -465):  # +x  -y
                print("Region 4!")
                print("|| CURRENT ANGLE:" , diff_angle, "||", diff_angle1, "||")
                return diff_angle, diff_angle1
                #diff_x, diff_y, diff_angle, diff_angle1 = self.position_controller(frame)
                #print(f"|| PAN: {self.pan_angle} || TILT: {self.pan_tilt} ||")
            else:
                print("NO REGION!!")
        except Exception:
            self.logger.error("Could not calculate desired angle!")
            raise RuntimeError("Error calculating angle!!")
    
    def send_data_arduino(self,frame):
        try:
            pan_angle , tilt_angle = self.position_controller(frame)
            self.ardunio.write(str(pan_angle).encode())
            self.ardunio.write("\n".encode())    
            self.ardunio.write(str(tilt_angle).encode())
            self.ardunio.write("\n".encode())    
        except Exception as exception_error:
            print("Error occurred. Exiting Program")
            print("Error: " + str(exception_error))
        
    def main(self):
        dispW=640
        dispH=480
        flip=0
        camSet='nvarguscamerasrc wbmode=3 tnr-mode=2 tnr-strength=1 ee-mode=2 ee-strength=1 ! video/x-raw(memory:NVMM), width=3264, height=2464, format=NV12, framerate=21/1 ! nvvidconv flip-method='+str(flip)+' ! video/x-raw, width='+str(dispW)+', height='+str(dispH)+', format=BGRx ! videoconvert ! video/x-raw, format=BGR ! videobalance contrast=1.5 brightness=-.2 saturation=1.2 ! appsink'
        self.open(source=camSet)
        start = time()
        while not self.stopped:
            if self.video_getter.stopped or self.video_shower.stopped:
                self.close()
                break
            else: 
                frame = self.video_getter.frame
                sleep(self.delay)
                self.num_frames_processed += 1       
                self.video_shower.frame = frame
                #sleep(self.delay)
                self.send_data_arduino(frame)
        end = time()
        elapsed = end - start
        fps = self.num_frames_processed/elapsed 
        print("FPS: {} , Elapsed Time: {} ".format(fps, elapsed))
        
if __name__ == "__main__":
    print("CV2 Version: ", cv.__version__)
    print("----------------------------------------------------------------")
    controller = Color_Controller()
    controller.main()
