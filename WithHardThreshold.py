import cv2
import numpy as np
import time
import sys
sys.path.append('/usr/lib/python3/dist-packages')
from picamera2 import Picamera2
from ultralytics import YOLO
from gpiozero import Motor


motor1 = Motor(23, 14)
motor2 = Motor(24, 27)




Kp = 0.15  #0.25
Ki = 0
Kd = 0
Previous = time.time()
PreviousObj = time.time()
errorPrevious = 0
Integral = 0

KpObj = 10
KiObj = 0
KdObj = 0
errorPreviousObj = 0
IntegralObj = 0
base = 255
#124
distanceObject = 0


height =480
width=640
middle =((width//2),(height//2))
cam = Picamera2()
cam.configure(cam.create_video_configuration(main={"format": 'XRGB8888',"size": (width, height)}))
cam.start()

pt = 200 #200



model = YOLO('yolov8n_openvino_model/')





while True:
    rightLane = None
    leftLane = None
    distanceObject = 0
    t=time.time()
    frame = cam.capture_array()[:,:,:3]
    frame = cv2.flip(frame, -1) 
    frame = cv2.GaussianBlur(frame, (11, 11), 0)
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    edges = cv2.Canny(gray, 60, 120)
    ROI = frame[pt:height, 0:width].copy()
    ROIedge = edges[pt:height, 0:width].copy()
    lines = cv2.HoughLines(ROIedge, 1, np.pi / 180, 40 ) 
    lin = frame
    lin2 = ROIedge
    rightmotor = 0
    leftmotor = 0
    results = model.predict(lin, imgsz=256)
    obstacles = []
    for r in results:
        for obj in r.boxes:
            x, y, w, h = obj.xywh.tolist()[0]
            label = obj.cls
            obstacles.append((x, y, w, h, label))
    
    if lines is not None:
        prevRightDistanceFromMid = 0
        prevLeftDistanceFromMid = 0
        #rightLane = None
        #leftLane = None
        for line in lines:
            r, th = line[0]
            a = np.cos(th)
            b = np.sin(th)
            x0 = a*r
            y0 = b*r
            x1 = int(x0 + 1000*(-b))
            y1 = int(y0 + 1000*a)
            x2 = int(x0 + 1000*(b))
            y2 = int(y0 + 1000*(-a))

            if x2-x1 != 0:
                angle = np.arctan((y2 - y1) / (x2 - x1))
                if (angle < np.pi/2 and angle > np.pi/30) or (angle > -np.pi/2 and angle < -np.pi/30):
                    m = (y2 - y1) / (x2 - x1)
                    inter = -m*x1 + y1
                    midx = (height-pt-inter)/m
                    distanceFromMid = midx - width/2
                    if distanceFromMid > 0:
                        if distanceFromMid > prevRightDistanceFromMid:
                            rightLane = line[0]
                            prevRightDistanceFromMid = distanceFromMid
                    else:
                        if abs(distanceFromMid) > prevLeftDistanceFromMid:
                            leftLane = line[0]
                            prevLeftDistanceFromMid = abs(distanceFromMid)


        lin = results[0].plot()
        



        
        if rightLane is not None:
            r, th = rightLane
            a = np.cos(th)
            b = np.sin(th)
            xr0 = a*r
            yr0 = b*r
            xr1 = int(xr0 + 1500*(-b))
            yr1 = int(yr0 + 1500*a)
            xr2 = int(xr0 + 1500*(b))
            yr2 = int(yr0 + 1500*(-a))
            mr = (yr2 - yr1) / (xr2 - xr1)
            interr = -mr * xr1 + yr1
            lin = cv2.line(lin, (xr1, yr1+pt), (xr2, yr2+pt), (255, 255, 255), 2)
            lin2 = cv2.line(lin2, (xr1, yr1), (xr2, yr2), (255, 255, 255), 2)
        if leftLane is not None:
            r, th = leftLane
            a = np.cos(th)
            b = np.sin(th)
            xl0 = a*r
            yl0 = b*r
            xl1 = int(xl0 + 1500*(-b))
            yl1 = int(yl0 + 1500*a)
            xl2 = int(xl0 + 1500*(b))
            yl2 = int(yl0 + 1500*(-a))
            ml = (yl2 - yl1) / (xl2 - xl1)
            interl = -ml * xl1 + yl1
            lin = cv2.line(lin, (xl1, yl1+pt), (xl2, yl2+pt), (255, 255, 255), 2)
            lin2 = cv2.line(lin2, (xl1, yl1), (xl2, yl2), (255, 255, 255), 2)
        if leftLane is not None and rightLane is not None:
            if (ml-mr) != 0:
                x = (-interl+interr)/(ml-mr)
                y = mr*x + interr
                midxl = (height - pt - interl) / ml
                midxr = (height - pt - interr) / mr
                distR = midxr-width/2
                distL = midxl-width/2
                error =  distR + distL
                Current = time.time()
                dt = Current - Previous
                Derivative = (error - errorPrevious) / dt
                Integral = Integral + error * dt
                Control = Kp * error + Ki * Integral + Kd * Derivative
                errorPrevious = error
                rightmotor = base - Control
                leftmotor = base + Control

                
                if obstacles:
                    closest = 0
                    index = 0
                    for i in range(len(obstacles)):
                  
                        if(obstacles[i][1] + obstacles[i][3]/2 > 250):
                            if(obstacles[i][0] < 480 and obstacles[i][0] > 160):
                                if obstacles[i][1]+obstacles[i][3]/2 > closest:
                                    index = i
                                    closest = obstacles[i][1]+obstacles[i][3]/2
                                    distanceObject = 480-closest
               

            errorObj =  -distanceObject
            print(errorObj)
            CurrentObj = time.time()
            dtObj = CurrentObj - PreviousObj
            DerivativeObj = (errorObj - errorPreviousObj) / dtObj
            IntegralObj = IntegralObj + errorObj * dtObj
            ControlObj = KpObj * errorObj + KiObj * IntegralObj + KdObj * DerivativeObj
            errorPreviousObj = errorObj
            rightmotor = rightmotor + ControlObj
            leftmotor = leftmotor + ControlObj

            Previous = Current
            errorPrevious = error

            if(rightmotor > 255):
                rightmotor = 255
            elif(rightmotor < 0) :
                rightmotor = 0
            if(leftmotor > 255):
                leftmotor = 255
            elif(leftmotor < 0):
                leftmotor = 0
        if leftLane is not None and rightLane is None:
            rightmotor = 0
            leftmotor = base
        if leftLane is None and rightLane is not None:
            rightmotor = base
            leftmotor = 0

       
        motor1.forward(rightmotor/255)
        motor2.forward(leftmotor/255)

        print("Left Motor: " + str(leftmotor) + " Right Motor: " + str(rightmotor))
    cv2.imshow('Lines and Objects', lin)

    key = cv2.waitKey(1) & 0xff
    if key == ord('q'):
        motor1.stop()
        motor2.stop()
        break

cam.stop()
cv2.destroyAllWindows()