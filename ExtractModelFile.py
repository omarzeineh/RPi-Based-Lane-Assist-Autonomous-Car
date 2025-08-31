import cv2
import numpy as np
import time
import sys
sys.path.append('/usr/lib/python3/dist-packages')
from picamera2 import Picamera2
from ultralytics import YOLO
from gpiozero import Motor

model = YOLO("yolov8n.pt")

model.export(format="openvino", 256)


model = YOLO('yolov8n_openvino_model/')
