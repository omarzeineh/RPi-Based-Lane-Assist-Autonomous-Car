# RPi-Based Lane-Assist Autonomous Car

This repository contains the full implementation of a **hand-designed autonomous driving system** built on a **Raspberry Pi 5**. The project integrates **lane detection, obstacle recognition, and adaptive motor control** into a compact prototype car.  

Unlike black-box implementations, every part of this project—from preprocessing frames to PID tuning—was carefully hand-crafted, tested, and optimized for embedded performance.

---

## 🚀 Project Overview

The car uses:

- **PiCamera2** to capture real-time frames  
- **OpenCV** for edge detection and Hough Line Transform to identify lane boundaries  
- **YOLOv8 (OpenVINO optimized)** for real-time object detection with reduced inference time (from ~400 ms to ~30–50 ms)  
- **PID controllers**:
  - One for lane centering  
  - Another for obstacle distance handling  
- **GPIO-controlled DC motors** via `gpiozero` for precise speed adjustment  

The result is a system that can:  
✔ Stay centered in its lane  
✔ Detect and highlight obstacles  
✔ Stop before collisions and resume when the lane is clear  

---

## 🛠️ Features

- **Lane Detection**
  - Gaussian blur, grayscale conversion, and Canny edge detection
  - Hough Transform to detect lane lines
  - Dynamic ROI adjustment for robustness  

- **Obstacle Detection**
  - YOLOv8 with OpenVINO export for embedded inference
  - Identifies in-lane obstacles and ignores irrelevant ones  

- **PID-Controlled Motion**
  - Fine-tuned proportional–integral–derivative control for smooth navigation
  - Dual-PID: Lane alignment + obstacle stopping  

- **Embedded Friendly**
  - Runs fully on Raspberry Pi 5
  - Uses OpenVINO for lightweight deep learning deployment  

---

## 📂 Repository Contents

- `ExtractModelFile.py` → YOLOv8 export & OpenVINO integration  
- `FinalCodeForCar.py` → Full integration: lane detection + YOLO + dual-PID + motor control  
- `WithHardThreshold.py` → Alternative version with stricter obstacle thresholds  
- `AIRE475_Project_Report.pdf` → Detailed report with methodology, experiments, and results  

---

## 🖥️ System Architecture

1. **Frame Capture** → PiCamera2  
2. **Preprocessing** → Gaussian blur + grayscale  
3. **Edge Detection** → Canny filter  
4. **ROI & Line Detection** → Hough Transform  
5. **Lane Boundary Estimation** → Select left & right lane lines  
6. **PID Control (Lane)** → Adjust motor speeds for centering  
7. **YOLOv8 Detection** → Find and classify obstacles  
8. **PID Control (Obstacle)** → Adjust speed or stop before collision  
9. **Motor Commands** → GPIO-based motor control  

---

## 📊 Results

- Real-time lane following on a two-lane track  
- Accurate obstacle recognition and stopping behavior  
- Smooth navigation in controlled environments  
- Limitations in low-light or faded-lane scenarios (future improvement: adaptive thresholding)  

---

## 🔧 Hardware Used

- Raspberry Pi 5  
- PiCamera2  
- DC motors + H-Bridge motor driver  
- Power bank for stable supply  
- Car kit base for integration  

---

## 📈 Future Improvements

- Adaptive thresholding for better lane detection in variable lighting  
- Integration with LiDAR/ultrasonic sensors for depth-based obstacle detection  
- Dynamic PID tuning for sharp turns  
- Expansion toward multi-lane or curved road environments  
