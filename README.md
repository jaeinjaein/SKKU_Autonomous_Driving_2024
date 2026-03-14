# SKKU Autonomous Driving System (2024)

Autonomous driving system developed for the **2024 Gachon University Autonomous Driving Competition**.

This project implements a full autonomous driving pipeline including **lane perception, steering control, and LiDAR-based vehicle detection** on a 1/5-scale electric vehicle platform.

The system integrates deep learning-based lane segmentation with classical control algorithms to enable stable autonomous driving in a real-world track environment.

---

# Demo

📹 **Competition Driving Demo**

[VIDEO LINK HERE]

<!-- 
추천:
YouTube embed gif 또는 thumbnail

예:
https://youtu.be/xxxxx
-->

---

# System Overview

<!--
여기에 시스템 아키텍처 그림 넣기
perception → control pipeline diagram

추천 파일명
docs/system_architecture.png
-->

![System Architecture](docs/system_architecture.png)

The autonomous driving system follows a robotics pipeline:

Camera
↓
Lane Segmentation (YOLOv8-seg)
↓
Lane Geometry Extraction
↓
Steering Angle Calculation
↓
Vehicle Controller
LiDAR
↓
Point Clustering
↓
Object Detection


---

# Hardware Platform
TODO : Vehicle System Overview Photo

| Component | Device |
|---|---|
| Computing | Macbook Pro M2 |
| Controller | Arduino Mega 2560 |
| Camera | Logitech C920 |
| LiDAR | RPLiDAR A1M8-R6 |
| Motor Driver | SZH-GNP521 |

Sensors are connected to the HPC via USB and the controller via serial communication.  
The controller runs motor control using **RTOS-based parallel processing**.

---

# Vehicle Hardware

### External View

TODO : Vehicle External View Photo

### Internal System

TODO : Vehicle Internal Control System Photo

---

# Lane Perception

Lane detection is implemented using **YOLOv8-seg**.

The model detects two classes:

- left lane
- right lane

### Model Inference Example

TODO : Roboflow dataset labeling screenshot

Training dataset was labeled using **Roboflow** and finetuned from a COCO pretrained model.

Dataset split:

| Train | Validation | Test |
|---|---|---|
| 2672 | 500 | 167 |

---

# Lane Following Algorithm

The steering value is computed from lane geometry.

### Algorithm Pipeline

TODO : Algorithm Photo

Steps:

1. YOLOv8-seg inference
2. Extract segmentation points
3. Perspective transform
4. RANSAC line regression
5. Convert lane angle to steering command

---

# Steering Control Example

TODO : Steering result photo

The system calculates the steering angle from lane orientation and maps it into **40 discrete steering steps**.

---

# Straight Driving Correction

To prevent accumulated error during long straight driving, a correction algorithm was implemented.

Correction equation: D_calib = (M_set - M_img) / 30

Final steering value : D_steer = D_line + D_calib


---

# LiDAR Vehicle Detection

Vehicle detection is implemented using LiDAR clustering.

### LiDAR Clustering Result

TODO : Lidar clustering result photo

Algorithm steps:

1. Convert polar coordinates to Cartesian
2. Cluster nearby points
3. Estimate object size
4. Classify vehicle

Detected vehicles are marked with bounding boxes.

---

# Driving Track

TODO : Driving Track Image

---

# Driving Result

TODO : Driving Result Video

Result:

- Completed **2 laps**
- **No lane violations**
- Total time: **~70 seconds**

---

# Project Structure
SKKU_Autonomous_Driving_2024
perception/
lane_segmentation
yolov8_model
control/
steering_control
lane_following
lidar/
lidar_clustering
vehicle_detection
controller/
serial_communication
motor_control

---

# Key Contributions

- Built a **complete autonomous driving pipeline**
- Implemented **YOLOv8 segmentation based lane detection**
- Designed **lane geometry steering algorithm**
- Developed **straight driving correction algorithm**
- Implemented **LiDAR clustering vehicle detection**
- Validated system in real-world competition environment

---

# Future Work

Future improvements:

- vision + LiDAR sensor fusion
- obstacle avoidance
- trajectory planning
- deployment on full-scale vehicles

---

# Author

Jaein Lee  
Sungkyunkwan University  
Electrical and Electronic Engineering



