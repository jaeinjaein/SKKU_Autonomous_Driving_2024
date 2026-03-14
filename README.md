# SKKU Autonomous Driving System (2024)

Autonomous driving system developed for the **2024 Gachon University Autonomous Driving Competition**.

This project implements a full autonomous driving pipeline including **lane perception, steering control, driving object detection, and LiDAR-based vehicle detection** on a 1/5-scale electric vehicle platform.

The system integrates deep learning-based lane segmentation with classical control algorithms to enable stable autonomous driving in a real-world track environment.

---

# Demo

📹 **Competition Driving Demo**

Video Link : https://www.youtube.com/watch?v=FwGlec1eLXw&t=10243s

Driving : 46:06 ~ 47:56

Mission(Avoid Vehicles, Traffic Signs, Parking : 2:49:40 ~ 2:53:20

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

Camera -> Lane Segmentation (YOLOv8-seg) -> Lane Geometry Extraction -> Steering Angle Calculation -> Vehicle Control
LiDAR -> Point Clustering -> Object Detection

---

# Hardware Platform
![Hardware Platform](docs/car_platform.png)

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
![Exterinal View](docs/car_exterior.png)

### Internal System
![Internal View](docs/car_interior.png)

---

# Lane Perception

### Model Finetuning

Lane detection is implemented using **YOLOv8-seg**(Finetuning Head layer).
![Head layer](docs/head_layer.png)

The model finetuned to detect two classes:

- left lane
- right lane

Training dataset was labeled using **Roboflow**.

![Labeling Example](docs/labeling_example.png)

Dataset split:

| Train | Validation | Test |
|---|---|---|
| 2672 | 500 | 167 |

| Model | Inferenece Time | mAP | Track invasion count|   
|---|---|---|---|
| yolov8n | 22ms | 90.90% | 1 |
| yolov8s | 48ms | 93.57% | 0 |
| **yolov8m** | **87ms** | **95.13%** | **0** |
| yolov8l | 130ms | 95.26% | 1 |
| yolov8x | 220ms | 96.92% | 1 |

---

# Lane Following Algorithm

The steering value is computed from lane geometry.

### Algorithm Pipeline
![Algorithm Pipeline](docs/steering_architecture.png)

Steps:

1. YOLOv8-seg inference
2. Extract segmentation points
3. Perspective transform (BEV)
4. RANSAC line regression
5. Convert lane angle to steering command

---

# Steering Control Example
![Steering Example](docs/inference_example.png)
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
![Lidar Result](docs/lidar_result.png)

Algorithm steps:

1. Convert polar coordinates to Cartesian
2. Cluster nearby points
3. Estimate object size
4. Classify vehicle

Detected vehicles are marked with bounding boxes.

---

# Driving Track
![Driving_Track](docs/driving_track.png)

---

# Driving Result
- Completed **2 laps**
- **No lane violations**
- Total time: **~70.829 seconds**

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



