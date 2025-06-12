# Drone Obstacle Detection & Landing

[![Drone demo](https://img.youtube.com/vi/19D_F1DPRKg/0.jpg)](https://www.youtube.com/shorts/19D_F1DPRKg)

## Overview
This repository contains three C++ programs demonstrating autonomous drone behaviors using MAVSDK and various sensors:
1. **Stereo Vision Landing**  
   - Uses a stereo camera and YOLO deep-learning model to detect obstacles within 1.5 m and land.
2. **Ultrasonic Ceiling Landing**  
   - Employs a top-facing ultrasonic sensor to measure distance to ceiling and land when below 1 m.
3. **Forward Flight with Obstacle Avoidance**  
   - Executes a 10 m forward mission, maintains 0.5 m cruise altitude, and climbs 0.5 m to avoid frontal obstacles detected via stereo depth.

## Prerequisites
- **MAVSDK** (Core, Action, Telemetry, Offboard plugins)  
- **OpenCV** (>= 4.11) & GStreamer
- **libgpiod** (for ultrasonic sensor)  
- C++17 compiler, CMake

## Building
```bash
git clone https://github.com/Deireb/AutonomousQUAD.git
cd AutonomousQUAD
mkdir build && cd build
cmake ..
make

