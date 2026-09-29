# Autonomous Quadcopter – Stereo Vision Obstacle Avoidance

[![Drone demo](https://img.youtube.com/vi/19D_F1DPRKg/0.jpg)](https://www.youtube.com/shorts/19D_F1DPRKg)

Final Year Project (Bachelor's thesis) – School of Aeronautical Engineering, University of Vigo.
Author: David Rodríguez El Bahri · Supervisor: Pedro Orgeira Crespo.

## Overview

An onboard companion computer controls a quadcopter through MAVSDK in offboard mode. The drone flies a straight
10 m leg at low altitude, detects frontal obstacles with a stereo camera pair and a YOLO model, climbs to fly over
them, and lands at the end of the leg.

Mission (`main.cpp`):

1. Take off and settle at the cruise altitude (0.5 m).
2. Fly forward at 0.5 m/s.
3. On every frame: rectify both images, compute a disparity map (StereoSGBM), run YOLO (`yolo11n.onnx`, OpenCV DNN)
   and estimate the distance to each detection from the disparity.
4. If an obstacle is closer than 1.75 m: stop, climb to 1.0 m, and keep flying forward at that altitude until no
   obstacle has been detected for 3 s. Then descend back to cruise altitude.
5. After 10 m of travel: land and disarm.

State machine: `FLY → ASCEND → HOLD → DESCEND → FLY … → LAND → FINISHED`.

## Repository contents

| File | Description |
|------|-------------|
| `main.cpp` | Main mission: forward flight with stereo-vision obstacle avoidance and landing. |
| `inference.cpp`, `inference.h` | YOLO inference wrapper (OpenCV DNN, ONNX model, COCO classes). |
| `yolo11n.onnx` | YOLO11n model (COCO, 640×640 input). |
| `stereoMap.xml` | Stereo rectification maps and `Q` matrix for the camera pair. |
| `ultrasonidos.cpp` | Standalone test: take off to 1 m and land when an ultrasonic sensor measures less than 1 m. |

### Ultrasonic sensor

`ultrasonidos.cpp` was developed and tested separately, but it is **not part of the final mission**: at the test site
the grass was too tall and irregular for the ultrasonic sensor to be used. It is kept as a reference and is not built
by default.

## Hardware

- Raspberry Pi 5 (companion computer)
- 2 × IMX219 camera modules (stereo pair, 640×480)
- Pixhawk flight controller, connected over UART (`/dev/ttyAMA0`, 921600 baud)
- Ultrasonic trigger/echo sensor on GPIO 23/24 (only for `ultrasonidos.cpp`)

## Dependencies

- C++17 compiler, CMake ≥ 3.15
- OpenCV ≥ 4.11, built with GStreamer support (`libcamerasrc`)
- MAVSDK (Action, Telemetry and Offboard plugins)
- libgpiod v1.x – only for the optional ultrasonic program

## Build

```bash
git clone https://github.com/Deireb/AutonomousQUAD.git
cd AutonomousQUAD
mkdir build && cd build
cmake ..                          # add -DBUILD_ULTRASONIC=ON to also build the ultrasonic test
make
```

## Run

Run from the directory that contains `yolo11n.onnx`:

```bash
./DroneAvoidance [calibration_file] [connection_url]
```

| Argument | Default |
|----------|---------|
| `calibration_file` | `stereoMap.xml` |
| `connection_url` | `serial:///dev/ttyAMA0:921600` |

Mission parameters (altitudes, speed, detection threshold, distance) are constants at the top of `main.cpp`.

## Safety behaviour

The program aborts before takeoff if arming or the takeoff command fail. Once airborne it commands a landing if:

- cruise altitude is not reached within 15 s,
- offboard mode cannot be started,
- 5 consecutive camera reads fail,
- an exception is thrown inside the mission loop.

Always fly with an RC transmitter ready to take manual control.

## Limitations

- Only objects belonging to the 80 COCO classes are detected. Other obstacles (walls, posts, boxes…) are ignored.
- The forward leg is commanded as a velocity in the NED frame (north), so the vehicle must be placed facing north
  before takeoff for the camera to look along the flight path.
- Depth is computed as `K_DISPARITY / disparity`, where `K_DISPARITY = 72.1` was determined experimentally for this
  camera pair.
- Travelled distance is computed from the autopilot's global position estimate, so a position fix is required.

## License

MIT – see [LICENSE](LICENSE).
