/* 
Filename: main.cpp
Author: David Rodriguez El Bahri
Date: 01/06/2025
University: University of Vigo
Faculty: School of Aeronautical Engineering
Project: Final Year Project
Supervisor: Pedro Orgeira Crespo
Description: main mission where the drone flies forward, avoids frontal obstacles by changing altitude, and lands after covering a set distance.
*/

#include <iostream>
#include <cmath>
#include <chrono>
#include <thread>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>
#include "inference.h"
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

using namespace mavsdk;
using namespace std;
using namespace std::chrono;

// Mission parameters
static constexpr float CRUISE_ALT_M        = 0.5f;  
static constexpr float OBSTACLE_ALT_M      = CRUISE_ALT_M + 0.5f; 
static constexpr float FORWARD_SPEED_M_S   = 0.5f;    
static constexpr float DETECT_THRESHOLD_M  = 1.75f;   
static constexpr float MISSION_DISTANCE_M  = 10.0f;   
static constexpr float ALT_TOL             = 0.05f;   
static constexpr auto  LOOP_DELAY          = 100ms;

// Experimental constant
static constexpr float K_DISPARITY = 72.1f;

enum class State { FLY, ASCEND, HOLD, DESCEND, LAND, FINISHED };

// Compute distance between two GPS coords (meters)
double haversine(double lat1, double lon1, double lat2, double lon2) {
    static constexpr double R = 6371000.0;
    double dLat = (lat2 - lat1) * M_PI / 180.0;
    double dLon = (lon2 - lon1) * M_PI / 180.0;
    lat1 *= M_PI/180.0; lat2 *= M_PI/180.0;
    double a = sin(dLat/2)*sin(dLat/2)
             + cos(lat1)*cos(lat2)*sin(dLon/2)*sin(dLon/2);
    return R * 2 * atan2(sqrt(a), sqrt(1-a));
}

// Load stereo rectification maps
bool loadCalibration(const string& file,
                     cv::Mat& Lx, cv::Mat& Ly,
                     cv::Mat& Rx, cv::Mat& Ry,
                     cv::Mat& Q) {
    cv::FileStorage fs(file, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        cerr << "[ERROR] No se pudo abrir " << file << endl;
        return false;
    }
    fs["stereoMapL_x"] >> Lx;
    fs["stereoMapL_y"] >> Ly;
    fs["stereoMapR_x"] >> Rx;
    fs["stereoMapR_y"] >> Ry;
    fs["Q"]            >> Q;
    return !(Lx.empty() || Ly.empty() || Rx.empty() || Ry.empty() || Q.empty());
}

// Determine closest obstacle distance from detections
float detect_obstacle(const vector<Detection>& dets, const cv::Mat& disp32) {
    float min_dist = 9999.0f;
    for (const auto& d : dets) {
        int cx = d.box.x + d.box.width/2;
        int cy = d.box.y + d.box.height/2;
        if (cx>=0 && cx<disp32.cols && cy>=0 && cy<disp32.rows) {
            float disp = disp32.at<float>(cy, cx);
            if (disp > 0.1f) {
                float dist = K_DISPARITY / disp;
                min_dist = min(min_dist, dist);
            }
        }
    }
    return min_dist;
}

int main(int argc, char** argv) {
    // Load calibration
    cv::Mat Lx, Ly, Rx, Ry, Q;
    string calibFile = (argc > 1 ? argv[1] : "stereoMap.xml");
    if (!loadCalibration(calibFile, Lx, Ly, Rx, Ry, Q)) {
        return -1;
    }

    // Open cameras via GStreamer
    setenv("GST_DEBUG", "ERROR", 1);
    setenv("LIBCAMERA_LOG_LEVEL", "ERROR", 1);
    const string CAM_L = "/base/axi/pcie@1000120000/rp1/i2c@80000/imx219@10";
    const string CAM_R = "/base/axi/pcie@1000120000/rp1/i2c@88000/imx219@10";
    string pipeL = "libcamerasrc camera-name=\"" + CAM_L + "\" ! "
                   "video/x-raw,width=640,height=480,format=RGB ! "
                   "videoconvert ! video/x-raw,format=BGR ! appsink drop=true sync=false";
    string pipeR = "libcamerasrc camera-name=\"" + CAM_R + "\" ! "
                   "video/x-raw,width=640,height=480,format=RGB ! "
                   "videoconvert ! video/x-raw,format=BGR ! appsink drop=true sync=false";
    cv::VideoCapture capL(pipeL, cv::CAP_GSTREAMER),
                      capR(pipeR, cv::CAP_GSTREAMER);
    if (!capL.isOpened() || !capR.isOpened()) {
        cerr << "[ERROR] No se pudieron abrir las cámaras." << endl;
        return -1;
    }

    // Connect to autopilot
    Mavsdk mavsdk{Mavsdk::Configuration{ComponentType::GroundStation}};
    string connection_url = (argc > 2
        ? argv[2]
        : string("serial:///dev/ttyAMA0:921600"));
    if (mavsdk.add_any_connection(connection_url) != ConnectionResult::Success) {
        cerr << "[ERROR] Conexión fallida: " << connection_url << endl;
        return -1;
    }
    auto systemOpt = mavsdk.first_autopilot(10.0);
    if (!systemOpt.has_value()) {
        cerr << "[ERROR] Timeout esperando autopilot" << endl;
        return -1;
    }
    auto system = systemOpt.value();
    Action action{system};
    Telemetry telemetry{system};
    Offboard offboard{system};

    // Wait until drone health is OK
    while (!telemetry.health_all_ok()) {
        this_thread::sleep_for(1s);
    }

    // Take off to cruise altitude
    action.set_takeoff_altitude(CRUISE_ALT_M);
    action.arm();
    action.takeoff();
    while (telemetry.position().relative_altitude_m < CRUISE_ALT_M - 0.1f) {
        this_thread::sleep_for(100ms);
    }

    // Start offboard control
    offboard.set_velocity_ned({0,0,0, telemetry.attitude_euler().yaw_deg});
    offboard.start();

    // Adjust exactly to cruise altitude
    {
        const float Kp_init = 1.0f;
        while (fabs(telemetry.position().relative_altitude_m - CRUISE_ALT_M) > ALT_TOL) {
            float alt_error = CRUISE_ALT_M - telemetry.position().relative_altitude_m;
            float down_speed = -Kp_init * alt_error;
            down_speed = clamp(down_speed, -0.5f, 0.5f);
            offboard.set_velocity_ned({0,0,down_speed, telemetry.attitude_euler().yaw_deg});
            this_thread::sleep_for(100ms);
        }
        offboard.set_velocity_ned({0,0,0, telemetry.attitude_euler().yaw_deg});
        cout << "[INFO] Altitud de crucero ajustada a "
             << CRUISE_ALT_M << " m ± " << ALT_TOL << " m.\n";
    }

    // Initialize vision and state
    Inference inf("yolo11n.onnx", cv::Size(640,640), "", false);
    double startLat = telemetry.position().latitude_deg;
    double startLon = telemetry.position().longitude_deg;
    State state = State::FLY;
    steady_clock::time_point hold_start{};
    auto stereo = cv::StereoSGBM::create(
        0, 16*8, 5, 8*3*5*5, 32*3*5*5,
        1, 10, 100, 2, 31,
        cv::StereoSGBM::MODE_SGBM_3WAY
    );

    // Main mission loop
    while (state != State::FINISHED) {
        auto pos = telemetry.position();
        double travelled = haversine(startLat, startLon, pos.latitude_deg, pos.longitude_deg);

        // Capture and rectify frames, compute disparity
        cv::Mat fL, fR, rL, rR, grayL, grayR, disp16, disp32;
        capL.read(fL); capR.read(fR);
        cv::remap(fL, rL, Lx, Ly, cv::INTER_LINEAR);
        cv::remap(fR, rR, Rx, Ry, cv::INTER_LINEAR);
        cv::cvtColor(rL, grayL, cv::COLOR_BGR2GRAY);
        cv::cvtColor(rR, grayR, cv::COLOR_BGR2GRAY);
        stereo->compute(grayL, grayR, disp16);
        disp16.convertTo(disp32, CV_32F, 1.0/16.0f);

        auto dets = inf.runInference(rR);
        float min_dist = detect_obstacle(dets, disp32);
        
        // State machine for flight, avoidance, and landing
        switch (state) {
            case State::FLY: {
                if (min_dist < DETECT_THRESHOLD_M) {
                    cout << "[DETECCIÓN] Objeto a " << min_dist
                         << " m. Parando y subiendo a " << OBSTACLE_ALT_M << " m.\n";
                    offboard.set_velocity_ned({0,0,0, telemetry.attitude_euler().yaw_deg});
                    state = State::ASCEND;
                } else if (pos.relative_altitude_m > CRUISE_ALT_M + ALT_TOL) {
                    cout << "[INFO] Demasiado alto, descendiendo a crucero.\n";
                    state = State::DESCEND;
                } else if (travelled >= MISSION_DISTANCE_M) {
                    cout << "[INFO] Misión completada, aterrizando.\n";
                    offboard.stop();
                    action.land();
                    state = State::LAND;
                } else {
                    float alt_error = CRUISE_ALT_M - pos.relative_altitude_m;
                    float down_speed = (fabs(alt_error) > ALT_TOL)
                                       ? clamp(-1.0f * alt_error, -0.5f, 0.5f)
                                       : 0.0f;
                    Offboard::VelocityNedYaw cmd{};
                    cmd.north_m_s = FORWARD_SPEED_M_S;
                    cmd.east_m_s  = 0.0f;
                    cmd.down_m_s  = down_speed;
                    cmd.yaw_deg   = telemetry.attitude_euler().yaw_deg;
                    offboard.set_velocity_ned(cmd);
                }
                break;
            }
            case State::ASCEND: {
                if (pos.relative_altitude_m < OBSTACLE_ALT_M - ALT_TOL) {
                    offboard.set_velocity_ned({0,0,-0.4f, telemetry.attitude_euler().yaw_deg});
                } else {
                    // Llegado a altitud de evasión: iniciar HOLD
                    hold_start = steady_clock::now();
                    cout << "[INFO] Altitud de evasión alcanzada. Manteniendo durante 3s.\n";
                    state = State::HOLD;
                }
                break;
            }
            case State::HOLD: {
                auto now = steady_clock::now();
                // si sigue detectando, resetear temporizador
                if (min_dist < DETECT_THRESHOLD_M) {
                    hold_start = now;
                }
                auto elapsed = duration_cast<seconds>(now - hold_start).count();
                if (elapsed < 3) {
                    // mantener altitud de evasión
                    float alt_error = OBSTACLE_ALT_M - pos.relative_altitude_m;
                    float down_speed = clamp(-1.0f * alt_error, -0.5f, 0.5f);
                    offboard.set_velocity_ned({FORWARD_SPEED_M_S, 0, down_speed, telemetry.attitude_euler().yaw_deg});
                } else {
                    cout << "[INFO] Hold finalizado. Reanudando crucero.\n";
                    state = State::FLY;
                }
                break;
            }
            case State::DESCEND: {
                if (pos.relative_altitude_m > CRUISE_ALT_M + ALT_TOL) {
                    offboard.set_velocity_ned({0,0,0.3f, telemetry.attitude_euler().yaw_deg});
                } else {
                    offboard.set_velocity_ned({0,0,0, telemetry.attitude_euler().yaw_deg});
                    cout << "[INFO] Altura de crucero restaurada.\n";
                    state = State::FLY;
                }
                break;
            }
            case State::LAND:
                while (telemetry.in_air()) {
                    this_thread::sleep_for(500ms);
                }
                action.disarm();
                state = State::FINISHED;
                break;
            default:
                break;
        }

        this_thread::sleep_for(LOOP_DELAY);
    }

    cout << "[INFO] Secuencia terminada\n";
    return 0;
}
