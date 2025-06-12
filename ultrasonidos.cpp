// ultrasonidos.cpp
//
// Author: David Rodriguez El Bahri
// Date: 2025-06-01
// Descripción:
//   Despega a 1 m, mide la distancia al techo con ultrasonidos,
//   aterriza y desarma si detecta un objeto a menos de 1 m.

/* Compilar (ajusta PKG_CONFIG_PATH si instalaste MAVSDK en /usr/local):
   export PKG_CONFIG_PATH=/usr/local/lib/cmake/mavsdk:$PKG_CONFIG_PATH
   g++ ultrasonidos.cpp -o ultrasonidos \
       -std=c++17 \
       $(pkg-config --cflags --libs mavsdk) \
       -lgpiod -pthread
*/

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

#include <gpiod.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <algorithm>
#include <stdexcept>

using namespace mavsdk;
using namespace std::chrono;
using std::this_thread::sleep_for;

#define CHIP_NAME       "gpiochip4"
#define TRIG_LINE       23
#define ECHO_LINE       24
#define NUM_SAMPLES     5
#define ECHO_TIMEOUT_MS 100  // ms timeout

struct DistanceSensor {
    gpiod_chip* chip;
    gpiod_line* trig;
    gpiod_line* echo;

    DistanceSensor() {
        chip = gpiod_chip_open_by_name(CHIP_NAME);
        if (!chip) throw std::runtime_error("No pude abrir " CHIP_NAME);
        trig = gpiod_chip_get_line(chip, TRIG_LINE);
        echo = gpiod_chip_get_line(chip, ECHO_LINE);
        if (!trig || !echo) throw std::runtime_error("No pude obtener líneas GPIO");
        if (gpiod_line_request_output(trig, "ultra_trig", 0) < 0)
            throw std::runtime_error("Error solicitando TRIG");
        if (gpiod_line_request_both_edges_events(echo, "ultra_echo") < 0)
            throw std::runtime_error("Error solicitando ECHO");
    }

    ~DistanceSensor() {
        gpiod_line_release(trig);
        gpiod_line_release(echo);
        gpiod_chip_close(chip);
    }

    double measure_once() {
        gpiod_line_set_value(trig, 1);
        sleep_for(microseconds(10));
        gpiod_line_set_value(trig, 0);

        struct gpiod_line_event ev;
        timespec tout{ECHO_TIMEOUT_MS/1000, (ECHO_TIMEOUT_MS%1000)*1000000};
        if (gpiod_line_event_wait(echo, &tout) != 1) return -1;
        gpiod_line_event_read(echo, &ev);
        auto t_start = ev.ts;

        if (gpiod_line_event_wait(echo, &tout) != 1) return -1;
        gpiod_line_event_read(echo, &ev);
        auto t_end = ev.ts;

        double dt_us = (t_end.tv_sec - t_start.tv_sec) * 1e6 +
                       (t_end.tv_nsec - t_start.tv_nsec) / 1e3;
        return dt_us * 0.0343 / 2.0; // cm
    }

    double measure_median() {
        std::vector<double> samples;
        for (int i = 0; i < NUM_SAMPLES; ++i) {
            double d = measure_once();
            if (d > 0) samples.push_back(d);
            sleep_for(milliseconds(50));
        }
        if (samples.empty()) return -1;
        std::sort(samples.begin(), samples.end());
        return samples[samples.size()/2];
    }
};

int main() {
    // 1) Instancia MAVSDK como Ground Station
    Mavsdk mavsdk{Mavsdk::Configuration{ComponentType::GroundStation}};

    // 2) Conecta a Pixhawk vía serial
    std::cout << "1) Conectando al sistema...\n";
    if (mavsdk.add_any_connection("serial:///dev/ttyAMA0:921600")
        != ConnectionResult::Success) {
        std::cerr << "ERROR: conexión serial fallida\n";
        return 1;
    }

    // 3) Espera al autopilot
    std::cout << "2) Esperando autopilot...\n";
    auto system_opt = mavsdk.first_autopilot(5.0);
    if (!system_opt.has_value()) {
        std::cerr << "ERROR: timeout esperando autopilot\n";
        return 1;
    }
    auto system = system_opt.value();
    std::cout << "3) Autopilot listo\n";

    // 4) Crea plugins
    Action action{system};
    Telemetry telemetry{system};

    // 5) Configura altitud de takeoff a 1 m
    action.set_takeoff_altitude(1.0);

    // 6) Arma
    std::cout << "4) Armando...\n";
    if (action.arm() != Action::Result::Success) {
        std::cerr << "ERROR: arm failed\n";
        return 1;
    }

    // 7) Despega
    std::cout << "5) Despegando a 1 m...\n";
    if (action.takeoff() != Action::Result::Success) {
        std::cerr << "ERROR: takeoff failed\n";
        return 1;
    }
    sleep_for(seconds(6));
    std::cout << "6) Altura ~1 m alcanzada\n";

    // 8) Prepara sensor
    DistanceSensor sensor;
    std::cout << "7) Sensor listo\n";

    // 9) Bucle de medición y control
    while (true) {
        double dist = sensor.measure_median();
        if (dist < 0) {
            std::cerr << "   [Sensor] ERROR/timeout\n";
        } else {
            std::cout << "   [Sensor] Distancia al techo: " << dist << " cm\n";
            if (dist < 100.0) {
                std::cout << "*** <1 m detectado – Iniciando aterrizaje ***\n";
                std::cout << "8) Aterrizando...\n";
                action.land();
                telemetry.set_rate_landed_state(1.0);
                while (telemetry.landed_state() != Telemetry::LandedState::OnGround) {
                    sleep_for(seconds(1));
                }
                std::cout << "9) Aterrizado\n";
                std::cout << "10) Desarmando...\n";
                action.disarm();
                std::cout << "11) Desarmado. Fin de misión.\n";
                break;
            }
        }
        sleep_for(milliseconds(200));
    }
    return 0;
}

