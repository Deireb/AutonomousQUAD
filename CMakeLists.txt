cmake_minimum_required(VERSION 3.15)

project(DroneAvoidance LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Werror)

# Optional: standalone ultrasonic ceiling-landing test (requires libgpiod v1.x)
option(BUILD_ULTRASONIC "Build the ultrasonic ceiling-landing test program" OFF)

# Dependencies
find_package(OpenCV REQUIRED)
find_package(MAVSDK REQUIRED CONFIG)

# Main mission: stereo vision + YOLO (OpenCV DNN) obstacle avoidance
add_executable(DroneAvoidance
    main.cpp
    inference.cpp
)

target_link_libraries(DroneAvoidance PRIVATE
    ${OpenCV_LIBS}
    MAVSDK::mavsdk
    pthread
)

if (BUILD_ULTRASONIC)
    add_executable(UltrasonicLanding ultrasonidos.cpp)
    target_link_libraries(UltrasonicLanding PRIVATE
        MAVSDK::mavsdk
        gpiod
        pthread
    )
endif()
