cmake_minimum_required(VERSION 3.15)

project(DroneAvoidance LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_definitions(-Wall -Wextra -Werror)

# Dependencias
find_package(OpenCV REQUIRED)
find_package(MAVSDK REQUIRED CONFIG)

find_path(ONNXRUNTIME_INCLUDE_DIR
    NAMES onnxruntime_cxx_api.h
    PATHS /usr/local/include /usr/include
)
find_library(ONNXRUNTIME_LIB
    NAMES onnxruntime
    PATHS /usr/local/lib /usr/local/lib64
)
if (NOT ONNXRUNTIME_INCLUDE_DIR OR NOT ONNXRUNTIME_LIB)
    message(FATAL_ERROR "ONNX Runtime no encontrado. Instala libonnxruntime y headers.")
endif()

add_executable(DroneAvoidance
    main.cpp
    inference.cpp
#   ultrasonidos.cpp 
)

target_include_directories(DroneAvoidance PRIVATE
    ${ONNXRUNTIME_INCLUDE_DIR}
)

target_link_libraries(DroneAvoidance PRIVATE
    ${OpenCV_LIBS}
    MAVSDK::mavsdk
    ${ONNXRUNTIME_LIB}
    gpiod
    pthread
)
