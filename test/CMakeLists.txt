cmake_minimum_required(VERSION 3.16)
project(flight_estimator_tests CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../libs/matrix/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../libs/ekf/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../drivers/mpu6050/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../src)

add_executable(test_telemetry 
    test_telemetry.cpp 
    ../src/fdir_manager.cpp
)
