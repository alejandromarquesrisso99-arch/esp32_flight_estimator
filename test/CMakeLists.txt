cmake_minimum_required(VERSION 3.16)
project(flight_estimator_tests CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

enable_testing()

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../libs/matrix/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../libs/ekf/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../drivers/mpu6050/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../src)

# 1. Matrix Engine Unit Tests
add_executable(test_matrix test_matrix.cpp)
add_test(NAME test_matrix COMMAND test_matrix)

# 2. 7-State EKF Unit Tests
add_executable(test_ekf 
    test_ekf.cpp 
    ../libs/ekf/src/ekf.cpp
)
add_test(NAME test_ekf COMMAND test_ekf)

# 3. Telemetry & FDIR Protocol Unit Tests
add_executable(test_telemetry 
    test_telemetry.cpp 
    ../src/fdir_manager.cpp
)
add_test(NAME test_telemetry COMMAND test_telemetry)
