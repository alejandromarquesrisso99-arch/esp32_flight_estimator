cmake_minimum_required(VERSION 3.16)

# Register custom component directories
set(EXTRA_COMPONENT_DIRS 
    "drivers/mpu6050"
    "libs/matrix"
    "libs/ekf"
    "src"
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp32_flight_estimator)
