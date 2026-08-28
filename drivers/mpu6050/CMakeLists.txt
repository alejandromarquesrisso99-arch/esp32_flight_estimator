if(ESP_PLATFORM)
    idf_component_register(
        SRCS "src/mpu6050_driver.cpp"
        INCLUDE_DIRS "include"
        PRIV_INCLUDE_DIRS "../../include"
        REQUIRES driver esp_driver_i2c freertos
    )
else()
    add_library(mpu6050 STATIC src/mpu6050_driver.cpp)
    target_include_directories(mpu6050 PUBLIC include ../../include)
endif()
