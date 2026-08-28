if(ESP_PLATFORM)
    idf_component_register(
        SRCS 
            "src/mpu6050_driver.cpp"
            "src/drdy_sync.cpp"
        INCLUDE_DIRS "include"
        PRIV_INCLUDE_DIRS "../../include"
        REQUIRES driver esp_driver_i2c esp_driver_gpio freertos esp_hw_support
    )
else()
    add_library(mpu6050 STATIC src/mpu6050_driver.cpp src/drdy_sync.cpp)
    target_include_directories(mpu6050 PUBLIC include ../../include)
endif()
