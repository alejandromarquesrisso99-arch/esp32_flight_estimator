if(ESP_PLATFORM)
    idf_component_register(
        SRCS "src/ekf.cpp"
        INCLUDE_DIRS "include"
        PRIV_INCLUDE_DIRS "../../include"
        REQUIRES matrix
    )
else()
    add_library(ekf STATIC src/ekf.cpp)
    target_include_directories(ekf PUBLIC 
        include
        /../../include
    )
    target_link_libraries(ekf PUBLIC matrix)
endif()
