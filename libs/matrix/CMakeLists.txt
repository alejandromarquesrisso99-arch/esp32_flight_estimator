if(ESP_PLATFORM)
    idf_component_register(
        INCLUDE_DIRS "include"
    )
else()
    add_library(matrix INTERFACE)
    target_include_directories(matrix INTERFACE include)
endif()
