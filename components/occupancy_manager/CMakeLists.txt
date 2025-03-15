# components/occupancy_manager/CMakeLists.txt

idf_component_register(
    SRCS
        "src/occupancy_manager.c"
    INCLUDE_DIRS
        "include"
    PRIV_INCLUDE_DIRS
        "src"
    REQUIRES
        driver
        log
        freertos
        esp_event
        esp_common
        esp_timer
)