# components/people_counter/CMakeLists.txt

idf_component_register(
    SRCS
        "src/people_counter.c"
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
        ld2450_driver
)
