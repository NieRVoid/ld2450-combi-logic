idf_component_register(
    SRCS "ld2450_cl_main.c"
    INCLUDE_DIRS ""
    REQUIRES console nvs_flash driver ld2450_driver people_counter occupancy_manager
    PRIV_REQUIRES spi_flash
)
