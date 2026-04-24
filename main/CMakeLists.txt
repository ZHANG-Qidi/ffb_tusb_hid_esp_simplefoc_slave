idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_gpio esp_driver_uart
    PRIV_REQUIRES esp_driver_gpio
)
