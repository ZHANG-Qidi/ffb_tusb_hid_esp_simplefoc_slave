idf_component_register(
    SRCS "uart.cpp" "interface.cpp" "main.cpp" "espnow_example_main.c" "simple_foc.cpp"
    PRIV_REQUIRES nvs_flash esp_event esp_netif esp_wifi esp_driver_gpio esp_driver_uart
    INCLUDE_DIRS "."
)
target_compile_options(${COMPONENT_LIB} PRIVATE
    -fpermissive
    -Wno-missing-field-initializers
)
