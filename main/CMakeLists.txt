idf_component_register(
    SRCS
        "main.c"
        "touch_screen.c"
    INCLUDE_DIRS "."
    REQUIRES
        esp_driver_gpio
        esp_adc
)
