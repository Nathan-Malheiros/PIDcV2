idf_component_register(
    SRCS
        "main.c"
        "touch_screen.c"
        "kinematics.c"
        "pid.c"
        "steppers.c"
        "control.c"
        "cal_store.c"
    INCLUDE_DIRS "."
    REQUIRES
        esp_driver_gpio
        esp_driver_uart
        esp_adc
        esp_timer
        nvs_flash
)
