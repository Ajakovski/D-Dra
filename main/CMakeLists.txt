idf_component_register(SRCS "main.c" "imu.c" "setpoint.c" "pid.c" "flight_control.c" "status_leds.c"
                    INCLUDE_DIRS "."
                    REQUIRES mqtt esp_wifi nvs_flash json driver freertos lwip)