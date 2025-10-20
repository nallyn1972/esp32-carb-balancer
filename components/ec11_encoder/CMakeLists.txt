idf_component_register(
    SRCS "ec11_encoder.c"
    INCLUDE_DIRS "include"
    REQUIRES "driver" "esp_timer" "lvgl__lvgl"
)