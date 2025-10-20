/**
 * @file main.c
 * @brief ESP32-S3 Carburetor Balancing Tool - Main Application
 * 
 * This application provides a digital carburetor balancing tool with:
 * - ILI9341 240x320 LCD display via SPI
 * - LVGL GUI framework (v9.x) for user interface
 * - EC11 rotary encoder with button for navigation
 * - Multiple vacuum gauge inputs for carburetor synchronization
 * - PWM backlight control
 * - Real-time vacuum readings and differential display
 * 
 * Hardware: ESP32-S3 with custom PCB for carburetor balancing.
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_ili9341.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/ledc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_lvgl_port.h"
#include "lvgl.h"
#include "ec11_encoder.h"

#include "hardware_config.h"

static const char *TAG = "CARB_BALANCER";

// LCD and LVGL handles
static esp_lcd_panel_io_handle_t lcd_io = NULL;
static esp_lcd_panel_handle_t lcd_panel = NULL;
static lv_display_t *lvgl_disp = NULL;
static lv_indev_t *lvgl_encoder_indev = NULL;
static lv_group_t *default_group = NULL;

// =============================================================================
// LCD Initialization
// =============================================================================

static esp_err_t lcd_init(void)
{
    esp_err_t ret = ESP_OK;

    ESP_LOGI(TAG, "Initialize SPI bus");
    const spi_bus_config_t bus_config = {
        .sclk_io_num = LCD_PIN_NUM_SCLK,
        .mosi_io_num = LCD_PIN_NUM_MOSI,
        .miso_io_num = LCD_PIN_NUM_MISO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_H_RES * 50 * sizeof(uint16_t),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &bus_config, SPI_DMA_CH_AUTO));

    ESP_LOGI(TAG, "Install panel IO");
    const esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = LCD_PIN_NUM_LCD_DC,
        .cs_gpio_num = LCD_PIN_NUM_LCD_CS,
        .pclk_hz = LCD_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits = LCD_CMD_BITS,
        .lcd_param_bits = LCD_PARAM_BITS,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &lcd_io));

    ESP_LOGI(TAG, "Install ILI9341 panel driver");
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = LCD_PIN_NUM_LCD_RST,
        .color_space = ESP_LCD_COLOR_SPACE_BGR,
        .bits_per_pixel = LCD_BITS_PER_PIXEL,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_ili9341(lcd_io, &panel_config, &lcd_panel));

    ESP_LOGI(TAG, "Initialize LCD panel");
    ESP_ERROR_CHECK(esp_lcd_panel_reset(lcd_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(lcd_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(lcd_panel, false, true));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(lcd_panel, true));

    ESP_LOGI(TAG, "LCD initialization complete");
    return ret;
}

// =============================================================================
// Backlight Initialization
// =============================================================================

static esp_err_t backlight_init(void)
{
    ESP_LOGI(TAG, "Initialize backlight (PWM)");
    
    ledc_timer_config_t ledc_timer = {
        .speed_mode = BK_LIGHT_MODE,
        .duty_resolution = BK_LIGHT_DUTY_RES,
        .timer_num = BK_LIGHT_TIMER,
        .freq_hz = BK_LIGHT_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel = {
        .speed_mode = BK_LIGHT_MODE,
        .channel = BK_LIGHT_CHANNEL,
        .timer_sel = BK_LIGHT_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = BK_LIGHT_OUTPUT_IO,
        .duty = BK_LIGHT_DUTY,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    ESP_LOGI(TAG, "Backlight initialized at %d%% brightness", 
             (int)((BK_LIGHT_DUTY * 100) / BK_LIGHT_MAX_DUTY));
    return ESP_OK;
}

// =============================================================================
// ADC Initialization for Vacuum Sensors
// =============================================================================

// ADC oneshot unit handle and calibration handle
static adc_oneshot_unit_handle_t adc1_handle;
static adc_cali_handle_t adc1_cali_chan0_handle = NULL;
static adc_cali_handle_t adc1_cali_chan6_handle = NULL;
static adc_cali_handle_t adc1_cali_chan7_handle = NULL;
static adc_cali_handle_t adc1_cali_chan8_handle = NULL;

// ADC channel array for easy iteration
static const adc_channel_t vacuum_channels[4] = {
    ADC_CHANNEL_0,
    ADC_CHANNEL_6,
    ADC_CHANNEL_7,
    ADC_CHANNEL_8
};

static esp_err_t adc_init(void)
{
    ESP_LOGI(TAG, "Initialize ADC for vacuum sensors");
    
    // Initialize ADC oneshot
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    // Configure ADC channels
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };
    
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_0, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_6, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_7, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_8, &config));

    // Initialize ADC calibration for each channel
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };

    esp_err_t ret = adc_cali_create_scheme_curve_fitting(&cali_config, &adc1_cali_chan0_handle);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration Success for Channel 0");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !adc1_cali_chan0_handle) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration for Channel 0");
    }

    ret = adc_cali_create_scheme_curve_fitting(&cali_config, &adc1_cali_chan6_handle);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration Success for Channel 6");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !adc1_cali_chan6_handle) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration for Channel 6");
    }

    ret = adc_cali_create_scheme_curve_fitting(&cali_config, &adc1_cali_chan7_handle);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration Success for Channel 7");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !adc1_cali_chan7_handle) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration for Channel 7");
    }

    ret = adc_cali_create_scheme_curve_fitting(&cali_config, &adc1_cali_chan8_handle);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration Success for Channel 8");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !adc1_cali_chan8_handle) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration for Channel 8");
    }

    ESP_LOGI(TAG, "ADC initialization complete");
    return ESP_OK;
}

// Read vacuum sensor value and convert to cmHg
static float read_vacuum_sensor(int sensor_index)
{
    if (sensor_index < 0 || sensor_index >= 4) {
        return 0.0f;
    }
    
    int adc_raw = 0;
    int voltage = 0;
    adc_cali_handle_t cali_handle = NULL;
    
    // Select calibration handle based on sensor index
    switch (sensor_index) {
        case 0: cali_handle = adc1_cali_chan0_handle; break;
        case 1: cali_handle = adc1_cali_chan6_handle; break;
        case 2: cali_handle = adc1_cali_chan7_handle; break;
        case 3: cali_handle = adc1_cali_chan8_handle; break;
    }

    // Take multiple samples and average
    uint32_t adc_reading = 0;
    for (int i = 0; i < VACUUM_ADC_SAMPLES; i++) {
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, vacuum_channels[sensor_index], &adc_raw));
        adc_reading += adc_raw;
    }
    adc_reading /= VACUUM_ADC_SAMPLES;

    // Convert ADC reading to voltage in mV
    if (cali_handle) {
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(cali_handle, adc_reading, &voltage));
    } else {
        // Fallback: rough conversion without calibration
        voltage = adc_reading * 3300 / 4095;
    }

    // Convert voltage to vacuum pressure (cmHg)
    // Linear interpolation between sensor min/max points
    float vacuum_cmhg = 0.0f;
    
    if (voltage >= VACUUM_SENSOR_MIN_MV && voltage <= VACUUM_SENSOR_MAX_MV) {
        // Linear mapping: voltage -> vacuum pressure
        vacuum_cmhg = (float)(voltage - VACUUM_SENSOR_MIN_MV) * 
                      (VACUUM_SENSOR_MAX_CMHG - VACUUM_SENSOR_MIN_CMHG) / 
                      (VACUUM_SENSOR_MAX_MV - VACUUM_SENSOR_MIN_MV) + 
                      VACUUM_SENSOR_MIN_CMHG;
    } else if (voltage < VACUUM_SENSOR_MIN_MV) {
        vacuum_cmhg = VACUUM_SENSOR_MIN_CMHG;
    } else {
        vacuum_cmhg = VACUUM_SENSOR_MAX_CMHG;
    }
    
    return vacuum_cmhg;
}

// Read all vacuum sensors
static void read_all_vacuum_sensors(float *readings)
{
    for (int i = 0; i < 4; i++) {
        readings[i] = read_vacuum_sensor(i);
    }
}

// =============================================================================
// Encoder Initialization using EC11 Component
// =============================================================================

static esp_err_t encoder_init(void)
{
    ESP_LOGI(TAG, "Initialize EC11 encoder component");
    
    // Configure encoder
    ec11_encoder_config_t encoder_config = {
        .gpio_a = EC11_GPIO_A,
        .gpio_b = EC11_GPIO_B,
        .gpio_button = EC11_GPIO_BUTTON,
        .button_active_low = (BUTTON_ACTIVE_LEVEL == 0),
        .debounce_ms = 8
    };
    
    // Initialize encoder
    ESP_ERROR_CHECK(ec11_encoder_init(&encoder_config));
    
    // Create LVGL input device for encoder
    lv_indev_t *encoder_indev = ec11_encoder_create_lvgl_indev();
    if (encoder_indev == NULL) {
        ESP_LOGE(TAG, "Failed to create encoder LVGL input device");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "EC11 encoder initialized successfully");
    return ESP_OK;
}

// =============================================================================
// LVGL Initialization
// =============================================================================

static esp_err_t lvgl_init(void)
{
    ESP_LOGI(TAG, "Initialize LVGL port");
    const lvgl_port_cfg_t lvgl_cfg = {
        .task_priority = LVGL_TASK_PRIORITY,
        .task_stack = LVGL_TASK_STACK_SIZE,
        .task_affinity = -1,
        .task_max_sleep_ms = LVGL_TASK_MAX_DELAY_MS,
        .timer_period_ms = LVGL_TICK_PERIOD_MS,
    };
    ESP_ERROR_CHECK(lvgl_port_init(&lvgl_cfg));

    ESP_LOGI(TAG, "Add LCD display");
    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = lcd_io,
        .panel_handle = lcd_panel,
        .buffer_size = (LCD_H_RES * LCD_V_RES) / 10,
        .double_buffer = false,
        .hres = LCD_H_RES,
        .vres = LCD_V_RES,
        .monochrome = false,
        .color_format = LV_COLOR_FORMAT_RGB565,
        .rotation = {
            .swap_xy = false,
            .mirror_x = true,
            .mirror_y = false,
        },
        .flags = {
            .buff_dma = true,
            .swap_bytes = true,
        }
    };
    lvgl_disp = lvgl_port_add_disp(&disp_cfg);

    // Initialize encoder component
    ESP_ERROR_CHECK(encoder_init());

    // Create LVGL input device for encoder
    ESP_LOGI(TAG, "Create LVGL encoder input device");
    lvgl_encoder_indev = ec11_encoder_create_lvgl_indev();
    if (lvgl_encoder_indev == NULL) {
        ESP_LOGE(TAG, "Failed to create encoder input device");
        return ESP_FAIL;
    }
    
    // Create default group and set as default
    default_group = lv_group_create();
    if (default_group == NULL) {
        ESP_LOGE(TAG, "Failed to create LVGL group");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Created LVGL group: %p", default_group);
    
    lv_group_set_default(default_group);
    lv_indev_set_group(lvgl_encoder_indev, default_group);
    ESP_LOGI(TAG, "Assigned encoder input device to group");
    
    // Configure group to reduce navigation sensitivity
    lv_group_set_wrap(default_group, true);  // Allow wrapping around items

    ESP_LOGI(TAG, "LVGL initialization complete");
    return ESP_OK;
}

// =============================================================================
// Carburetor Balancing UI - Up to 4 Carburetors
// =============================================================================

// Global variables for vacuum readings
static int32_t vacuum_readings[4] = {15, 12, 14, 16}; // Current readings (will be updated by ADC)
static lv_obj_t *vacuum_gauges[4];
static lv_obj_t *vacuum_labels[4];
static lv_obj_t *diff_labels[3]; // Diffs relative to carb 1
static lv_obj_t *mode_label; // Shows current mode (ADC/Manual)
static esp_timer_handle_t vacuum_update_timer;
static QueueHandle_t vacuum_update_queue; // Queue for communicating sensor data from timer to UI task
static bool adc_mode = true; // true = ADC readings, false = manual encoder mode
static int current_gauge_selection = 0; // Which gauge is selected for encoder control

// Structure for vacuum sensor data
typedef struct {
    float readings[4];
} vacuum_data_t;

// =============================================================================
// Encoder Event Handlers
// =============================================================================

// Update slider editability based on mode
static void update_slider_editability(void)
{
    for (int i = 0; i < 4; i++) {
        if (vacuum_gauges[i] != NULL) {
            if (adc_mode) {
                // In ADC mode, make sliders non-interactive
                lv_obj_clear_flag(vacuum_gauges[i], LV_OBJ_FLAG_CLICKABLE);
                lv_obj_set_style_border_width(vacuum_gauges[i], 0, LV_PART_MAIN);
            } else {
                // In manual mode, make sliders interactive
                lv_obj_add_flag(vacuum_gauges[i], LV_OBJ_FLAG_CLICKABLE);
                
                // Highlight the currently selected gauge
                if (i == current_gauge_selection) {
                    lv_obj_set_style_border_width(vacuum_gauges[i], 3, LV_PART_MAIN);
                    lv_obj_set_style_border_color(vacuum_gauges[i], lv_color_hex(0xffff00), LV_PART_MAIN);
                    lv_group_focus_obj(vacuum_gauges[i]);
                } else {
                    lv_obj_set_style_border_width(vacuum_gauges[i], 0, LV_PART_MAIN);
                }
            }
        }
    }
}

// Encoder button callback - cycle between gauges or toggle ADC/manual mode
static void encoder_button_cb(void)
{
    if (adc_mode) {
        // Switch to manual mode
        adc_mode = false;
        current_gauge_selection = 0;
        ESP_LOGI(TAG, "Switched to Manual Mode - Use encoder to adjust Cylinder %d", current_gauge_selection + 1);
        
        // Update mode label
        if (mode_label != NULL) {
            lv_label_set_text_fmt(mode_label, "MANUAL - CYL %d", current_gauge_selection + 1);
            lv_obj_set_style_text_color(mode_label, lv_color_hex(0xff8800), LV_PART_MAIN); // Orange for manual mode
        }
        
        // Update slider states and focus
        update_slider_editability();
    } else {
        // In manual mode, cycle to next gauge or back to ADC mode
        current_gauge_selection++;
        if (current_gauge_selection >= 4) {
            // Cycle back to ADC mode
            adc_mode = true;
            current_gauge_selection = 0;
            ESP_LOGI(TAG, "Switched to ADC Mode - Real-time sensor readings");
            
            // Update mode label
            if (mode_label != NULL) {
                lv_label_set_text(mode_label, "ADC MODE");
                lv_obj_set_style_text_color(mode_label, lv_color_hex(0x00ff00), LV_PART_MAIN); // Green for ADC mode
            }
        } else {
            // Select next gauge
            ESP_LOGI(TAG, "Selected Cylinder %d for manual adjustment", current_gauge_selection + 1);
            
            // Update mode label
            if (mode_label != NULL) {
                lv_label_set_text_fmt(mode_label, "MANUAL - CYL %d", current_gauge_selection + 1);
                lv_obj_set_style_text_color(mode_label, lv_color_hex(0xff8800), LV_PART_MAIN); // Orange for manual mode
            }
        }
        
        // Update slider states and focus
        update_slider_editability();
    }
}

// =============================================================================
// Timer Callbacks
// =============================================================================

// Timer callback to update vacuum readings from ADC
static void vacuum_update_timer_callback(void* arg)
{
    // Only update from ADC if in ADC mode
    if (!adc_mode) {
        return;
    }
    
    vacuum_data_t data;
    
    // Read all vacuum sensors (this is safe to do from timer context)
    read_all_vacuum_sensors(data.readings);
    
    // Send data to UI update task (non-blocking)
    xQueueSendFromISR(vacuum_update_queue, &data, NULL);
}

// UI update task - safely updates LVGL objects from task context
static void ui_update_task(void *pvParameters)
{
    vacuum_data_t data;
    
    while (1) {
        // Wait for sensor data from timer callback
        if (xQueueReceive(vacuum_update_queue, &data, portMAX_DELAY) == pdTRUE) {
            // Lock LVGL mutex before updating UI
            lvgl_port_lock(0);
            
            // Update the vacuum readings array with new values
            for (int i = 0; i < 4; i++) {
                vacuum_readings[i] = (int32_t)data.readings[i];
            }
            
            // Update UI gauges with new readings
            for (int i = 0; i < 4; i++) {
                if (vacuum_gauges[i] != NULL) {
                    lv_slider_set_value(vacuum_gauges[i], vacuum_readings[i], LV_ANIM_ON);
                }
                if (vacuum_labels[i] != NULL) {
                    lv_label_set_text_fmt(vacuum_labels[i], "%ld cmHg", (long)vacuum_readings[i]);
                }
            }
            
            // Update differential labels (relative to cylinder 1)
            for (int i = 0; i < 3; i++) {
                if (diff_labels[i] != NULL) {
                    int32_t diff = vacuum_readings[i + 1] - vacuum_readings[0];
                    lv_label_set_text_fmt(diff_labels[i], "D%d: %+ld cmHg", i + 2, (long)diff);
                    
                    // Color coding for differential values
                    if (abs(diff) <= 1) {
                        lv_obj_set_style_text_color(diff_labels[i], lv_color_hex(0x00ff00), LV_PART_MAIN); // Green - good
                    } else if (abs(diff) <= 3) {
                        lv_obj_set_style_text_color(diff_labels[i], lv_color_hex(0xff8800), LV_PART_MAIN); // Orange - warning
                    } else {
                        lv_obj_set_style_text_color(diff_labels[i], lv_color_hex(0xff0000), LV_PART_MAIN); // Red - bad
                    }
                }
            }
            
            // Unlock LVGL mutex
            lvgl_port_unlock();
        }
    }
}

// Encoder key event callback for mode switching
static void encoder_key_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    
    if (code == LV_EVENT_KEY) {
        uint32_t key = lv_indev_get_key(lv_indev_get_act());
        if (key == LV_KEY_ENTER) {
            // Toggle between ADC and manual modes
            encoder_button_cb();
        }
    }
}

// Vacuum gauge value changed callback
static void vacuum_gauge_event_cb(lv_event_t *e)
{
    // Only allow manual changes when not in ADC mode
    if (adc_mode) {
        return;
    }
    
    lv_obj_t *gauge = lv_event_get_target(e);
    lv_obj_t *label = (lv_obj_t *)lv_event_get_user_data(e);
    int32_t value = lv_slider_get_value(gauge);
    
    // Find which gauge this is and update the reading
    for (int i = 0; i < 4; i++) {
        if (vacuum_gauges[i] == gauge) {
            vacuum_readings[i] = value;
            break;
        }
    }
    
    // Update the label for this gauge
    lv_label_set_text_fmt(label, "%ld cmHg", (long)value);
    
    // Update all differential displays
    for (int i = 0; i < 3; i++) {
        int32_t diff = vacuum_readings[i + 1] - vacuum_readings[0]; // Relative to carb 1
        lv_label_set_text_fmt(diff_labels[i], "D%d: %+ld cmHg", i + 2, (long)diff);
        
        // Color code the differences
        if (abs(diff) <= 2) {
            lv_obj_set_style_text_color(diff_labels[i], lv_color_hex(0x00ff00), LV_PART_MAIN); // Green - good
        } else if (abs(diff) <= 5) {
            lv_obj_set_style_text_color(diff_labels[i], lv_color_hex(0xff8800), LV_PART_MAIN); // Orange - warning
        } else {
            lv_obj_set_style_text_color(diff_labels[i], lv_color_hex(0xff0000), LV_PART_MAIN); // Red - bad
        }
    }
}

static void create_carb_balancer_ui(void)
{
    // Lock LVGL mutex before creating UI
    lvgl_port_lock(0);

    // Create main screen
    lv_obj_t *scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x1a1a1a), LV_PART_MAIN);

    // Title label - top center
    lv_obj_t *title = lv_label_create(scr);
    lv_label_set_text(title, "4-Cylinder Carb Balancer");
    lv_obj_set_style_text_color(title, lv_color_hex(0x00ff00), LV_PART_MAIN);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_18, LV_PART_MAIN);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 5);

    // Mode indicator label - top right
    mode_label = lv_label_create(scr);
    lv_label_set_text(mode_label, "ADC MODE");
    lv_obj_set_style_text_font(mode_label, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_set_style_text_color(mode_label, lv_color_hex(0x00ff00), LV_PART_MAIN); // Green for ADC mode
    lv_obj_align(mode_label, LV_ALIGN_TOP_RIGHT, -5, 5);

    // Status label - below title
    lv_obj_t *status = lv_label_create(scr);
    lv_label_set_text(status, "System Ready - Ref: Cyl 1");
    lv_obj_set_style_text_color(status, lv_color_hex(0xffff00), LV_PART_MAIN);
    lv_obj_set_style_text_font(status, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_align(status, LV_ALIGN_TOP_MID, 0, 30);

    // Create 4 carburetors arranged in a 2x2 grid with more spacing
    const char* cyl_names[4] = {"Cyl 1", "Cyl 2", "Cyl 3", "Cyl 4"};
    const lv_color_t cyl_colors[4] = {
        lv_color_hex(0x00ff00), // Green - reference
        lv_color_hex(0xff8800), // Orange
        lv_color_hex(0x0088ff), // Blue  
        lv_color_hex(0xff00ff)  // Magenta
    };
    
    // Positions for 2x2 grid - adjusted for better spacing
    const lv_coord_t x_positions[4] = {-70, 70, -70, 70};
    const lv_coord_t y_positions[4] = {-20, -20, 35, 35};

    for (int i = 0; i < 4; i++) {
        // Cylinder label
        lv_obj_t *cyl_label = lv_label_create(scr);
        lv_label_set_text(cyl_label, cyl_names[i]);
        lv_obj_set_style_text_color(cyl_label, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
        lv_obj_set_style_text_font(cyl_label, &lv_font_montserrat_14, LV_PART_MAIN);
        lv_obj_align(cyl_label, LV_ALIGN_CENTER, x_positions[i], y_positions[i] - 40);

        // Vacuum gauge (vertical slider) - narrower with smaller knob
        vacuum_gauges[i] = lv_slider_create(scr);
        lv_obj_set_size(vacuum_gauges[i], 15, 60);  // Reduced width from 25 to 15
        lv_obj_align(vacuum_gauges[i], LV_ALIGN_CENTER, x_positions[i], y_positions[i]);
        lv_slider_set_range(vacuum_gauges[i], 0, 30);  // 0-30 cmHg vacuum
        lv_slider_set_value(vacuum_gauges[i], vacuum_readings[i], LV_ANIM_OFF);
        lv_obj_set_style_bg_color(vacuum_gauges[i], lv_color_hex(0x333333), LV_PART_MAIN);
        lv_obj_set_style_bg_color(vacuum_gauges[i], cyl_colors[i], LV_PART_INDICATOR);
        
        // Style the slider knob to be smaller
        lv_obj_set_style_width(vacuum_gauges[i], 8, LV_PART_KNOB);   // Knob width: half of slider width
        lv_obj_set_style_height(vacuum_gauges[i], 10, LV_PART_KNOB); // Knob height
        lv_obj_set_style_bg_color(vacuum_gauges[i], lv_color_hex(0xffffff), LV_PART_KNOB); // White knob
        
        // Add to group for encoder control
        if (default_group != NULL) {
            lv_group_add_obj(default_group, vacuum_gauges[i]);
            ESP_LOGI(TAG, "Added vacuum gauge %d to encoder group", i + 1);
            if (i == 0) {
                lv_group_focus_obj(vacuum_gauges[i]); // Start with cylinder 1
                ESP_LOGI(TAG, "Focused on cylinder 1");
            }
        } else {
            ESP_LOGE(TAG, "default_group is NULL! Cannot add slider to group");
        }

        // Value label
        vacuum_labels[i] = lv_label_create(scr);
        lv_label_set_text_fmt(vacuum_labels[i], "%ld cmHg", (long)vacuum_readings[i]);
        lv_obj_set_style_text_color(vacuum_labels[i], cyl_colors[i], LV_PART_MAIN);
        lv_obj_align_to(vacuum_labels[i], vacuum_gauges[i], LV_ALIGN_OUT_BOTTOM_MID, 0, 5);

        // Add event callbacks
        lv_obj_add_event_cb(vacuum_gauges[i], vacuum_gauge_event_cb, LV_EVENT_VALUE_CHANGED, vacuum_labels[i]);
        lv_obj_add_event_cb(vacuum_gauges[i], encoder_key_event_cb, LV_EVENT_KEY, NULL);
    }

    // Differential displays (relative to cylinder 1) - positioned at bottom right
    for (int i = 0; i < 3; i++) {
        diff_labels[i] = lv_label_create(scr);
        int32_t diff = vacuum_readings[i + 1] - vacuum_readings[0];
        lv_label_set_text_fmt(diff_labels[i], "D%d: %+ld cmHg", i + 2, (long)diff);
        lv_obj_set_style_text_font(diff_labels[i], &lv_font_montserrat_14, LV_PART_MAIN);
        
        // Position differentials at bottom right to avoid gauge overlap
        lv_obj_align(diff_labels[i], LV_ALIGN_BOTTOM_RIGHT, -5, -35 - (i * 18));
        
        // Color code based on difference
        if (abs(diff) <= 2) {
            lv_obj_set_style_text_color(diff_labels[i], lv_color_hex(0x00ff00), LV_PART_MAIN);
        } else if (abs(diff) <= 5) {
            lv_obj_set_style_text_color(diff_labels[i], lv_color_hex(0xff8800), LV_PART_MAIN);
        } else {
            lv_obj_set_style_text_color(diff_labels[i], lv_color_hex(0xff0000), LV_PART_MAIN);
        }
    }

    // Instructions - at the bottom with smaller font
    lv_obj_t *instructions = lv_label_create(scr);
    lv_label_set_text(instructions, 
        "Rotate: Adjust cylinder | Press: Switch mode/cylinder");
    lv_obj_set_style_text_color(instructions, lv_color_hex(0xcccccc), LV_PART_MAIN);
    lv_obj_set_style_text_font(instructions, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_set_style_text_align(instructions, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_align(instructions, LV_ALIGN_BOTTOM_MID, 0, -5);

    // Set initial slider states (ADC mode)
    update_slider_editability();

    // Unlock LVGL mutex
    lvgl_port_unlock();

    ESP_LOGI(TAG, "4-Cylinder Carburetor Balancer UI created");
}

// =============================================================================
// Main Application
// =============================================================================

void app_main(void)
{
    ESP_LOGI(TAG, "ESP32-S3 4-Cylinder Carburetor Balancer Starting...");
    ESP_LOGI(TAG, "Hardware: ESP32-S3, ILI9341 LCD, EC11 Encoder");

    // Initialize hardware
    ESP_ERROR_CHECK(backlight_init());
    ESP_ERROR_CHECK(adc_init());
    ESP_ERROR_CHECK(lcd_init());
    ESP_ERROR_CHECK(lvgl_init()); // This already calls encoder_init()

    // Create 4-cylinder carburetor balancer UI
    create_carb_balancer_ui();

    // Create queue for vacuum sensor data communication
    vacuum_update_queue = xQueueCreate(10, sizeof(vacuum_data_t));
    if (vacuum_update_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create vacuum update queue");
        return;
    }
    
    // Start UI update task
    xTaskCreate(ui_update_task, "ui_update", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "UI update task started");

    // Start vacuum sensor update timer
    const esp_timer_create_args_t vacuum_timer_args = {
        .callback = &vacuum_update_timer_callback,
        .name = "vacuum_update"
    };
    ESP_ERROR_CHECK(esp_timer_create(&vacuum_timer_args, &vacuum_update_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(vacuum_update_timer, VACUUM_UPDATE_PERIOD_MS * 1000)); // Convert ms to us

    // Start encoder polling task (commented out - using LVGL encoder integration instead)
    // xTaskCreate(encoder_task, "encoder_task", 2048, NULL, 5, NULL);

    ESP_LOGI(TAG, "4-Cylinder Carburetor Balancer ready! Real-time vacuum sensors active.");

    // Main loop - LVGL tasks run in background
    while (1) {
        // Future: Add ADC reading for 4 vacuum sensors
        // Future: Add advanced carburetor balancing algorithms
        // Future: Add data logging and calibration features
        // LVGL updates automatically via lvgl_port
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
