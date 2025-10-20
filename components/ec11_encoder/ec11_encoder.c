/**
 * @file ec11_encoder.c
 * @brief EC11 Rotary Encoder Component Implementation
 * 
 * This component provides a custom implementation for EC11 rotary encoders
 * with proper quadrature decoding and LVGL integration.
 */

#include "ec11_encoder.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_err.h"
#include "lvgl.h"

static const char *TAG = "EC11_ENCODER";

// Encoder state variables
static volatile int16_t encoder_count = 0;
static volatile uint8_t encoder_last_state = 0;
static volatile uint32_t last_interrupt_time = 0;

// Configuration
static ec11_encoder_config_t encoder_config = {0};
static bool encoder_initialized = false;

// Interrupt handler for encoder
static void IRAM_ATTR encoder_isr_handler(void* arg)
{
    uint32_t current_time = esp_timer_get_time() / 1000; // Convert to ms
    
    // Debounce - ignore interrupts that happen too quickly
    if (current_time - last_interrupt_time < encoder_config.debounce_ms) {
        return;
    }
    
    uint8_t a = gpio_get_level(encoder_config.gpio_a);
    uint8_t b = gpio_get_level(encoder_config.gpio_b);
    uint8_t current_state = (a << 1) | b;
    
    // Standard quadrature encoding: only count on one transition per detent
    // EC11 sequence per detent: 11 -> 01 -> 00 -> 10 -> 11 (clockwise)
    // We'll count only on the 11 -> 01 transition for clockwise
    // and 11 -> 10 transition for counter-clockwise
    
    if (encoder_last_state == 0x3) {  // Starting from state 11 (both high)
        if (current_state == 0x1) {   // 11 -> 01 (A goes low first)
            encoder_count++;          // Clockwise
            last_interrupt_time = current_time;
        } else if (current_state == 0x2) {  // 11 -> 10 (B goes low first)
            encoder_count--;          // Counter-clockwise
            last_interrupt_time = current_time;
        }
    }
    
    encoder_last_state = current_state;
}

// LVGL encoder reading function
static void encoder_read_cb(lv_indev_t *indev, lv_indev_data_t *data)
{
    static int16_t last_encoder_count = 0;
    
    // Get the current encoder count and calculate difference
    int16_t current_count = encoder_count;
    int16_t diff = current_count - last_encoder_count;
    last_encoder_count = current_count;
    
    // Return the difference
    data->enc_diff = diff;
    
    // Read button state
    bool button_pressed = (gpio_get_level(encoder_config.gpio_button) == (encoder_config.button_active_low ? 0 : 1));
    data->state = button_pressed ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;
}

esp_err_t ec11_encoder_init(const ec11_encoder_config_t *config)
{
    if (!config) {
        ESP_LOGE(TAG, "Configuration pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (encoder_initialized) {
        ESP_LOGW(TAG, "Encoder already initialized");
        return ESP_OK;
    }
    
    // Copy configuration
    encoder_config = *config;
    
    // Set default debounce time if not specified
    if (encoder_config.debounce_ms == 0) {
        encoder_config.debounce_ms = 5;
    }
    
    ESP_LOGI(TAG, "Initializing EC11 encoder on pins A=%d, B=%d, Button=%d", 
             encoder_config.gpio_a, encoder_config.gpio_b, encoder_config.gpio_button);
    
    // Configure encoder pins with pull-ups
    gpio_config_t encoder_gpio_config = {
        .pin_bit_mask = (1ULL << encoder_config.gpio_a) | (1ULL << encoder_config.gpio_b),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE,  // Trigger on any edge
    };
    ESP_ERROR_CHECK(gpio_config(&encoder_gpio_config));
    
    // Configure button pin
    gpio_config_t button_gpio_config = {
        .pin_bit_mask = (1ULL << encoder_config.gpio_button),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&button_gpio_config));
    
    // Install ISR service and add handlers
    esp_err_t ret = gpio_install_isr_service(0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        // ESP_ERR_INVALID_STATE means ISR service is already installed, which is OK
        ESP_ERROR_CHECK(ret);
    }
    
    ESP_ERROR_CHECK(gpio_isr_handler_add(encoder_config.gpio_a, encoder_isr_handler, NULL));
    ESP_ERROR_CHECK(gpio_isr_handler_add(encoder_config.gpio_b, encoder_isr_handler, NULL));
    
    // Initialize encoder state
    uint8_t a = gpio_get_level(encoder_config.gpio_a);
    uint8_t b = gpio_get_level(encoder_config.gpio_b);
    encoder_last_state = (a << 1) | b;
    
    // Reset encoder count
    encoder_count = 0;
    
    encoder_initialized = true;
    ESP_LOGI(TAG, "EC11 encoder initialized successfully");
    
    return ESP_OK;
}

lv_indev_t* ec11_encoder_create_lvgl_indev(void)
{
    if (!encoder_initialized) {
        ESP_LOGE(TAG, "Encoder not initialized. Call ec11_encoder_init() first");
        return NULL;
    }
    
    // Create LVGL input device
    lv_indev_t *indev = lv_indev_create();
    if (!indev) {
        ESP_LOGE(TAG, "Failed to create LVGL input device");
        return NULL;
    }
    
    lv_indev_set_type(indev, LV_INDEV_TYPE_ENCODER);
    lv_indev_set_read_cb(indev, encoder_read_cb);
    
    // Configure input device properties
    lv_indev_set_scroll_throw(indev, false);
    lv_indev_set_long_press_time(indev, 400);
    
    ESP_LOGI(TAG, "LVGL input device created successfully");
    return indev;
}

int16_t ec11_encoder_get_count(void)
{
    return encoder_count;
}

void ec11_encoder_reset_count(void)
{
    encoder_count = 0;
}

bool ec11_encoder_get_button_state(void)
{
    if (!encoder_initialized) {
        return false;
    }
    
    return (gpio_get_level(encoder_config.gpio_button) == (encoder_config.button_active_low ? 0 : 1));
}

esp_err_t ec11_encoder_deinit(void)
{
    if (!encoder_initialized) {
        return ESP_OK;
    }
    
    // Remove ISR handlers
    gpio_isr_handler_remove(encoder_config.gpio_a);
    gpio_isr_handler_remove(encoder_config.gpio_b);
    
    // Reset GPIO pins
    gpio_reset_pin(encoder_config.gpio_a);
    gpio_reset_pin(encoder_config.gpio_b);
    gpio_reset_pin(encoder_config.gpio_button);
    
    encoder_initialized = false;
    ESP_LOGI(TAG, "EC11 encoder deinitialized");
    
    return ESP_OK;
}