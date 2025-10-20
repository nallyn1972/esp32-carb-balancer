/**
 * @file ec11_encoder.h
 * @brief EC11 Rotary Encoder Component for LVGL
 * 
 * This component provides a custom implementation for EC11 rotary encoders
 * that works better with LVGL than generic encoder libraries. It provides
 * precise single-step increments per detent and stable direction detection.
 * 
 * Features:
 * - Interrupt-based encoder reading for responsive input
 * - Proper quadrature decoding for EC11 encoders
 * - Hardware debouncing to prevent bounce/noise issues
 * - Integrated button handling
 * - Direct LVGL integration
 * 
 * @author ESP32-S3 LVGL Template Project
 * @date 2025
 */

#ifndef EC11_ENCODER_H
#define EC11_ENCODER_H

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

// Forward declaration for LVGL types (to avoid requiring LVGL header)
typedef struct _lv_indev_t lv_indev_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief EC11 Encoder configuration structure
 */
typedef struct {
    int gpio_a;              /**< GPIO number for encoder phase A */
    int gpio_b;              /**< GPIO number for encoder phase B */
    int gpio_button;         /**< GPIO number for encoder button (push) */
    bool button_active_low;  /**< true if button is active low, false if active high */
    uint32_t debounce_ms;    /**< Debounce time in milliseconds (default: 5) */
} ec11_encoder_config_t;

/**
 * @brief Initialize EC11 encoder with GPIO configuration
 * 
 * This function configures the GPIO pins for the encoder and button,
 * sets up interrupts, and initializes the encoder state.
 * 
 * @param config Pointer to encoder configuration structure
 * @return ESP_OK on success, ESP_ERR_* on error
 */
esp_err_t ec11_encoder_init(const ec11_encoder_config_t *config);

/**
 * @brief Create and configure LVGL input device for the encoder
 * 
 * This function creates an LVGL input device of type LV_INDEV_TYPE_ENCODER
 * and configures it to read from the EC11 encoder.
 * 
 * @return Pointer to created LVGL input device, or NULL on error
 */
lv_indev_t* ec11_encoder_create_lvgl_indev(void);

/**
 * @brief Get the current encoder count value
 * 
 * @return Current encoder count (can be negative)
 */
int16_t ec11_encoder_get_count(void);

/**
 * @brief Reset the encoder count to zero
 */
void ec11_encoder_reset_count(void);

/**
 * @brief Get the current button state
 * 
 * @return true if button is pressed, false if not pressed
 */
bool ec11_encoder_get_button_state(void);

/**
 * @brief Deinitialize the encoder and clean up resources
 * 
 * @return ESP_OK on success, ESP_ERR_* on error
 */
esp_err_t ec11_encoder_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // EC11_ENCODER_H