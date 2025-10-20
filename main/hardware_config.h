/**
 * @file hardware_config.h
 * @brief Hardware pin definitions for ESP32-S3 LVGL Template
 * 
 * This file contains all hardware pin definitions and constants
 * for the ESP32-S3 board with ILI9341 LCD and EC11 rotary encoder.
 * 
 * Hardware Configuration:
 * - ESP32-S3 with 32MB Flash, 8MB PSRAM (Octal)
 * - ILI9341 320x240 LCD (SPI interface)
 * - EC11 Rotary Encoder with button
 * - PWM backlight control
 */

#ifndef HARDWARE_CONFIG_H
#define HARDWARE_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================================
// LCD Display Configuration (ILI9341 via SPI)
// =============================================================================

// SPI Bus Configuration
#define LCD_HOST            SPI2_HOST
#define LCD_PIN_NUM_SCLK    18
#define LCD_PIN_NUM_MOSI    19
#define LCD_PIN_NUM_MISO    21

// LCD Control Pins
#define LCD_PIN_NUM_LCD_DC  5
#define LCD_PIN_NUM_LCD_CS  4
#define LCD_PIN_NUM_LCD_RST 3

// LCD Display Parameters
#define LCD_H_RES           240
#define LCD_V_RES           320
#define LCD_PIXEL_CLOCK_HZ  (40 * 1000 * 1000)
#define LCD_CMD_BITS        8
#define LCD_PARAM_BITS      8
#define LCD_BITS_PER_PIXEL  16

// =============================================================================
// Backlight Configuration (PWM/LEDC)
// =============================================================================

#define BK_LIGHT_OUTPUT_IO  2
#define BK_LIGHT_MODE       LEDC_LOW_SPEED_MODE
#define BK_LIGHT_CHANNEL    LEDC_CHANNEL_0
#define BK_LIGHT_TIMER      LEDC_TIMER_0
#define BK_LIGHT_DUTY_RES   LEDC_TIMER_13_BIT
#define BK_LIGHT_FREQ_HZ    5000
#define BK_LIGHT_MAX_DUTY   ((1 << 13) - 1)  // 8191 for 13-bit resolution
#define BK_LIGHT_DUTY       (BK_LIGHT_MAX_DUTY * 70 / 100)  // 70% default brightness

// =============================================================================
// Vacuum Sensor Configuration (ADC)
// =============================================================================

// ADC Configuration for 4 vacuum sensors
#define VACUUM_ADC_UNIT         ADC_UNIT_1
#define VACUUM_ADC_ATTEN        ADC_ATTEN_DB_12     // 0-3.3V range
#define VACUUM_ADC_BITWIDTH     ADC_BITWIDTH_12     // 12-bit resolution (0-4095)

// Vacuum sensor ADC pins (ADC1 channels for ESP32-S3)
// Note: Avoiding pins used by LCD (18,19,21,5,4,3) and backlight (2)
#define VACUUM_SENSOR_1_PIN     1    // ADC1_CH0 - Cylinder 1 (reference)
#define VACUUM_SENSOR_2_PIN     7    // ADC1_CH6 - Cylinder 2
#define VACUUM_SENSOR_3_PIN     8    // ADC1_CH7 - Cylinder 3  
#define VACUUM_SENSOR_4_PIN     9    // ADC1_CH8 - Cylinder 4

// ADC channels corresponding to pins
#define VACUUM_SENSOR_1_CHANNEL ADC1_CHANNEL_0
#define VACUUM_SENSOR_2_CHANNEL ADC1_CHANNEL_6
#define VACUUM_SENSOR_3_CHANNEL ADC1_CHANNEL_7
#define VACUUM_SENSOR_4_CHANNEL ADC1_CHANNEL_8

// Vacuum sensor calibration (pressure transducer specific)
#define VACUUM_SENSOR_VCC       3300    // Sensor supply voltage (mV)
#define VACUUM_SENSOR_MIN_MV    500     // Minimum output voltage (0 cmHg)
#define VACUUM_SENSOR_MAX_MV    2500    // Maximum output voltage (30 cmHg)
#define VACUUM_SENSOR_MIN_CMHG  0       // Minimum vacuum (cmHg)
#define VACUUM_SENSOR_MAX_CMHG  30      // Maximum vacuum (cmHg)

// ADC sampling configuration
#define VACUUM_ADC_SAMPLES      8       // Number of samples to average
#define VACUUM_UPDATE_PERIOD_MS 100     // Update period in milliseconds

// =============================================================================
// Rotary Encoder Configuration (EC11)
// =============================================================================

#define EC11_GPIO_A         47   // Encoder phase A
#define EC11_GPIO_B         48   // Encoder phase B
#define EC11_GPIO_BUTTON    6   // Encoder button (push)

// Button configuration
#define BUTTON_ACTIVE_LEVEL 0   // Active low

// =============================================================================
// LVGL Configuration Constants
// =============================================================================

#define LVGL_TASK_PRIORITY       5
#define LVGL_TASK_STACK_SIZE     (6 * 1024)
#define LVGL_TASK_MAX_DELAY_MS   500
#define LVGL_TICK_PERIOD_MS      5

#ifdef __cplusplus
}
#endif

#endif // HARDWARE_CONFIG_H
