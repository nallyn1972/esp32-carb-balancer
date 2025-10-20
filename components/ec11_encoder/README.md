# EC11 Encoder Component

A custom ESP-IDF component for EC11 rotary encoders with integrated LVGL support.

## Features

- ✅ Interrupt-based encoder reading for responsive input
- ✅ Proper quadrature decoding specifically tuned for EC11 encoders
- ✅ Hardware debouncing to prevent bounce/noise issues
- ✅ Integrated button handling (encoder push button)
- ✅ Direct LVGL integration with input device creation
- ✅ Precise single-step increments per detent
- ✅ Stable direction detection

## Usage

### 1. Include the Component

Add this component to your project's `components/` directory or reference it in your main component's dependencies.

### 2. Basic Usage

```c
#include "ec11_encoder.h"

// Configure encoder
ec11_encoder_config_t encoder_config = {
    .gpio_a = 47,              // Encoder phase A pin
    .gpio_b = 48,              // Encoder phase B pin  
    .gpio_button = 6,          // Encoder button pin
    .button_active_low = true, // Button is active low
    .debounce_ms = 5           // 5ms debounce (optional, default is 5)
};

// Initialize encoder
ESP_ERROR_CHECK(ec11_encoder_init(&encoder_config));

// Create LVGL input device
lv_indev_t *encoder_indev = ec11_encoder_create_lvgl_indev();

// Create and configure LVGL group
lv_group_t *group = lv_group_create();
lv_indev_set_group(encoder_indev, group);
lv_group_set_default(group);

// Add UI objects to the group
lv_group_add_obj(group, your_slider);
lv_group_add_obj(group, your_button);
```

### 3. Advanced Usage

```c
// Get current encoder count
int16_t count = ec11_encoder_get_count();

// Reset encoder count to zero
ec11_encoder_reset_count();

// Check button state manually
bool button_pressed = ec11_encoder_get_button_state();

// Clean up when done
ec11_encoder_deinit();
```

## Configuration

The `ec11_encoder_config_t` structure contains:

- `gpio_a`: GPIO number for encoder phase A
- `gpio_b`: GPIO number for encoder phase B  
- `gpio_button`: GPIO number for encoder button
- `button_active_low`: Set to `true` if button is active low, `false` if active high
- `debounce_ms`: Debounce time in milliseconds (default: 5ms)

## Hardware Requirements

- EC11 rotary encoder (or compatible quadrature encoder)
- Pull-up resistors (handled automatically by internal pull-ups)
- GPIO pins capable of interrupt handling

## Technical Details

### Quadrature Decoding

The component uses a state machine that detects specific transitions in the EC11's quadrature output:

- **Clockwise**: Detects `11 → 01` transition (phase A goes low first)
- **Counter-clockwise**: Detects `11 → 10` transition (phase B goes low first)

This ensures exactly one count per physical detent click.

### Interrupt Handling

- Uses GPIO interrupts on both encoder phases for responsive input
- Hardware debouncing prevents multiple triggers from mechanical bounce
- IRAM-safe interrupt handler for real-time performance

## Compatibility

- **ESP-IDF**: 5.0+
- **LVGL**: 8.x, 9.x
- **Hardware**: EC11, similar quadrature encoders

## License

This component is part of the ESP32-S3 LVGL Template project and is provided as-is for educational and development purposes.