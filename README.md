# ESP32-S3 Carburetor Balancing Tool

A digital carburetor synchronization tool built with ESP32-S3, featuring a modern LVGL GUI and precise vacuum measurements for motorcycle and automotive carburetor balancing.

## Features

- 🎛️ **Digital Vacuum Gauges**: Real-time vacuum readings for multiple carburetors
- 📱 **Modern GUI**: LVGL-based user interface with intuitive controls
- 🎚️ **Rotary Encoder Navigation**: EC11 encoder for smooth menu navigation
- 📊 **Differential Display**: Shows vacuum differences between cylinders
- ⚡ **Real-time Updates**: Live vacuum readings and calculations
- 🔧 **Portable**: Battery-powered handheld design

## Hardware Requirements

### Core Components
- **ESP32-S3** microcontroller (32MB Flash, 8MB PSRAM recommended)
- **ILI9341** 240x320 LCD display (SPI interface)
- **EC11** rotary encoder with push button
- **Vacuum sensors** (pressure transducers) for each carburetor

### Pin Configuration
- **Display (SPI)**: SCLK=18, MOSI=19, MISO=21, DC=5, CS=4, RST=3
- **Backlight PWM**: GPIO 2
- **Encoder**: Phase A=47, Phase B=48, Button=6
- **Vacuum Sensors**: ADC pins (configurable)

## Software Architecture

### Components
- **EC11 Encoder Component**: Custom component for precise encoder handling
- **LVGL GUI**: Modern graphics library for the user interface
- **ESP-IDF Framework**: ESP32 development framework
- **Hardware Abstraction**: Configurable pin assignments

### Key Features
- Interrupt-based encoder reading with 8ms debouncing
- Vacuum differential calculations
- Color-coded gauge displays (green=good, orange=warning, red=error)
- Multi-cylinder support (expandable)

## Getting Started

### Prerequisites
1. **ESP-IDF** v5.0 or later
2. **VS Code** with ESP-IDF extension
3. **Hardware setup** as per pin configuration

### Building the Project
1. Clone or copy this project to your workspace
2. Open in VS Code with ESP-IDF extension
3. Configure target: `ESP32-S3`
4. Build: `Ctrl+Shift+P` → "ESP-IDF: Build"
5. Flash: `Ctrl+Shift+P` → "ESP-IDF: Flash"

### Usage
1. Connect vacuum lines to the sensors
2. Power on the device
3. Use the rotary encoder to navigate between gauges
4. Monitor vacuum differences in real-time
5. Adjust carburetors until differential is <2 cmHg

## User Interface

### Main Screen
- **Title**: "Carburetor Balancer"
- **Status**: System status indicator
- **Vacuum Gauges**: Vertical sliders showing vacuum levels (0-30 cmHg)
- **Differential**: Real-time difference calculation
- **Instructions**: On-screen help text

### Controls
- **Rotate Encoder**: Adjust target values or navigate
- **Press Encoder**: Switch between gauge controls
- **Target**: Keep differential under 2 cmHg for optimal balance

## Technical Specifications

- **Vacuum Range**: 0-30 cmHg
- **Resolution**: 0.1 cmHg
- **Update Rate**: 10 Hz
- **Display**: 240x320 color LCD
- **Power**: USB or battery powered
- **Operating Temp**: 0°C to 50°C

## Development

### Project Structure
```
esp_carb_bal/
├── main/                    # Main application code
│   ├── main.c              # Main application
│   ├── hardware_config.h   # Pin definitions
│   └── idf_component.yml   # Component dependencies
├── components/             # Custom components
│   └── ec11_encoder/      # EC11 encoder component
├── .vscode/               # VS Code configuration
└── CMakeLists.txt         # Build configuration
```

### Adding Features
- **More Cylinders**: Duplicate gauge creation code
- **Data Logging**: Add SD card or WiFi logging
- **Calibration**: Implement sensor calibration routines
- **Alarms**: Add audio/visual alerts for imbalances

## Troubleshooting

### Common Issues
1. **Encoder not responding**: Check pin connections and debounce settings
2. **Display issues**: Verify SPI connections and power supply
3. **Build errors**: Ensure ESP-IDF is properly configured

### Debug Features
- Serial logging with detailed component status
- Test modes for individual components
- Configuration validation

## License

This project is based on the ESP32-S3 LVGL Template and includes the custom EC11 encoder component. Provided for educational and development purposes.

## Acknowledgments

- Based on ESP32-S3 LVGL Template v1.1.0
- Uses LVGL graphics library
- Includes custom EC11 encoder component with optimized debouncing

---

**Version**: 1.0.0  
**Target**: ESP32-S3  
**Framework**: ESP-IDF v5.0+  
**GUI**: LVGL v9.x