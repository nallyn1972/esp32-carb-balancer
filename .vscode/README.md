# VS Code Configuration for ESP32-S3 Carburetor Balancer

This directory contains VS Code workspace configuration for cross-platform development.

## Prerequisites

1. **ESP-IDF Extension**: Install the official ESP-IDF extension from the VS Code marketplace
2. **ESP-IDF Framework**: Install ESP-IDF v5.0 or later following the [official guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/)

## Platform-Specific Setup

### Windows
- Ensure ESP-IDF is properly installed via the Windows installer
- The extension will auto-detect ESP-IDF paths
- Serial port will typically be `COM3`, `COM4`, etc.

### Linux
- Install ESP-IDF following the Linux installation guide
- Add your user to the `dialout` group for serial port access:
  ```bash
  sudo usermod -a -G dialout $USER
  ```
- Serial port will typically be `/dev/ttyUSB0` or `/dev/ttyACM0`
- Log out and back in for group changes to take effect

### macOS
- Install ESP-IDF following the macOS installation guide
- Serial port will typically be `/dev/cu.usbserial-*` or `/dev/cu.SLAB_USBtoUART`

## Configuration Files

- **`settings.json`**: ESP-IDF extension settings and editor preferences
- **`c_cpp_properties.json`**: IntelliSense configuration for all platforms
- **`tasks.json`**: Build, flash, and monitor tasks

## Usage

1. Open this project in VS Code
2. The ESP-IDF extension should automatically detect the configuration
3. Use `Ctrl+Shift+P` (Windows/Linux) or `Cmd+Shift+P` (macOS) and search for "ESP-IDF" commands
4. Use the provided tasks:
   - **Build**: `Ctrl+Shift+P` → "Tasks: Run Task" → "Build - ESP-IDF"
   - **Flash**: `Ctrl+Shift+P` → "Tasks: Run Task" → "Flash - ESP-IDF"
   - **Monitor**: `Ctrl+Shift+P` → "Tasks: Run Task" → "Monitor - ESP-IDF"

## Troubleshooting

### IntelliSense Issues
- Run the ESP-IDF build at least once to generate `compile_commands.json`
- Restart VS Code after changing ESP-IDF settings
- Check that the ESP-IDF extension is properly configured

### Serial Port Issues
- **Windows**: Check Device Manager for correct COM port
- **Linux**: Ensure user is in `dialout` group and port permissions are correct
- **macOS**: Use `ls /dev/cu.*` to find the correct port

### Build Issues
- Ensure ESP-IDF environment is properly sourced
- Check that `idf.py` is in your PATH
- Verify ESP-IDF target is set to `esp32s3`

## File Structure
```
.vscode/
├── README.md                 # This file
├── settings.json            # VS Code and ESP-IDF settings
├── c_cpp_properties.json    # C/C++ IntelliSense configuration
├── tasks.json              # Build/flash/monitor tasks
├── INTELLISENSE_SETUP.md    # IntelliSense setup guide
└── PLATFORM_NOTES.md       # Platform-specific notes
```