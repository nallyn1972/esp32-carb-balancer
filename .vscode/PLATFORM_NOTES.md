# Cross-Platform Development Notes

This project is configured to work on both **Linux** and **Windows 11**.

## VS Code Configuration

### IntelliSense Configurations

The project includes two IntelliSense configurations in `c_cpp_properties.json`:

1. **ESP-IDF (Linux)** - For Linux development
2. **ESP-IDF (Windows)** - For Windows 11 development

VS Code automatically selects the correct configuration based on your OS.

### Serial Port Configuration

Edit `.vscode/settings.json` to set your serial port:

**Linux:**
```json
"idf.port": "/dev/ttyUSB0"
```
Common ports: `/dev/ttyUSB0`, `/dev/ttyACM0`, `/dev/ttyS0`

Check available ports:
```bash
ls /dev/tty*
```

**Windows:**
```json
"idf.portWin": "COM3"
```
Common ports: `COM3`, `COM4`, `COM5`, etc.

Check Device Manager under "Ports (COM & LPT)"

## Path Differences

### ESP-IDF Location

**Linux:**
- Default: `~/esp/esp-idf`
- Tools: `~/.espressif`

**Windows:**
- Default: `C:\Users\<username>\esp\esp-idf`
- Tools: `C:\Users\<username>\.espressif`

### Compiler Paths

**Linux:**
```
~/.espressif/tools/xtensa-esp-elf/esp-13.2.0_20240530/xtensa-esp-elf/bin/xtensa-esp32-elf-gcc
```

**Windows:**
```
C:\Users\<username>\.espressif\tools\xtensa-esp-elf\esp-13.2.0_20240530\xtensa-esp-elf\bin\xtensa-esp32-elf-gcc.exe
```

## Environment Setup

### Linux

Add to `~/.bashrc` for convenience:
```bash
alias get_idf='. $HOME/esp/esp-idf/export.sh'
```

Then use:
```bash
get_idf
```

### Windows

Create a PowerShell function by adding to your profile:
```powershell
# Edit profile: notepad $PROFILE
function Get-IDF {
    & "$HOME\esp\esp-idf\export.ps1"
}
```

Then use:
```powershell
Get-IDF
```

## Command Differences

### Building

**Linux/Mac:**
```bash
. ~/esp/esp-idf/export.sh
idf.py build
```

**Windows PowerShell:**
```powershell
$HOME\esp\esp-idf\export.ps1
idf.py build
```

**Windows CMD:**
```cmd
%USERPROFILE%\esp\esp-idf\export.bat
idf.py build
```

### Flashing

**Linux:**
```bash
idf.py -p /dev/ttyUSB0 flash monitor
```

**Windows:**
```powershell
idf.py -p COM3 flash monitor
```

## USB Driver Requirements

### Linux
- Usually works out of the box
- If permission denied: `sudo usermod -a -G dialout $USER`
- Then log out and back in

### Windows
- **CP210x**: Install [Silicon Labs drivers](https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers)
- **CH340**: Install [CH340 drivers](https://sparks.gogo.co.nz/ch340.html)
- **FTDI**: Usually automatic via Windows Update

## VS Code Tasks

All tasks work on both platforms:

- **Build - ESP-IDF**: `Ctrl+Shift+B`
- **Flash - ESP-IDF**: Flash to device
- **Monitor - ESP-IDF**: Serial monitor
- **Flash & Monitor - ESP-IDF**: Combined operation

Tasks automatically use the correct port configuration (`idf.port` for Linux, `idf.portWin` for Windows).

## Troubleshooting

### IntelliSense Issues

1. Build the project first: `idf.py build`
2. Reset IntelliSense: `Ctrl+Shift+P` → `C/C++: Reset IntelliSense Database`
3. Check configuration: `Ctrl+Shift+P` → `C/C++: Select a Configuration`
4. Verify `build/compile_commands.json` exists

### ESP-IDF Not Found

**Linux:**
- Check: `echo $IDF_PATH`
- Should be: `/home/<user>/esp/esp-idf`
- Fix: `. ~/esp/esp-idf/export.sh`

**Windows:**
- Check: `$env:IDF_PATH`
- Should be: `C:\Users\<user>\esp\esp-idf`
- Fix: `$HOME\esp\esp-idf\export.ps1`

### Build Errors After Switching Platforms

If you switch between Linux and Windows:
```bash
# Delete build directory
rm -rf build      # Linux
Remove-Item -Recurse -Force build  # Windows

# Rebuild
idf.py build
```

## Best Practices

1. **Don't commit `build/` directory** - It's platform-specific and in `.gitignore`
2. **Don't commit `sdkconfig`** - It may have platform-specific paths
3. **Keep `.vscode/` in version control** - It's configured for both platforms
4. **Update serial ports** in `.vscode/settings.json` for your setup
5. **Always build after switching platforms** to regenerate platform-specific files

## Sharing Project

When sharing this project:
1. Recipients need to install ESP-IDF for their platform
2. They should update serial port in `.vscode/settings.json`
3. First build will download all dependencies automatically
4. No platform-specific modifications needed to the code

## Additional Resources

- [ESP-IDF Installation Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html)
- [VS Code ESP-IDF Extension](https://github.com/espressif/vscode-esp-idf-extension)
- [ESP32 USB Drivers](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/establish-serial-connection.html)
