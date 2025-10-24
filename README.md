# ESP32-S3 HMI for Battery Management System

A complete Human-Machine Interface (HMI) application for ESP32-S3 with 7" touch display, designed for Battery Management System monitoring via CAN bus.

## Features

- **7" Touch LCD Display** - LVGL-based user interface
- **WiFi Connectivity** - Station mode with AP fallback
- **OTA Updates** - Over-the-air firmware updates (no USB needed!)
- **CAN Bus Communication** - CANopen heartbeat and NMT support
- **Modbus TCP Server** - Remote data access on port 502
- **Telnet Serial** - Remote debugging on port 23
- **Configuration via secrets.h** - No hardcoded credentials

## Hardware

- **Board:** Waveshare ESP32-S3-Touch-LCD-7
- **MCU:** ESP32-S3-N16R8 (16MB Flash, 8MB PSRAM)
- **Display:** 7" RGB LCD with capacitive touch (800x480)
- **CAN:** Integrated CAN transceiver

## Getting Started

### 1. Install PlatformIO

```bash
# Install PlatformIO Core or use PlatformIO IDE extension for VSCode
pip install platformio
```

### 2. Clone and Configure

```bash
git clone <your-repo-url>
cd ESP32S3_HMI_IFS

# Copy secrets template and configure
cp src/secrets.h.example src/secrets.h
# Edit src/secrets.h with your WiFi credentials and settings
```

### 3. Build and Upload

**First time (via USB):**
```bash
# Build
pio run -e IBB_HMI

# Upload (hold BOOT button, then power cycle)
pio run -e IBB_HMI --target upload
```

**Subsequent updates (via OTA - recommended):**
```bash
# Build and upload over WiFi
pio run -e IBB_HMI_OTA --target upload
```

## Configuration (secrets.h)

All sensitive configuration is stored in `src/secrets.h` (not committed to git).

**Copy `src/secrets.h.example` to `src/secrets.h` and configure:**

- WiFi credentials (STA and AP mode)
- OTA hostname and password
- CAN bus settings (node ID, speed)
- Modbus TCP settings
- Telnet settings
- Display version text and color

## Project Structure

```
├── src/
│   ├── main.cpp                    # Main application
│   ├── secrets.h.example           # Configuration template
│   ├── lvgl_v8_port.h/cpp         # LVGL display driver
│   └── ui*.h                       # UI components
├── lib/
│   ├── WaveshareCAN/              # CAN bus library
│   └── eModbus/                    # Modbus TCP library
├── docs/                           # CAN protocol documentation
└── platformio.ini                  # PlatformIO configuration
```

## Network Services

When connected to WiFi, the following services are available:

- **OTA Update:** Port 3232 (configured in secrets.h)
- **Modbus TCP:** Port 502
- **Telnet Serial:** Port 23

Connect via telnet for remote debugging:
```bash
telnet <device-ip> 23
```

## CAN Bus

- **Protocol:** CANopen
- **Node ID:** Configurable in secrets.h (default: 136 / 0x88 hex)
- **Heartbeat:** 1Hz transmission (COB-ID: 0x700 + Node ID)
- **NMT Support:**
  - `0x000 81 <node-id>` - Reset ESP32
  - `0x000 81 00` - Reset CAN layer

## Display

- Shows WiFi status and IP address
- CAN bus status
- OTA update progress
- Customizable version text and title color

## Important Notes

### USB Programming Limitation

⚠️ **When CAN is active, USB programming may not work reliably** due to hardware pin sharing (GPIO19/20 via FSUSB42UMX multiplexer).

**Solution:** Use OTA for all updates after initial USB upload.

### OTA Benefits

- No BOOT button needed
- No manual power cycle needed
- Faster development workflow
- Can update remotely

## Development Workflow

1. Make code changes
2. Build: `pio run -e IBB_HMI_OTA`
3. Upload: `pio run -e IBB_HMI_OTA --target upload`
4. Device automatically reboots with new firmware

## Troubleshooting

**WiFi not connecting?**
- Check credentials in `secrets.h`
- Device will create AP "HMI_AP" as fallback

**OTA not working?**
- Verify device IP matches `upload_port` in platformio.ini
- Check OTA password in `secrets.h`

**Display issues?**
- Check LVGL configuration in `src/lv_conf.h`
- Verify display panel settings in `src/esp_panel_*.h`

## License

[Specify your license here]

## Credits

- [paelzer/p43lz3r_Waveshare_ESP32S3_Touch_LCD_7_LVGL_CANbus](https://github.com/paelzer/p43lz3r_Waveshare_ESP32S3_Touch_LCD_7_LVGL_CANbus) - Invaluable reference for Waveshare ESP32-S3 with LVGL and CAN bus integration
- ESP32 Display Panel Library
- LVGL - Graphics library
- eModbus - Modbus TCP library
- WaveshareCAN - CAN bus library

## Support

For issues and questions, please open an issue on GitHub.
