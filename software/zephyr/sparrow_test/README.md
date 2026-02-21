# Sparrow Test Project (Zephyr)

Minimal Zephyr application intended for the Sparrow v3.2 board (ESP32-C6-WROOM-1-N4, 4MB Flash).

## Build & Flash

From this folder:

```sh
west build -b esp32c6_devkitc/esp32c6/hpcore -p auto .
west flash -d build --runner esp32
```

## Connect over USB
```sh
screen /dev/cu.usbmodem1101 115200
```

## One-shot dev script

There is a helper script to clean, build, flash the LittleFS image, flash
firmware, and open a serial terminal:

```sh
./scripts/dev_flash.sh /dev/cu.usbmodem1101
```

Override the board with `BOARD=...` if needed.

## LittleFS assets

This app serves the all file assets directly into the LittleFS partition in the ESP32C6 internal Flash.

Build the image (requires `mklittlefs` in your PATH, run >brew install mklittlefs):

```sh
west build -t littlefs_image
```

Flash the image (overwrites `/lfs` contents, including logs):

```sh
ESPTOOL_PORT=/dev/tty.usbmodemXXXX python3 scripts/flash_littlefs_image.py --build-dir build
```

## Startup script

Files under `lfs/` are copied to the LittleFS root. If `lfs/startup.txt` is
present, the app executes each non-empty line as a shell command at boot.

Example `lfs/startup.txt`:

```txt
net iface up 1
wifi connect -s "WIFI_AP" -p "WIFI_PASSWORD" -k 1 -i 1
```

## Boot log

Boot and startup script events are logged to `/lfs/logs/boot.log`. You can read
it from the shell:

```sh
fs cat /lfs/logs/boot.log
```

## Features

This application includes a rich set of features for interacting with the Sparrow v3.2 hardware.

- **Peripherals**: Drivers and shell commands for the onboard OLED display, BME680, LSM6DSL, LTR303, MAX17048 sensors, and Neopixel.
- **Connectivity**: WiFi station mode with network time (NTP) synchronization.
- **Storage**: LittleFS filesystem on the internal flash for configuration, web assets, and data logging.
- **Remote Access**: A web-based UI and a comprehensive serial shell for control and diagnostics.
- **Automation**: A startup script (`/lfs/startup.txt`) can be used to run shell commands on boot.

## Peripherals

### OLED Display

The monochrome OLED display can be controlled via the `oled` shell command. It can display text, status information, and images.

- **Show status**: `oled status`
- **Show info screen**: `oled info 5` (updates every 5 seconds)
- **Turn display on/off**: `oled on`, `oled off`
- **Print text**: `oled print 0 0 "Hello"`
- **Draw a PBM image**: `oled bmp bitmaps/smiley.pbm`

The info screen provides a summary of the device status, including time, IP address, and sensor readings.

### Sensors

The application supports several onboard sensors, each with its own set of shell commands for configuration and data logging.

- **BME680 (Temp/Humidity/Pressure/Gas)**: `bme` command.
  - `bme read`: Read current sensor values.
  - `bme log start 30`: Start logging data every 30 seconds to `/lfs/logs/bme.csv`.
- **LSM6DSL (Accelerometer/Gyro)**: `lsm6dsl` command.
  - `lsm6dsl read`: Read current sensor values.
  - `lsm6dsl log start`: Start logging data.
- **LTR303 (Ambient Light)**: `ltr` command.
  - `ltr read`: Read current light sensor values (lux).
  - `ltr log start`: Start logging data.
- **MAX17048 (Fuel Gauge)**: `max` command.
  - `max read`: Read battery voltage and state of charge.
  - `max log start`: Start logging battery status.

### Neopixel

The RGB Neopixel can be controlled with the `neopixel` command.

- **Set color**: `neopixel set_rgb 255 0 0` (Red)
- **Set brightness**: `neopixel set_hsv 0 255 50`
- **Turn off**: `neopixel off`

## Web Interface

The device runs a web server that provides a user interface for monitoring the device.

1. Connect the device to your WiFi network:
   ```sh
   wifi connect -s "YOUR_WIFI_AP" -p "YOUR_WIFI_PASSWORD"
   ```
2. Get the device's IP address:
   ```sh
   net iface
   ```
3. Open a web browser and navigate to `http://<device-ip>/`.

The web interface is served from the `/lfs/web/` directory. You can also retrieve sensor logs, e.g., `http://<device-ip>/bme_log`.
