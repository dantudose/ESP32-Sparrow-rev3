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
