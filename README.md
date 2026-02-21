# ESP32-Sparrow rev.3 Board

The ESP32 Sparrow board is an IoT device designed for projects that require low battery consumption. It is built around the ESP32-C6 module, which features Zigbee and Thread compatibility.

## Pinout

![ESP32](./images/ESP32-C6-Sparrow-Pinout_v3-1.png)

## Components

Its other features include:

- **Display:** DFR0647 - Data display (I2C)
- **Memory:** 
    - **Flash:** W25Q512JVEIQ - 64 MB Flash memory (SPI)
    - **SD card** slot for external storage (SPI)
- **Sensors:** 
    - **BME688**  - Temperature, humidity, pressure, and air quality sensor (I2C)
    - **SLSM6DSL** - 3-axis accelerometer and gyroscope (I2C)
    - **LTR303** - Ambient light sensor (I2C)
- **Microphone:** ICS-43434 - High-quality digital microphone (I2S)
- **LED:** WS2812B2020 - Neopixel RGB LED (GPIO)
- **Communication:** PRT-14417 - Qwiic/Stemma QT connector (I2C)
- **Voltage Supervisor:** BD5229G-TR - Shuts down microcontroller below 2.9V
- **Battery Level:** MAX17048 - Measures battery voltage (I2C)
- **USB C** connector for programming/charging
- **Power:** 18650 Battery - Holder on the back

![ESP32](./images/sparrow_rev3.png)

## Repository Contents
* **/hardware** - Fusion360 design files (.brd, .sch) and .pdf schematic
* **/images** - Pinout and board images
* **/software** - Code examples
    * **/arduino-ide** - Simple Arduino project
    * **/platformio** - Test project for PlatformIO
    * **/zephyr** - Test project for Zephyr OS


## Pins for each component

**1. MicroSD Card Holder (SPI)**

| PIN NAME            | GPIO Pin   |
|---------------------|------------|
| SD_CS               | GPIO10     |
| MOSI                | GPIO7      |
| MISO                | GPIO2      |
| SCK                 | GPIO6      |


**2. NOR Flash (SPI)**

| PIN NAME            | GPIO Pin   |
|---------------------|------------|
| FLASH_CS            | GPIO23     |
| MOSI                | GPIO7      |
| MISO                | GPIO2      |
| SCK                 | GPIO6      |


**3. I2C Sensors**

| PIN NAME    | GPIO Pin |
|-------------|----------|
| SCL         | GPIO22   |
| SDA         | GPIO21   |

**4. I2C Bus Addresses**

| CHIP    | Address |
|---------|---------|
| LSM6DSL | 0x6A    |
| BME680  | 0x76    |
| LTR303  | 0x29    |


**5. Neopixel RGB LED**

| PIN NAME | GPIO Pin |
|----------|----------|
| NEOPIXEL | GPIO3    |


**6. Microphone ICS-43434 (I2S)**

| PIN NAME | GPIO Pin |
|----------|----------|
| MIC_WS   | GPIO20   |
| MIC_SCK  | GPIO18   |
| MIC_SD   | GPIO19   |


**7. Battery Charge Level MAX17048**

| PIN NAME | GPIO Pin |
|----------|----------|
| INT_VLT  | GPIO4    |



**8. Other pins**

| PIN NAME               | GPIO Pin |
|------------------------|----------|
| IO / BOOT Button       | GPIO9    |
| UART_TX                | GPIO16   |
| UART_RX                | GPIO17   |
| Accelerometer Interrupt| GPIO11   |
| LTR303 Interrupt       | GPIO15   |


License Information
-------------------

This product is _**open source**_! 

Please review the LICENSE file for license information. 

Distributed as-is; no warranty is given.

