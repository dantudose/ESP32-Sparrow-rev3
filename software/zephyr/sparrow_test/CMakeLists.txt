cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(sparrow_littlefs)

target_sources(app PRIVATE src/main.c src/cfb_font_0508.c src/startup.c src/oled.c src/fs_utils.c src/http_server.c src/net_time.c src/sensors.c src/neopixel.c src/boot.c src/i2c_scan.c)

file(GLOB_RECURSE WEB_ASSETS CONFIGURE_DEPENDS ${CMAKE_CURRENT_LIST_DIR}/web/*)
file(GLOB_RECURSE LFS_ROOT_ASSETS CONFIGURE_DEPENDS ${CMAKE_CURRENT_LIST_DIR}/lfs/*)

set(LFS_IMAGE ${CMAKE_BINARY_DIR}/littlefs.img)
add_custom_command(
  OUTPUT ${LFS_IMAGE}
  COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_LIST_DIR}/scripts/make_littlefs_image.py
          --build-dir ${CMAKE_BINARY_DIR}
          --web-dir ${CMAKE_CURRENT_LIST_DIR}/web
          --output ${LFS_IMAGE}
  DEPENDS ${WEB_ASSETS} ${LFS_ROOT_ASSETS}
          ${CMAKE_CURRENT_LIST_DIR}/scripts/make_littlefs_image.py
  COMMENT "Building LittleFS image from web assets"
  VERBATIM
)
add_custom_target(littlefs_image DEPENDS ${LFS_IMAGE})

zephyr_linker_sources(SECTIONS sections-rom.ld)
