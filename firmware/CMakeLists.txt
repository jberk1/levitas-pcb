# This is the main compiler configuration file for this project.
# This is used both when building via Linux command line or the
# Raspberry Pi Pico VS Code extension on Windows.
#
# See the Raspberry Pi Pico API documentation for details.
#
# Important notes:
#
# - This file must be manually changed to include new source files
#   (via add_executable) and imports (via target_link_libraries).
#   (If using VS Code, you must run "Configure CMake" after this.)
#
# - The one following section is necessary for VS Code:

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.3.1)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================

# All lines below this can be manually configured.

cmake_minimum_required(VERSION 3.13)

# Use the RP2350 on the Pico 2 dev board.
# ("pico2" firmware also runs on a Pico 2 W: same microcontroller and header
# pins. The only difference is the LED; see USE_LED in src/main_v2.c.)
set(PICO_BOARD pico2 CACHE STRING "Board type")
set(PICO_PLATFORM rp2350)

# Find the Pico SDK.
# - With the VS Code extension, PICO_SDK_PATH was set above to the SDK the
#   extension downloaded (~/.pico-sdk/sdk/<sdkVersion>).
# - Otherwise (e.g., Linux command line with no PICO_SDK_PATH environment
#   variable), use the copy of the SDK stored in this git repo.
# (note: this must happen before project())
if(NOT PICO_SDK_PATH AND NOT DEFINED ENV{PICO_SDK_PATH})
    set(PICO_SDK_PATH ${CMAKE_CURRENT_LIST_DIR}/pico-sdk)
endif()
include(pico_sdk_import.cmake)

# define project name
project(levitas C CXX ASM)

# initialize Raspberry Pi Pico SDK
pico_sdk_init()

# ====================================================================================
# Firmware for the first version of the PCB (four FDC2112 sensors, high voltage).
# (build/levitas.uf2, build/levitas.elf)

add_executable(levitas
    # main program
    src/main.c

    # PIO i2c library
    src/pio_i2c.c
    src/pio_i2c.h

    # (to run tests, enable any of the following and disable main.c)
    # test fdc2112 cap sensors with hardware i2c (worse than PIO)
    # src/test_fdc2112_i2c.c
    # test fdc2112 cap sensors with PIO i2c (better than hardware i2c)
    # src/test_fdc2112_pio_i2c.c
    # test mpu6050 accelerometer with hardware i2c
    # src/test_mpu6050_i2c.c
    # test max5715 DAC with hardware spi
    # src/test_max5715_spi.c
)

# take PIO file and generate a header file (to be included in C files)
pico_generate_pio_header(levitas ${CMAKE_CURRENT_LIST_DIR}/src/pio_i2c.pio)

# add libraries (pico_stdlib for general use, etc)
target_link_libraries(levitas
    pico_stdlib
    hardware_i2c
    hardware_spi
    hardware_pio
    pico_multicore
    pico_util
    pico_time
    pico_bootrom
)

# The control loop runs on the second microcontroller core, which by default
# only gets 2kB of stack memory. Give it 4kB (it formats text messages, which
# needs a fair amount). 4kB is the most that fits in the memory area the SDK
# reserves for this stack.
target_compile_definitions(levitas PRIVATE PICO_CORE1_STACK_SIZE=0x1000)

# The Pico SDK restarts into BOOTSEL mode (e.g., when picotool asks it to over
# USB) by calling the functions named below. Make it call our own versions in
# main.c instead ("__wrap_..."), which first set the high voltage to 0V.
# (The second function only exists in newer SDK versions.)
target_link_options(levitas PRIVATE "LINKER:--wrap=rom_reset_usb_boot")
file(READ ${PICO_SDK_PATH}/src/rp2_common/pico_bootrom/include/pico/bootrom.h BOOTROM_H)
if(BOOTROM_H MATCHES "rom_reset_usb_boot_extra")
    target_link_options(levitas PRIVATE "LINKER:--wrap=rom_reset_usb_boot_extra")
    target_compile_definitions(levitas PRIVATE LEVITAS_WRAP_RESET_USB_BOOT_EXTRA=1)
endif()

# UART on pins 0/1 is enabled by default. Disable it.
pico_enable_stdio_uart(levitas 0)

# Enable USB virtual serial port.
# (i.e., appear as a serial port when plugged in over USB, use w/ printf)
# (this is equivalent to including pico_stdio_usb)
# (includes functionality to reset over USB)
pico_enable_stdio_usb(levitas 1)

# create map/bin/hex/uf2 files in addition to ELF
pico_add_extra_outputs(levitas)

# (The firmware for the second version of the PCB, levitas_v2, is a separate
# project in the "firmware_v2" folder.)
