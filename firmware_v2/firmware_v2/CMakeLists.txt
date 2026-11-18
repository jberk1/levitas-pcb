# This is the main compiler configuration file for the firmware of the
# second version of the LeviTAS PCB (levitas_v2: eight FDC2112 sensors).
# It builds build/levitas_v2.uf2 and build/levitas_v2.elf.
# (The firmware for the first version of the PCB is separate; it is in
# the "firmware" folder.)
#
# This is used both when building via Linux command line or the
# Raspberry Pi Pico VS Code extension on Windows.
#
# Important notes:
#
# - This file must be manually changed to include new source files
#   (via add_executable) and imports (via target_link_libraries).
#   (If using VS Code, you must run "Configure CMake" after this.)
#
# - The project name (in project()) and the program name (in
#   add_executable()) are the same on purpose: the VS Code extension's
#   "Run" button loads build/<project name>.elf.

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

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

# Board the firmware is built for: Raspberry Pi Pico 2 W.
# (For a plain Pico 2, change this to pico2. See "Board" in src/main_v2.c.)
set(PICO_BOARD pico2_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
# (Without the VS Code extension, set the PICO_SDK_PATH environment variable
# to a Pico SDK folder, version 2.1.0 or newer. The older copy of the SDK
# in the "firmware" folder does not know the Pico 2 W.)
include(pico_sdk_import.cmake)

project(levitas_v2 C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# define local project files
add_executable(levitas_v2
    # main program
    src/main_v2.c

    # PIO i2c library
    src/pio_i2c.c
    src/pio_i2c.h
)

pico_set_program_name(levitas_v2 "levitas_v2")
pico_set_program_version(levitas_v2 "0.1")

# take PIO file and generate a header file (to be included in C files)
pico_generate_pio_header(levitas_v2 ${CMAKE_CURRENT_LIST_DIR}/src/pio_i2c.pio)

# add libraries (pico_stdlib for general use, etc)
target_link_libraries(levitas_v2
    pico_stdlib
    hardware_pio
    pico_multicore
    pico_util
    pico_time
    pico_bootrom    # restart into BOOTSEL mode from software
)

# UART on pins 0/1 is enabled by default. Disable it (those are sensor pins).
pico_enable_stdio_uart(levitas_v2 0)

# Enable USB virtual serial port.
# (i.e., appear as a serial port when plugged in over USB, use w/ printf)
# (the USB serial port is also what lets new firmware be loaded without
# pressing the BOOTSEL button; see "Loading new firmware" in src/main_v2.c)
pico_enable_stdio_usb(levitas_v2 1)

# create map/bin/hex/uf2 files in addition to ELF
pico_add_extra_outputs(levitas_v2)
