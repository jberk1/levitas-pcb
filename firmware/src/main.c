// This file is the entry point (and main source code file) for the
// current LeviTAS PCB project.
//
// It contains code to read from four FDC2112 capacitance sensors,
// write voltages to a 4-channel MAX5715 DAC (connected to a HV264
// amplifier), and write data out via a serial port (over USB).
//
// It also runs a PD controller that levitates the mass at a chosen height
// between its two stops, and takes commands over the USB serial port
// (type "help" in a serial monitor; see the list of commands further down).

// include necessary standard libraries
#include "pico/stdlib.h"        // standard functions
#include <stdio.h>              // printf
#include "pico/multicore.h"     // run the control loop on the second core
#include "pico/util/queue.h"    // queue for multicore communication
#include "pico/time.h"          // assorted time functions
#include "hardware/spi.h"       // SPI for MAX5715
#include "pico/stdio_usb.h"     // USB connection functions
#include "pico/bootrom.h"       // restart into BOOTSEL mode from software
#include "hardware/sync.h"      // briefly disable interrupts
#include <math.h>               // sqrtf, expf
#include <string.h>             // strcmp, strtok
#include <stdlib.h>             // strtof
#include <stdarg.h>             // printf-style function arguments
// also include our custom I2C PIO library
#include "pio_i2c.h"

// ===========================================================================
// ===========================================================================
// ========== FDC2112 capacitance sensors ====================================
// ===========================================================================
// ===========================================================================
// (This section contains data and functions to use the capacitance sensors.)
// (adapted from test_fdc2112_pio_i2c.c)

// I2C address of all FDC2112 chips (each on its own I2C bus)
#define FDC2112_I2C_ADDR 0x2b
// I2C-accessible registers of FDC2112 chip
#define FDC2112_REG_DATA_CH0 0x00
#define FDC2112_REG_RCOUNT_CH0 0x08
#define FDC2112_REG_OFFSET_CH0 0x0c
#define FDC2112_REG_SETTLECOUNT_CH0 0x10
#define FDC2112_REG_CLOCK_DIVIDERS_CH0 0x14
#define FDC2112_REG_STATUS 0x18
#define FDC2112_REG_ERROR_CONFIG 0x19
#define FDC2112_REG_CONFIG 0x1a
#define FDC2112_REG_MUX_CONFIG 0x1b
#define FDC2112_REG_RESET_DEV 0x1c
#define FDC2112_REG_DRIVE_CURRENT_CH0 0x1e
#define FDC2112_REG_MANUFACTURER_ID 0x7e
#define FDC2112_REG_DEVICE_ID 0x7f
// how long each measurement takes (see fdc2112_startup): (RCOUNT*16+4)/21.7MHz
#define FDC2112_RCOUNT 0x0080

// write a 16-bit value to the given register
void fdc2112_write_register(PIO pio, uint sm, uint8_t reg, uint16_t data){
    uint8_t buf[3];
    buf[0] = reg;
    buf[1] = data >> 8; // send MSB byte first
    buf[2] = data & 0xff; // send LSB byte last
    int err = pio_i2c_write_blocking(pio, sm, FDC2112_I2C_ADDR, buf, 3, false);
}
void fdc2112_write_register_4(PIO pio, uint sm0, uint sm1, uint sm2, uint sm3, uint8_t reg, uint16_t data){
    uint8_t buf[3];
    buf[0] = reg;
    buf[1] = data >> 8; // send MSB byte first
    buf[2] = data & 0xff; // send LSB byte last
    pio_i2c_write_blocking_4(pio, sm0, sm1, sm2, sm3, FDC2112_I2C_ADDR, buf, 3, false);
}

// read a 16-bit value from given register
uint16_t fdc2112_read_register(PIO pio, uint sm, uint8_t reg){
    uint8_t buf_reg[1];
    buf_reg[0] = reg;
    uint8_t buf_data[] = {0x00, 0x00};
    pio_i2c_write_blocking(pio, sm, FDC2112_I2C_ADDR, buf_reg, 1, true); // shouldn't stop
    pio_i2c_read_blocking(pio, sm, FDC2112_I2C_ADDR, buf_data, 2);
    // result is MSB then LSB byte
    return (((uint16_t)buf_data[0]) << 8) | ((uint16_t)buf_data[1]);
}
void fdc2112_read_register_4(PIO pio, uint sm0, uint sm1, uint sm2, uint sm3, uint8_t reg, uint16_t* result){
    uint8_t buf_reg[1];
    buf_reg[0] = reg;
    uint8_t buf_data0[] = {0x00, 0x00};
    uint8_t buf_data1[] = {0x00, 0x00};
    uint8_t buf_data2[] = {0x00, 0x00};
    uint8_t buf_data3[] = {0x00, 0x00};
    pio_i2c_write_blocking_4(pio, sm0, sm1, sm2, sm3, FDC2112_I2C_ADDR, buf_reg, 1, true); // shouldn't stop
    pio_i2c_read_blocking_4(pio, sm0, sm1, sm2, sm3, FDC2112_I2C_ADDR, buf_data0, buf_data1, buf_data2, buf_data3, 2);
    // result is MSB then LSB byte
    result[0] =  (((uint16_t)buf_data0[0]) << 8) | ((uint16_t)buf_data0[1]);
    result[1] =  (((uint16_t)buf_data1[0]) << 8) | ((uint16_t)buf_data1[1]);
    result[2] =  (((uint16_t)buf_data2[0]) << 8) | ((uint16_t)buf_data2[1]);
    result[3] =  (((uint16_t)buf_data3[0]) << 8) | ((uint16_t)buf_data3[1]);
}

// read a 16-bit value (assuming register was previously indicated, e.g., by a full read)
uint16_t fdc2112_read_register_nowrite(PIO pio, uint sm){
    uint8_t buf_data[] = {0x00, 0x00};
    //pio_i2c_write_blocking(pio, sm, FDC2112_I2C_ADDR, buf_reg, 1, true); // shouldn't stop
    pio_i2c_read_blocking(pio, sm, FDC2112_I2C_ADDR, buf_data, 2);
    // result is MSB then LSB byte
    return (((uint16_t)buf_data[0]) << 8) | ((uint16_t)buf_data[1]);
}
void fdc2112_read_register_nowrite_4(PIO pio, uint sm0, uint sm1, uint sm2, uint sm3, uint16_t* result){
    uint8_t buf_data0[] = {0x00, 0x00};
    uint8_t buf_data1[] = {0x00, 0x00};
    uint8_t buf_data2[] = {0x00, 0x00};
    uint8_t buf_data3[] = {0x00, 0x00};
    pio_i2c_read_blocking_4(pio, sm0, sm1, sm2, sm3, FDC2112_I2C_ADDR, buf_data0, buf_data1, buf_data2, buf_data3, 2);
    // result is MSB then LSB byte
    result[0] =  (((uint16_t)buf_data0[0]) << 8) | ((uint16_t)buf_data0[1]);
    result[1] =  (((uint16_t)buf_data1[0]) << 8) | ((uint16_t)buf_data1[1]);
    result[2] =  (((uint16_t)buf_data2[0]) << 8) | ((uint16_t)buf_data2[1]);
    result[3] =  (((uint16_t)buf_data3[0]) << 8) | ((uint16_t)buf_data3[1]);
}

static void fdc2112_startup(PIO pio, uint sm){
    // Chip is in sleep state by default and must be enabled.
    // To do so, set a bit in the CONFIG register. First, set the
    // rest of the configuration while we're at it.

    // ===== Measurement time behavior =====
    //
    // First we set the RCOUNT and SETTLECOUNT registers.
    // These control how long the chip takes to read a given signal
    // (see manual pages 15, 16).
    //
    // If the chip is used to read capacitance from multiple channels,
    // SETTLECOUNT determines the amount of time a measurement is
    // allowed to stabilize after switching channels. This does not
    // apply to us, so we set SETTLECOUNT and otherwise ignore it.
    // (TODO CONFIRM THIS WORKS)
    //
    // The chip takes a configurable amount of time to take a measurement.
    // Longer measurement times give more accurate values. The time is
    // determined by RCOUNT (specifically, (RCOUNT*16+4)/f_{REF0} where
    // f_{REF0} is the reference oscillator frequency, about 40MHz).
    // Measuring n bits of precision requires 2^n = (RCOUNT*16).
    // In our case, we want to take measurements within maybe 10us. We
    // choose RCOUNT=33=0x0021 giving 9.04 bits of precision in 13.us.
    // (Note the minimum is RCOUNT>8, which we satisfy; see page 13)
    //
    // (but register map on page 25 says min is 0x0100 or 0x0080?)
    //
    // UPDATE: the main loop only reads the sensor once every 150us, so a
    // 25us measurement wastes most of that time. For the PD controller we
    // use a longer measurement instead, RCOUNT=0x80 (about 95us, still
    // shorter than one loop), which gives roughly 4x finer readings.
    //fdc2112_write_register(pio, sm, FDC2112_REG_RCOUNT_CH0, 0x0040);
    //fdc2112_write_register(pio, sm, FDC2112_REG_RCOUNT_CH0, 0x0021);
    fdc2112_write_register(pio, sm, FDC2112_REG_RCOUNT_CH0, FDC2112_RCOUNT);
    // determines the sensor activation time
    // must have settlecount > 3
    fdc2112_write_register(pio, sm, FDC2112_REG_SETTLECOUNT_CH0, 0x000a);

    // ===== Small things =====
    //
    // The chip has some error reporting functions; leave them off
    fdc2112_write_register(pio, sm, FDC2112_REG_ERROR_CONFIG, 0x0000);
    //
    // Prepare to only read capacitance from a single channel on the chip
    // (this setting can choose to alternate readings between channels)
    // 0000 0010 0000 1101 = 0x020d
    fdc2112_write_register(pio, sm, FDC2112_REG_MUX_CONFIG, 0x020d);

    // ===== Gain and offset settings =====
    //
    // The following clock divider setting changes some measurement
    // parameters:
    // f_{REF0}=f_{CLK}/CH0_FREQ_DIVIDER
    // f_{IN0}=f_{SENSOR0}/CH0_FIN_SEL
    fdc2112_write_register(pio, sm, FDC2112_REG_CLOCK_DIVIDERS_CH0, 0x2002);
    //
    // ...and the resulting digital data is 16 bits, of which only
    // 12 bits can be read over I2C. However, the data can be
    // bit-shifted and/or given an offset before I2C reading in order
    // to see the LSB (e.g., if there is only a small capacitance
    // change in a relatively large capacitance).
    //
    // set gain to a 4-bit shift (2^4=16x multiplier):
    // (see manual page 35)
    // 0 0000 00 0 | 0000 0000
    //fdc2112_write_register(pio, sm, FDC2112_REG_RESET_DEV, 0x0000); // 0-bit shift
    //fdc2112_write_register(pio, sm, FDC2112_REG_RESET_DEV, 0x0400); // 3-bit shift
    fdc2112_write_register(pio, sm, FDC2112_REG_RESET_DEV, 0x0600); // 4-bit shift
    //
    // offset:
    // also add an offset value in order to fit data within the bits that we're sending back
    // (see manual page 27)
    //fdc2112_write_register(pio, sm, FDC2112_REG_OFFSET_CH0, 0x0000); // no offset
    fdc2112_write_register(pio, sm, FDC2112_REG_OFFSET_CH0, 0x1800); // set offset=0x1800

    // ===== Drive current =====
    //
    // The current the sensor uses to read capacitance can be adjusted.
    // This results in slightly different capacitance readings.
    // Supposedly there's an optimum value that can be found by tweaking
    // until the sensor output drive signal is about 1.8V, but I wasn't
    // able to measureme of the signal (probably too high frequency?).
    //
    // Using the default values (0x0000) seems to work fine.
    //
    // Determines the drive current:
    // (see chart on manual page 36)
    //
    // code  resulting capacitance readings
    // ====================================
    // 00000 0x01b8 0x01d3 0x01d3 0x01c9
    // 00001 0x01ba 0x01d5 0x01d5 0x01cb
    // 00010 0x01bb 0x01d7 0x01d7 0x01cc
    // 00011 0x01bd 0x01d9 0x01d9 0x01cd
    // ...
    // 01001 0x01c3 0x01df 0x01df 0x01d4
    //
    // Result: higher current -> slightly higher reading?
    // (TBD: measure voltage)
    fdc2112_write_register(pio, sm, FDC2112_REG_DRIVE_CURRENT_CH0, 0x0000);

    // ===== Start measurement =====
    //
    // Upon startup, the chip is in a sleep state. Use the following command
    // to start the chip so we can read capacitance. Note that afterward,
    // additional configuration changes might not work until the chip is
    // powered off and on again (which is why we do this last).
    //
    // At the same time, this register also choose which oscillator to use.
    // We will use the chip's internal oscillator (we don't have an external
    // oscillator connected).
    //
    // Set register 0x1a to 00010100 00000001 = 0x1401
    // (sense on channel 0, no sleep mode, full current, internal oscillator)
    // - this also tells the chip to use its internal reference oscillator
    //   - f_{CLK} = f_{INTCLK}
    //   - f_{INTCLK} = internal clock = 43MHz (approximately; ranges 35-55MHz with manufacturing)
    fdc2112_write_register(pio, sm, FDC2112_REG_CONFIG, 0x1401);
}

// ===========================================================================
// ===========================================================================
// ========== MAX5715 DAC ====================================================
// ===========================================================================
// ===========================================================================
// (This section contains data and functions to use the high voltage output.)
// (adapted from test_max5715_spi.c)

// SPI1 CS is GPIO 13 = Pico pin 17
// SPI1 SCLK is GPIO 14 = Pico pin 19
// SPI1 MOSI is GPIO 15 = Pico pin 20
// SPI1 MISO is not used
#define HVAWG_CS_PIN 13
#define HVAWG_SCLK_PIN 14
#define HVAWG_MOSI_PIN 15

// Sending data to DAC requires sending
// this HVAWG_init_command followed by 3 bytes = 24 bits via SPI.
static inline void HVAWG_init_command(){
    // To start command, raise CS for >= 20ns, then lower
    gpio_put(HVAWG_CS_PIN, 1);
    asm volatile("nop \n nop \n nop \n nop"); // assume <= 150MHz clock
    gpio_put(HVAWG_CS_PIN, 0);
}

// Configure DAC.
void HVAWG_reset(){

    // reset all settings to default
    // SW_RESET = 0101 0001 x x = 0x510000
    uint8_t buf_reset[] = {0x51, 0x00, 0x00};
    HVAWG_init_command();
    spi_write_blocking(spi1, buf_reset, 3);

    // set internal reference to always on, 4.096V
    // REF = 0111 0111 x x = 0x770000
    uint8_t buf_ref[] = {0x77, 0x00, 0x00};
    HVAWG_init_command();
    spi_write_blocking(spi1, buf_ref, 3);

    // let all DACs be controlled by LOAD commands
    // (already the default; redundant)
    // CONFIG = 01101000 00001111 x = 0x680f00
    //uint8_t buf_config[] = {0x68, 0x0f, 0x00};
    //HVAWG_init_command();
    //spi_write_blocking(spi1, buf_config, 3);

    // power on DACs
    // (already the default; redundant)
    // POWER = 01000000 00001111 x = 0x400f00
    //uint8_t buf_power[] = {0x40, 0x0f, 0x00};
    //HVAWG_init_command();
    //spi_write_blocking(spi1, buf_power, 3);
}

// Write values to high voltage DAC. Values 1-4 are mapped to
// HV1, HV2, HV3, HV4 on the PCB, respectively.
//
// Values are integers from 0 to 4096-1. This function is separate
// from HVAWG_write_voltage() for convenience because it can
// (approximately) guarantee that different inputs will result in
// different outputs, whereas two sufficiently close voltages might
// have the same integer code and thus same DAC output.
void HVAWG_write_count(uint val1, uint val2, uint val3, uint val4){

        // confirm there are only 12 bits to avoid
        // accidentally overwriting parts of SPI command
        if(val1 > 4095){ val1 = 4095; }
        if(val2 > 4095){ val2 = 4095; }
        if(val3 > 4095){ val3 = 4095; }
        if(val4 > 4095){ val4 = 4095; }

        // CODEn = 00000001 11111111 11110000 = 0x01fff0
        // CODEn = 00000010 11111111 11110000 = 0x02fff0
        // CODEn = 00000100 11111111 11110000 = 0x04fff0
        // CODEn_LOAD_ALL = 00101000 11111111 11110000 = 0x28fff0

        // WARNING: SENSORS SDA1,2,3,4 CORRESPOND TO HIGH VOLTAGE
        // OUTPUTS HV4,3,2,1 on PCB (NUMBERS ARE BACKWARD) (AND DAC
        // SPI ALSO HAS DIFFERENT WIRING).
        // Note that DAC channel 1,2,3,4 are labelled on the PCB
        // as high voltage outputs 4,3,2,1. Write the following data
        // such that HVAWG_write_count(10,0,0,0) will write a high
        // voltage to PCB HV1, for example.

        // use CODEn commands (see manual page 18) to
        // load values into first three DAC registers but without
        // updating DAC output yet
        HVAWG_init_command();
        uint8_t buf1[] = {0x03, val1 >> 4, (val1 & 0xf)<<4};
        spi_write_blocking(spi1, buf1, 3);
        HVAWG_init_command();
        uint8_t buf2[] = {0x02, val2 >> 4, (val2 & 0xf)<<4};
        spi_write_blocking(spi1, buf2, 3);
        HVAWG_init_command();
        uint8_t buf3[] = {0x01, val3 >> 4, (val3 & 0xf)<<4};
        spi_write_blocking(spi1, buf3, 3);

        // use CODEn_LOAD_ALL command (see manual page 18) to
        // load in value for the fourth and last channel, then
        // update the DAC output on all four channels at once
        HVAWG_init_command();
        uint8_t buf4[] = {0x20, val4 >> 4, (val4 & 0xf)<<4};
        spi_write_blocking(spi1, buf4, 3);
}

void HVAWG_write_voltage(float V1, float V2, float V3, float V4){
    // The DAC takes a 12-bit input command,
    // where the minimum (0b000000000000) outputs 0V
    // and the maximum (0b111111111111) outputs 4.096V
    // (as determined by the internal reference in HVAWG_reset().)
    // This is amplified approximately 66.7x by the HV264 amplifier.
    // The final output voltage is capped by the power supply,
    // which is usually about 240V but can dip to 220-230V when
    // driving many capacitive sensors at high frequency (which
    // requires greater current). Note this means it is possible
    // to tell the DAC to output a voltage higher than the amplifier
    // and power supply will do (i.e., 4.096*66.7=273V), in which
    // case the output will be capped to the power supply voltage.

    // convert voltage to DAC value (integer from 0 to 2^12-1):
    // divide by 66.7x HV264 gain, 4.096V DAC reference voltage,
    // multiply by 2^12 bits = 4096, and convert to integer.
    uint value1 = (uint)(V1/66.7/4.096*4096);
    uint value2 = (uint)(V2/66.7/4.096*4096);
    uint value3 = (uint)(V3/66.7/4.096*4096);
    uint value4 = (uint)(V4/66.7/4.096*4096);

    // write value to HVAWG
    HVAWG_write_count(value1, value2, value3, value4);
}

// Set all four high voltage outputs to the same voltage.
// (Only talks to the DAC when the value actually changes.)
static bool hv_ready = false;          // true once SPI and the DAC are set up
static uint hv_last_code = 0xffffffff; // forces the first DAC write
static void HVAWG_write_all(float V){
    uint code = (uint)(V/66.7f/4.096f*4096.0f + 0.5f);
    if(code != hv_last_code){
        HVAWG_write_count(code, code, code, code);
        hv_last_code = code;
    }
}

// Set all high voltage outputs to 0V right now, no matter what the rest of
// the program is doing. Used just before restarting into BOOTSEL mode.
// (Safe to call from an interrupt, e.g., from the USB code.)
static void HVAWG_force_zero(){
    if(!hv_ready){ return; }
    uint32_t irq_state = save_and_disable_interrupts();
    // if the main loop was in the middle of sending a command, let the byte
    // currently being sent finish (HVAWG_init_command then starts a new one)
    for(int i=0; i<10000 && spi_is_busy(spi1); i++){ }
    HVAWG_write_count(0, 0, 0, 0);
    hv_last_code = 0;
    restore_interrupts(irq_state);
}

// ===========================================================================
// ===========================================================================
// ========== Loading new firmware without the BOOTSEL button ================
// ===========================================================================
// ===========================================================================
// To load new firmware, the Pico has to be in BOOTSEL mode. Instead of
// unplugging USB and holding the BOOTSEL button, this program can be told
// to restart into BOOTSEL mode over USB. There are three ways:
//
// 1. picotool with the -f option ("picotool load -f -x build/levitas.uf2").
//    This is what "Run Project (USB)" in VS Code does. It works because
//    pico_enable_stdio_usb() in CMakeLists.txt adds a hidden USB "reset"
//    interface to this program.
//
// 2. Opening the serial port at 1200 baud (and closing it again).
//    Also provided by pico_enable_stdio_usb().
//
// 3. Sending the word "bootsel" over the serial port, e.g., by typing it
//    into the VS Code serial monitor (see the command list below).
//
// The DAC keeps its last output for as long as it has power, including while
// the Pico sits in BOOTSEL mode. So that high voltage is never left on while
// new firmware is loaded, all three ways go through the functions below,
// which set the high voltage to 0V first. (1) and (2) are inside the Pico
// SDK; the "wrap" lines in CMakeLists.txt make the SDK call these functions
// instead of its own, and __real_... is the SDK's original function.
//
// One complication: the DAC is driven by the control loop on the second
// core, while (1) and (2) happen on the first core. Two cores must not talk
// to the DAC at the same time, so in that case we ask the control loop to
// switch the high voltage off and stop (it checks reboot_requested on every
// pass), and wait for it to say it has done so.
static volatile bool reboot_requested = false;        // "control loop: switch high voltage off and stop"
static volatile bool reboot_hv_is_off = false;        // "done"
static volatile bool control_loop_running = false;
static void hv_off_before_reboot(){
    if(get_core_num() == 1 || !control_loop_running){
        // we are the control loop (or it hasn't started yet): do it directly
        HVAWG_force_zero();
        return;
    }
    reboot_requested = true;
    absolute_time_t give_up_time = make_timeout_time_ms(20); // (the control loop answers within 0.15ms)
    while(!reboot_hv_is_off && !time_reached(give_up_time)){
        tight_loop_contents();
    }
}
void __attribute__((noreturn)) __real_rom_reset_usb_boot(uint32_t usb_activity_gpio_pin_mask, uint32_t disable_interface_mask);
void __attribute__((noreturn)) __wrap_rom_reset_usb_boot(uint32_t usb_activity_gpio_pin_mask, uint32_t disable_interface_mask){
    hv_off_before_reboot();
    __real_rom_reset_usb_boot(usb_activity_gpio_pin_mask, disable_interface_mask);
}
#if LEVITAS_WRAP_RESET_USB_BOOT_EXTRA // (newer SDK versions use this one; see CMakeLists.txt)
void __attribute__((noreturn)) __real_rom_reset_usb_boot_extra(int usb_activity_gpio_pin, uint32_t disable_interface_mask, bool usb_activity_gpio_pin_active_low);
void __attribute__((noreturn)) __wrap_rom_reset_usb_boot_extra(int usb_activity_gpio_pin, uint32_t disable_interface_mask, bool usb_activity_gpio_pin_active_low){
    hv_off_before_reboot();
    __real_rom_reset_usb_boot_extra(usb_activity_gpio_pin, disable_interface_mask, usb_activity_gpio_pin_active_low);
}
#endif

// ===========================================================================
// ===========================================================================
// ========== Settings =======================================================
// ===========================================================================
// ===========================================================================

// Length of one pass of the main loop (read sensors, compute, set voltage).
// WARNING: THE MINIMUM LOOP TIME IS FIXED BY CAPACITIVE SENSOR READS AND
// HIGH VOLTAGE OUTPUTS. SETTING A LOWER LOOP TIME HERE WON'T DO ANYTHING.
// (the "status" command reports how many loops ran late)
#define LOOP_TIME_US 150
#define LOOP_DT (LOOP_TIME_US*1e-6f) // same, in seconds
#define MS_TO_LOOPS(ms) ((uint32_t)((ms)*1000/LOOP_TIME_US))

// The program never commands more than this voltage, whatever it is told.
#define HV_ABS_MAX_V 240.0f

// ----- sensor reading clean-up -----
// readings above this are failed reads (the sensor returns 0x0fff)
#define SENSOR_INVALID_ABOVE 0x07d0
// a reading that jumps by more than this many counts in one loop is treated
// as a glitch and ignored...
#define SENSOR_SPIKE_COUNTS 40
// ...unless it happens this many times in a row (then it's a real change)
#define SENSOR_SPIKE_MAX_RUN 3
// turn high voltage off if a sensor gives no valid reading for this long
#define SENSOR_DEAD_LOOPS MS_TO_LOOPS(30)

// ----- calibration ("cal" command) -----
#define CAL_SETTLE_LOOPS MS_TO_LOOPS(300)  // wait for the mass to settle
#define CAL_AVERAGE_LOOPS MS_TO_LOOPS(200) // then average readings this long
// (same two, but at the top stop with voltage on: kept short because the
// mass tends to stick to the top stop)
#define CAL_TOP_SETTLE_LOOPS MS_TO_LOOPS(200)
#define CAL_TOP_AVERAGE_LOOPS MS_TO_LOOPS(100)
#define CAL_RAMP_V_PER_S 100.0f            // how fast the voltage is ramped
#define CAL_MOVE_COUNTS 20.0f // (ramp up) mass has left the bottom when the reading of EVERY sensor changed this much
#define CAL_MIN_SPAN 30.0f    // every sensor must change at least this many counts between the two stops
// (at the end) how long to wait at 0V for the mass to drop back down before
// warning that it is stuck (it normally clings to the top stop for about 1s)
#define CAL_RELEASE_LOOPS MS_TO_LOOPS(4000)

// ----- controller ("start" command) -----
// how fast the target height moves to a new setpoint (fraction of travel per second)
#define SETPOINT_SLEW_PER_S 2.0f
// stop if the mass sits at the top with 0V, or at the bottom with full
// voltage, for this long (it is probably physically stuck)
// (at the top it normally lets go by itself about 1s after the voltage is
// removed, so wait longer than that)
#define STUCK_TOP_LOOPS MS_TO_LOOPS(3000)
#define STUCK_BOTTOM_LOOPS MS_TO_LOOPS(1000)

// ===========================================================================
// ===========================================================================
// ========== Communication between the two microcontroller cores ============
// ===========================================================================
// ===========================================================================
// The Pico has two cores. One runs the control loop (control_main) and must
// never wait for USB. The other does everything USB (usb_main): it prints the
// data, prints messages, and reads commands typed by the user. They talk
// through three queues.
//
// The control loop is on the second core (core 1) and USB on the first
// (core 0), not the other way round, because the Pico SDK handles USB
// interrupts on core 0. With the control loop there too, about 5% of loops
// were delayed by USB traffic (by up to 0.4ms).

// what the main loop is currently doing (printed as a number in the data)
enum {
    MODE_IDLE = 0,   // high voltage off
    MODE_MANUAL = 1, // fixed voltage set with the "hv" command
    MODE_CAL = 2,    // calibration in progress
    MODE_RUN = 3,    // PD controller running
};

// control core -> USB core: one line of data to print
typedef struct {
    uint64_t t_us;    // time of the sensor reading (microseconds since startup)
    uint16_t data[4]; // four capacitance sensor readings
    float hv;         // voltage commanded on all four high voltage outputs
    float z;          // height as a fraction of travel (0=bottom stop, 1=top stop)
    float z_ref;      // height the controller is aiming for
    uint8_t mode;     // one of MODE_...
} telemetry_t;
queue_t data_queue;

// control core -> USB core: a text message to print (as a "# ..." line)
typedef struct {
    char text[120];
} message_t;
queue_t message_queue;

// USB core -> control core: a command typed by the user
enum {
    CMD_BOOTSEL, CMD_CAL, CMD_START, CMD_STOP, CMD_STATUS, CMD_HV,
    CMD_SP, CMD_KP, CMD_KD, CMD_V0, CMD_V1, CMD_VMAX, CMD_FC, CMD_STREAM,
};
typedef struct {
    uint8_t id;  // one of CMD_...
    float value; // the number typed after the command, if any
} command_t;
queue_t command_queue;

// List of commands. Each is typed as one line over the USB serial port,
// e.g. "kp 3.5" (upper or lower case).
static const struct {
    const char* name;
    uint8_t id;
    bool needs_value;
    const char* help;
} COMMANDS[] = {
    {"bootsel", CMD_BOOTSEL, false, "high voltage off, restart into BOOTSEL mode to load new firmware"},
    {"cal",     CMD_CAL,     false, "calibrate: find sensor readings with the mass at the bottom and top stops"},
    {"start",   CMD_START,   false, "start the PD controller (needs cal first)"},
    {"stop",    CMD_STOP,    false, "high voltage off"},
    {"status",  CMD_STATUS,  false, "print all settings and calibration values"},
    {"hv",      CMD_HV,      true,  "apply a fixed voltage to all four outputs, e.g. hv 100 (hv 0 = off)"},
    {"sp",      CMD_SP,      true,  "setpoint: target height in % of travel (0=bottom stop, 100=top stop)"},
    {"kp",      CMD_KP,      true,  "proportional gain (force in units of the weight of the mass, per full travel of height error)"},
    {"kd",      CMD_KD,      true,  "derivative gain (same force units, per full travel per second; i.e. in seconds)"},
    {"v0",      CMD_V0,      true,  "voltage that balances gravity with the mass at the bottom stop (set by cal)"},
    {"v1",      CMD_V1,      true,  "voltage that balances gravity with the mass at the top stop (set by cal)"},
    {"vmax",    CMD_VMAX,    true,  "highest voltage to use (limited to 240)"},
    {"fc",      CMD_FC,      true,  "cutoff frequency in Hz of the low-pass filter on the height measurement"},
    {"stream",  CMD_STREAM,  true,  "print every Nth data line (1 = all, 0 = none)"},
};
#define NUM_COMMANDS (sizeof(COMMANDS)/sizeof(COMMANDS[0]))

// (control core) queue a message to be printed by the USB core.
// Used like printf. Does not wait; the message is dropped if the queue is full.
static void say(const char* format, ...){
    message_t message;
    va_list args;
    va_start(args, format);
    vsnprintf(message.text, sizeof(message.text), format, args);
    va_end(args);
    queue_try_add(&message_queue, &message);
}

// (USB core) act on one line typed by the user
static void handle_command_line(char* line){
    char* name = strtok(line, " \t");
    char* value = strtok(NULL, " \t");
    if(name == NULL){ return; }
    if(strcmp(name, "help") == 0){
        printf("# commands (type one per line):\n");
        for(uint i=0; i<NUM_COMMANDS; i++){
            printf("#   %-7s %s %s\n", COMMANDS[i].name, COMMANDS[i].needs_value ? "<x>" : "   ", COMMANDS[i].help);
        }
        printf("# data lines: time_us, 4x sensor reading, 4x voltage, height %%, target height %%, mode\n");
        printf("#   (mode: 0=idle 1=fixed voltage 2=calibrating 3=PD controller; height is -1 before cal)\n");
        return;
    }
    for(uint i=0; i<NUM_COMMANDS; i++){
        if(strcmp(name, COMMANDS[i].name) == 0){
            if(COMMANDS[i].needs_value && value == NULL){
                printf("# %s needs a number, e.g. \"%s 1\"\n", name, name);
                return;
            }
            command_t command = {COMMANDS[i].id, value ? strtof(value, NULL) : 0.0f};
            if(!queue_try_add(&command_queue, &command)){
                printf("# busy, try again\n");
            }
            return;
        }
    }
    printf("# unknown command \"%s\" (type help)\n", name);
}

// (USB core) collect characters received over USB serial into lines.
// Does not wait: only looks at characters that have already arrived.
static void check_for_commands(){
    static char line[48];
    static uint length = 0;
    int c;
    while((c = getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT){
        if(c == '\r' || c == '\n'){
            line[length] = 0;
            length = 0;
            handle_command_line(line);
        }else if(length < sizeof(line)-1){
            if(c >= 'A' && c <= 'Z'){ c += 'a' - 'A'; } // make lower case
            line[length] = (char)c;
            length += 1;
            // "bootsel" works even if the serial monitor doesn't send an
            // end-of-line after it
            if(length == 7 && memcmp(line, "bootsel", 7) == 0){
                line[length] = 0;
                length = 0;
                handle_command_line(line);
            }
        }else{
            length = 0; // line too long; throw it away
        }
    }
}

// This function runs on the first microcontroller core (core 0).
// It will, in an infinite loop, print data and messages out over USB serial
// and read commands coming in.
// - Printing over USB takes time (maybe 20us per printf, plus USB interrupts)
// - If we put this in the sensor read loop it would mess with the timing
// - So the sensor read loop runs on the other core instead (see control_main)
static void usb_main(){
    bool was_connected = false;
    while(true){
        bool did_something = false;

        // greet each program that opens the serial port
        bool connected = stdio_usb_connected();
        if(connected && !was_connected){
            printf("# levitas v1 firmware with PD levitation control. Type help for commands.\n");
        }
        was_connected = connected;

        // print messages from the control core
        message_t message;
        while(queue_try_remove(&message_queue, &message)){
            printf("# %s\n", message.text);
            did_something = true;
        }

        // print data from the control core:
        // - microseconds of time since program started,
        // - (4x) capacitance sensor data values (as a hexadecimal integer)
        //       (roughly speaking, higher values = lower capacitance,
        //        but the exact correlation between data value and capacitance
        //        is inexact and somewhat unknown due to large variances in
        //        part tolerances (FDC2112 oscillator frequency, and the
        //        tolerance of the capacitor and inductor in the circuit) as well
        //        as FDC2112 sensor drift (the values sometimes change slightly
        //        over time for an unknown reason)
        // - (4x) high voltage that microcontroller thinks it is applying
        //       (note actual value, measured via oscilloscope, might be
        //        slightly different because of rise times, power supply
        //        max current limits, etc; it's good to check)
        // - height of the mass in % of its travel (-1 before calibration)
        // - height the controller is aiming for in %
        // - what the main loop is doing (MODE_... number)
        // WARNING: SENSORS SDA1,2,3,4 CORRESPOND TO HIGH VOLTAGE
        // OUTPUTS HV4,3,2,1 on PCB (NUMBERS ARE BACKWARD) (AND DAC
        // SPI ALSO HAS DIFFERENT WIRING).
        telemetry_t d;
        if(queue_try_remove(&data_queue, &d)){
            // (printed with whole numbers only, e.g. 123.45 as "123" "." "45",
            // because printf is several times slower with decimal numbers
            // and could then not keep up with the main loop)
            int hv = (int)(d.hv*100.0f + 0.5f);          // in 0.01 V
            int z = (int)lroundf(d.z*10000.0f);          // in 0.01 %
            int z_ref = (int)(d.z_ref*10000.0f + 0.5f);  // in 0.01 %
            const char* z_sign = z < 0 ? "-" : "";
            if(z < 0){ z = -z; }
            printf("%llu 0x%04x 0x%04x 0x%04x 0x%04x %d.%02d %d.%02d %d.%02d %d.%02d %s%d.%02d %d.%02d %u\n", d.t_us,
                d.data[0], d.data[1], d.data[2], d.data[3],
                hv/100, hv%100, hv/100, hv%100, hv/100, hv%100, hv/100, hv%100,
                z_sign, z/100, z%100, z_ref/100, z_ref%100, d.mode
            );
            did_something = true;
        }

        check_for_commands();

        if(!did_something){
            sleep_us(20);
        }
    }
}

// ===========================================================================
// ===========================================================================
// ========== Main code ======================================================
// ===========================================================================
// ===========================================================================
// (This section contains the main function.)

static inline float clampf(float x, float low, float high){
    return x < low ? low : (x > high ? high : x);
}

// This is the control loop. It runs on the second microcontroller core
// (core 1; it is started by main() at the bottom of this file).
// - initializes FDC2112 capacitance sensors
// - initialize MAX5715 DAC for high voltage output
// - then runs main loop
//   - read from capacitance sensors
//   - work out the height of the mass
//   - set high voltage (calibration, or PD controller)
//   - send data to the other core to be printed with USB
//   - try to run at a fixed frequency
//
// High voltage stays at 0V until a command is received over USB. To levitate:
//   cal      (lifts the mass to the top stop and back to calibrate the sensors)
//   start    (starts the PD controller)
//   stop
//
// ===== How the PD controller works =====
// Only the height of the mass is controlled, not its tilt: the same voltage
// goes to all four electrodes, and the height is the average of the four
// sensors.
//
// The electrodes pull the mass up with a force proportional to
// (voltage / gap)^2. So the closer the mass gets to the electrodes, the
// stronger a given voltage pulls: with a fixed voltage the mass either falls
// or snaps all the way up. The voltage that exactly cancels the weight of the
// mass is proportional to the gap; we approximate it with a straight line
// from "v0" at the bottom stop to "v1" at the top stop:
//   v_weight(z) = v0 + (v1 - v0)*z         (z = height, 0=bottom, 1=top)
// The controller decides how much upward force it wants, in units of the
// weight of the mass:
//   force = 1 + kp*(z_target - z) - kd*(speed of z)
// (1 = hold the weight; the kp term pushes toward the target height; the kd
// term brakes the motion) and because force goes as voltage squared, applies
//   voltage = v_weight(z) * sqrt(force)
// v0 and v1 are measured by "cal" (the voltages at which the mass leaves the
// bottom and top stops). They are only approximate (the mass tends to stick
// to the stops; if it is still stuck to the top stop at 0V, v1 is set to 0),
// which mostly shows up as the mass hovering a bit away from the target
// height; adjust v0/v1 to trim that.
//
// NOTE: on this device the voltage moves the mass toward the sense
// electrodes (the readings go down), whichever way up the board is. "Top
// stop" in this file means the stop on that side. So the board has to be
// mounted with the sense electrodes above the mass (upside down compared to
// the other way) for the voltage to work against gravity.
static void control_main() {

    // set up MAX5715 DAC and set high voltage to zero
    // (do this first: if the Pico was restarted while high voltage was on,
    // the DAC is still outputting its old value)
    spi_init(spi1, 5000*1000);
    gpio_set_function(HVAWG_SCLK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(HVAWG_MOSI_PIN, GPIO_FUNC_SPI);
    gpio_init(HVAWG_CS_PIN);
    gpio_set_dir(HVAWG_CS_PIN, GPIO_OUT);
    gpio_put(HVAWG_CS_PIN, 0); // enable MAX5715 SPI (its CS pin is active low)
    sleep_ms(1);
    HVAWG_reset();
    HVAWG_write_voltage(0,0,0,0);
    hv_ready = true;

    // set up LED (on while high voltage is on)
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, 0);

    // set up FDC2112 sensors
    PIO pio_0 = pio0; // choose a PIO

    uint sm0 = 0; // choose PIO state machine
    uint sm1 = 1; // choose PIO state machine
    uint sm2 = 2; // choose PIO state machine
    uint sm3 = 3; // choose PIO state machine
    uint offset0 = pio_add_program(pio_0, &pio_i2c_program); // load program into PIO memory, return instruction memory offset it is loaded at

    pio_i2c_program_init(pio_0, sm0, offset0, 4, 5); // start PIO state machine at given instruction offset with given SDA and SCL pin
    pio_i2c_program_init(pio_0, sm1, offset0, 6, 7); // start PIO state machine at given instruction offset with given SDA and SCL pin
    pio_i2c_program_init(pio_0, sm2, offset0, 8, 9); // start PIO state machine at given instruction offset with given SDA and SCL pin
    pio_i2c_program_init(pio_0, sm3, offset0, 10, 11); // start PIO state machine at given instruction offset with given SDA and SCL pin

    // wait a bit just in case fdc2112 sensors need time to initialize
    sleep_ms(500);
    // send configuration information to sensor
    fdc2112_startup(pio_0, sm0);
    fdc2112_startup(pio_0, sm1);
    fdc2112_startup(pio_0, sm2);
    fdc2112_startup(pio_0, sm3);

    // we'll use this array to store data we read from the sensors
    uint16_t result0[] = {0x00, 0x00, 0x00, 0x00};

    // Make a first reading from the sensors using fdc2112_read_register_4().
    // This is necessary because reading data from the sensors requires:
    //   1. Telling sensor which of its data we want to read (specify register)
    //   2. Actually reading data
    // Later, in the main loop, we'll only do (2) and ignore (1) in order to
    // read data more quickly (via the read_register_nowrite() function).
    // However, we still need to do (1) at some point before that. This
    // function call does both (1) and (2).

    fdc2112_read_register_4(pio0, sm0, sm1, sm2, sm3, FDC2112_REG_DATA_CH0, result0);

    // In case the DAC was still powering up the first time (it needs 200us
    // after power is applied), configure it again and set high voltage to zero.
    HVAWG_reset();
    HVAWG_write_voltage(0,0,0,0);
    hv_last_code = 0;

    // We'll use these variables to keep main loop running
    // at a fixed frequency (determined by LOOP_TIME_US).
    absolute_time_t loop_start_time = get_absolute_time();
    uint32_t late_loops = 0; // how many loops took too long (for "status")
    uint32_t work_us_max = 0; // longest and total time spent working in a loop
    uint64_t work_us_sum = 0; // (i.e. not counting the wait at the end)
    uint32_t work_loops = 0;

    // ===== settings that can be changed with commands =====
    // Gains, tuned on the real device (2026-11-18: about 100mg mass, about
    // 80um gap, in air, board upside down so the voltage lifts the mass).
    // What the measurements showed:
    // - The air squeezed between the mass and the plates damps the motion
    //   heavily: at a fixed voltage the mass takes about 0.1s to cross from
    //   one stop to the other. So the controller needs very little damping
    //   of its own, and everything is slow compared with the 6.7kHz loop.
    // - Levitation was stable for every kp tried from 2 to 20 and kd from 0
    //   to 0.004. Higher kp makes the mass hover closer to the target height
    //   (a PD controller always leaves a small offset: about 4% of travel
    //   at kp=10, 2% at kp=20).
    // - Too much kd (0.010) made the voltage jump by tens of volts from one
    //   loop to the next, because the kd term amplifies sensor noise, and
    //   such fast voltage jumps also upset the sensors. Hence the small kd
    //   and the low filter cutoff frequency below.
    // - With these values the height holds to about 0.1% of travel and the
    //   voltage to about 1V.
    float kp = 10.0f;       // proportional gain
    float kd = 0.001f;      // derivative gain
    float setpoint = 0.5f;  // target height (fraction of travel)
    float v0 = 210.0f;      // voltage balancing gravity at the bottom stop
    float v1 = 10.0f;       // voltage balancing gravity at the top stop
    float vmax = HV_ABS_MAX_V; // highest voltage to use
    float fc = 100.0f;      // low-pass filter cutoff frequency (Hz)
    float filter_alpha = 1.0f - expf(-2.0f*3.14159265f*fc*LOOP_DT);
    uint32_t stream_every = 1; // print every Nth data line

    // ===== sensor state =====
    uint16_t good[4] = {0, 0, 0, 0};       // last accepted reading of each sensor
    uint32_t spike_run[4] = {0, 0, 0, 0};  // how many jumps in a row were ignored
    uint32_t invalid_run[4] = {0, 0, 0, 0};// how many failed reads in a row
    uint32_t spike_count = 0;              // total ignored readings (for "status")
    bool have_reading = false;
    float filtered[4] = {0, 0, 0, 0};      // low-pass filtered readings

    // ===== calibration =====
    bool cal_valid = false;
    float cal_bottom[4] = {0, 0, 0, 0}; // reading with mass on the bottom stop
    float cal_span[4] = {1, 1, 1, 1};   // how much the reading drops going to the top stop
    // progress of the "cal" command
    enum {CAL_REST_SETTLE, CAL_REST_AVERAGE, CAL_RAMP_UP, CAL_TOP_SETTLE,
          CAL_TOP_AVERAGE, CAL_RAMP_DOWN, CAL_END_SETTLE};
    int cal_step = CAL_REST_SETTLE;
    uint32_t cal_loops = 0;         // loops spent in this step
    float cal_sum[4] = {0, 0, 0, 0};
    float cal_lift_v = 0;           // voltage at which the mass left the bottom stop
    float cal_release_v = 0;        // voltage at which the mass left the top stop
    float cal_hold_v = 0;           // voltage used to hold the mass at the top stop
    bool cal_lift_seen = false;
    bool cal_release_seen = false;

    // ===== controller state =====
    uint8_t mode = MODE_IDLE;
    float hv = 0;            // voltage we are applying
    float manual_v = 0;      // voltage for MODE_MANUAL
    float z = -0.01f;        // height, fraction of travel (-0.01 = not calibrated, printed as -1)
    float z_previous = 0;
    float z_speed = 0;       // filtered speed of z (fraction of travel per second)
    float z_target = 0;      // moves smoothly toward setpoint
    uint32_t stuck_top_loops = 0;
    uint32_t stuck_bottom_loops = 0;
    uint32_t loop_count = 0;

    control_loop_running = true;
    while(true){

        // ===== about to restart into BOOTSEL mode? =====
        // (see hv_off_before_reboot)
        if(reboot_requested){
            HVAWG_force_zero();
            reboot_hv_is_off = true;
            while(true){ tight_loop_contents(); } // wait for the restart
        }

        // ===== read capacitance sensors =====
        // WARNING: SENSORS SDA1,2,3,4 CORRESPOND TO HIGH VOLTAGE
        // OUTPUTS HV4,3,2,1 on PCB (NUMBERS ARE BACKWARD) (AND DAC
        // SPI ALSO HAS DIFFERENT WIRING).
        // read quickly from the chip registers
        uint64_t t_loop_begin_us = to_us_since_boot(get_absolute_time());
        fdc2112_read_register_nowrite_4(pio_0, sm0, sm1, sm2, sm3, result0);
        uint64_t t_us = to_us_since_boot(get_absolute_time());

        // throw away bad readings (keep the previous good one instead):
        // failed reads, and single readings that jump implausibly far
        bool sensor_dead = false;
        for(int i=0; i<4; i++){
            uint16_t r = result0[i];
            if(r > SENSOR_INVALID_ABOVE){
                invalid_run[i] += 1;
                if(invalid_run[i] > SENSOR_DEAD_LOOPS){ sensor_dead = true; }
                continue;
            }
            invalid_run[i] = 0;
            int jump = (int)r - (int)good[i];
            if(jump < 0){ jump = -jump; }
            if(have_reading && jump > SENSOR_SPIKE_COUNTS && spike_run[i] < SENSOR_SPIKE_MAX_RUN){
                spike_run[i] += 1;
                spike_count += 1;
                continue;
            }
            spike_run[i] = 0;
            good[i] = r;
        }
        if(!have_reading){
            for(int i=0; i<4; i++){ filtered[i] = good[i]; }
            have_reading = true;
        }

        // low-pass filter the readings, then work out height and speed
        for(int i=0; i<4; i++){
            filtered[i] += filter_alpha*((float)good[i] - filtered[i]);
        }
        if(cal_valid){
            z = 0;
            for(int i=0; i<4; i++){
                z += 0.25f*(cal_bottom[i] - filtered[i])/cal_span[i];
            }
            z_speed += filter_alpha*((z - z_previous)/LOOP_DT - z_speed);
            z_previous = z;
        }else{
            z = -0.01f;
            z_speed = 0;
        }

        // ===== handle one command from the user, if there is one =====
        command_t command;
        if(queue_try_remove(&command_queue, &command)){
            float x = command.value;
            switch(command.id){
            case CMD_BOOTSEL:
                HVAWG_force_zero();
                say("high voltage off, restarting into BOOTSEL mode");
                sleep_ms(100); // give the message time to be sent
                // (0, 0) = no activity LED, keep both USB drive and picotool interfaces
                reset_usb_boot(0, 0); // does not return
                break;
            case CMD_STOP:
                if(mode == MODE_CAL){ cal_valid = false; } // unfinished calibration
                mode = MODE_IDLE;
                say("stopped, high voltage off");
                break;
            case CMD_HV:
                if(mode == MODE_CAL || mode == MODE_RUN){
                    say("busy: type stop first");
                }else{
                    manual_v = clampf(x, 0, vmax);
                    mode = manual_v > 0 ? MODE_MANUAL : MODE_IDLE;
                    say("fixed voltage %.1f V", manual_v);
                }
                break;
            case CMD_CAL:
                if(mode == MODE_RUN){
                    say("busy: type stop first");
                }else{
                    mode = MODE_CAL;
                    cal_valid = false;
                    cal_step = CAL_REST_SETTLE;
                    cal_loops = 0;
                    say("calibrating (takes 6 to 10 seconds)...");
                }
                break;
            case CMD_START:
                if(!cal_valid){
                    say("not calibrated: type cal first");
                }else if(mode == MODE_CAL){
                    say("busy: calibration still running");
                }else{
                    mode = MODE_RUN;
                    z_target = clampf(z, 0, 1); // start from where the mass is
                    stuck_top_loops = 0;
                    stuck_bottom_loops = 0;
                    say("PD controller started: sp=%.1f%% kp=%.3f kd=%.5f v0=%.1f v1=%.1f", setpoint*100, kp, kd, v0, v1);
                }
                break;
            case CMD_SP: setpoint = clampf(x/100.0f, 0.02f, 0.98f); say("sp = %.1f %%", setpoint*100); break;
            case CMD_KP: kp = x < 0 ? 0 : x; say("kp = %.3f", kp); break;
            case CMD_KD: kd = x < 0 ? 0 : x; say("kd = %.5f", kd); break;
            case CMD_V0: v0 = clampf(x, 0, HV_ABS_MAX_V); say("v0 = %.1f V", v0); break;
            case CMD_V1: v1 = clampf(x, 0, HV_ABS_MAX_V); say("v1 = %.1f V", v1); break;
            case CMD_VMAX: vmax = clampf(x, 0, HV_ABS_MAX_V); say("vmax = %.1f V", vmax); break;
            case CMD_FC:
                fc = clampf(x, 1.0f, 0.5e6f/LOOP_TIME_US); // at most half the loop frequency
                filter_alpha = 1.0f - expf(-2.0f*3.14159265f*fc*LOOP_DT);
                say("fc = %.0f Hz", fc);
                break;
            case CMD_STREAM: stream_every = x < 0 ? 0 : (uint32_t)x; say("stream = %u", (unsigned)stream_every); break;
            case CMD_STATUS:
                say("mode=%u (0=idle 1=fixed voltage 2=calibrating 3=PD) hv=%.1f V  height=%.1f %%", mode, hv, z*100);
                say("sp=%.1f %%  kp=%.3f  kd=%.5f  v0=%.1f V  v1=%.1f V  vmax=%.1f V  fc=%.0f Hz  stream=%u",
                    setpoint*100, kp, kd, v0, v1, vmax, fc, (unsigned)stream_every);
                if(cal_valid){
                    say("cal: bottom = %.1f %.1f %.1f %.1f counts", cal_bottom[0], cal_bottom[1], cal_bottom[2], cal_bottom[3]);
                    say("cal: travel = %.1f %.1f %.1f %.1f counts", cal_span[0], cal_span[1], cal_span[2], cal_span[3]);
                }else{
                    say("cal: not calibrated");
                }
                say("loop=%u us (work per loop since last status: average %u us, longest %u us)", (unsigned)LOOP_TIME_US,
                    (unsigned)(work_loops ? work_us_sum/work_loops : 0), (unsigned)work_us_max);
                say("late loops=%u  ignored sensor readings=%u", (unsigned)late_loops, (unsigned)spike_count);
                work_us_max = 0; work_us_sum = 0; work_loops = 0;
                break;
            }
        }

        // ===== safety checks =====
        if(mode != MODE_IDLE && !stdio_usb_connected()){
            // nobody is listening on USB any more: don't leave high voltage on
            if(mode == MODE_CAL){ cal_valid = false; }
            mode = MODE_IDLE;
        }
        if(mode != MODE_IDLE && sensor_dead){
            if(mode == MODE_CAL){ cal_valid = false; }
            mode = MODE_IDLE;
            say("ERROR: a sensor stopped giving readings (last: 0x%04x 0x%04x 0x%04x 0x%04x). High voltage off.",
                result0[0], result0[1], result0[2], result0[3]);
        }

        // ===== decide the high voltage =====
        if(mode == MODE_IDLE){
            hv = 0;

        }else if(mode == MODE_MANUAL){
            hv = manual_v;

        }else if(mode == MODE_CAL){
            // Calibration: find what each sensor reads with the mass on the
            // bottom stop (0V) and on the top stop (full voltage), and at
            // what voltage it leaves each stop.
            cal_loops += 1;
            // how far the corner of the mass that has moved the least has
            // come up from the bottom stop (in sensor counts)
            float smallest_drop = cal_bottom[0] - filtered[0];
            for(int i=1; i<4; i++){
                if(cal_bottom[i] - filtered[i] < smallest_drop){ smallest_drop = cal_bottom[i] - filtered[i]; }
            }
            switch(cal_step){
            case CAL_REST_SETTLE: // 0V, let the mass settle on the bottom stop
                hv = 0;
                if(cal_loops >= CAL_SETTLE_LOOPS){
                    for(int i=0; i<4; i++){ cal_sum[i] = 0; }
                    cal_step = CAL_REST_AVERAGE; cal_loops = 0;
                }
                break;
            case CAL_REST_AVERAGE: // average the readings at the bottom stop
                hv = 0;
                for(int i=0; i<4; i++){ cal_sum[i] += good[i]; }
                if(cal_loops >= CAL_AVERAGE_LOOPS){
                    for(int i=0; i<4; i++){ cal_bottom[i] = cal_sum[i]/cal_loops; }
                    cal_lift_seen = false;
                    cal_step = CAL_RAMP_UP; cal_loops = 0;
                }
                break;
            case CAL_RAMP_UP: // raise voltage slowly, note when the mass lifts off
                hv += CAL_RAMP_V_PER_S*LOOP_DT;
                // (All four corners must have left the bottom. The mass
                // usually tips up on one side first and can sit like that,
                // one corner still down, over a range of voltages.)
                if(!cal_lift_seen && smallest_drop > CAL_MOVE_COUNTS){
                    cal_lift_seen = true;
                    cal_lift_v = hv;
                }
                // Stop raising the voltage as soon as the mass has lifted off:
                // it then carries on to the top stop by itself (the pull gets
                // stronger as it rises), and the less voltage it is held there
                // with, the less it tends to stick. Otherwise go up to vmax.
                if(cal_lift_seen || hv >= vmax){
                    if(hv > vmax){ hv = vmax; }
                    cal_hold_v = hv;
                    cal_step = CAL_TOP_SETTLE; cal_loops = 0;
                }
                break;
            case CAL_TOP_SETTLE: // hold that voltage, let the mass settle on the top stop
                hv = cal_hold_v;
                if(cal_loops >= CAL_TOP_SETTLE_LOOPS){
                    for(int i=0; i<4; i++){ cal_sum[i] = 0; }
                    cal_step = CAL_TOP_AVERAGE; cal_loops = 0;
                }
                break;
            case CAL_TOP_AVERAGE: // average the readings at the top stop
                hv = cal_hold_v;
                for(int i=0; i<4; i++){ cal_sum[i] += good[i]; }
                if(cal_loops >= CAL_TOP_AVERAGE_LOOPS){
                    bool moved = true;
                    for(int i=0; i<4; i++){
                        cal_span[i] = cal_bottom[i] - cal_sum[i]/cal_loops;
                        if(cal_span[i] < CAL_MIN_SPAN){ moved = false; }
                    }
                    if(!moved){
                        say("CAL FAILED: the mass did not move at %.0f V (is it stuck?). High voltage off.", cal_hold_v);
                        say("  sensor change bottom->top = %.1f %.1f %.1f %.1f counts (need at least %.0f each)",
                            cal_span[0], cal_span[1], cal_span[2], cal_span[3], CAL_MIN_SPAN);
                        mode = MODE_IDLE;
                        hv = 0;
                    }else{
                        cal_valid = true; // (height is now available for the next steps)
                        z_previous = 1;
                        cal_release_seen = false;
                        cal_step = CAL_RAMP_DOWN; cal_loops = 0;
                    }
                }
                break;
            case CAL_RAMP_DOWN: // lower voltage slowly, note when the mass drops
                hv -= CAL_RAMP_V_PER_S*LOOP_DT;
                if(!cal_release_seen && z < 0.5f){
                    cal_release_seen = true;
                    cal_release_v = hv > 0 ? hv : 0;
                }
                if(hv <= 0){
                    hv = 0;
                    cal_step = CAL_END_SETTLE; cal_loops = 0;
                }
                break;
            case CAL_END_SETTLE: // 0V, wait for the mass to come back down
                hv = 0;
                if((z < 0.2f && cal_loops >= CAL_SETTLE_LOOPS) || cal_loops >= CAL_RELEASE_LOOPS){
                    if(cal_lift_seen){ v0 = cal_lift_v; }
                    v1 = cal_release_seen ? cal_release_v : 0;
                    say("cal done. bottom = %.1f %.1f %.1f %.1f counts", cal_bottom[0], cal_bottom[1], cal_bottom[2], cal_bottom[3]);
                    say("  travel = %.1f %.1f %.1f %.1f counts", cal_span[0], cal_span[1], cal_span[2], cal_span[3]);
                    say("  left bottom stop at v0 = %.1f V, left top stop at v1 = %.1f V", v0, v1);
                    float smallest = cal_span[0], largest = cal_span[0];
                    for(int i=1; i<4; i++){
                        if(cal_span[i] < smallest){ smallest = cal_span[i]; }
                        if(cal_span[i] > largest){ largest = cal_span[i]; }
                    }
                    if(smallest < 0.5f*largest){
                        say("  WARNING: travel is very uneven between sensors; the mass may be tilted or partly stuck");
                    }
                    if(z > 0.2f){
                        say("  WARNING: the mass did not come back down (height %.0f %% at 0 V); it may be stuck at the top", z*100);
                    }
                    mode = MODE_IDLE;
                }
                break;
            }

        }else if(mode == MODE_RUN){
            // PD controller (see explanation above main())
            // move the target height smoothly toward the setpoint
            float step = SETPOINT_SLEW_PER_S*LOOP_DT;
            z_target += clampf(setpoint - z_target, -step, step);
            // upward force wanted, in units of the weight of the mass
            float force = 1.0f + kp*(z_target - z) - kd*z_speed;
            if(force < 0){ force = 0; } // electrodes can only pull up
            // voltage that would just hold the weight at this height...
            float v_weight = v0 + (v1 - v0)*clampf(z, 0, 1);
            // ...scaled for the force wanted (force goes as voltage squared)
            hv = v_weight*sqrtf(force);

            // give up if the mass looks physically stuck
            stuck_top_loops = (z > 0.9f && hv < 1.0f) ? stuck_top_loops + 1 : 0;
            stuck_bottom_loops = (z < 0.1f && hv >= vmax) ? stuck_bottom_loops + 1 : 0;
            if(stuck_top_loops > STUCK_TOP_LOOPS){
                say("STOPPED: the mass stays at the top stop (height %.0f %%) with 0 V. It is probably stuck.", z*100);
                mode = MODE_IDLE;
                hv = 0;
            }else if(stuck_bottom_loops > STUCK_BOTTOM_LOOPS){
                say("STOPPED: the mass stays at the bottom stop (height %.0f %%) with %.0f V. It is probably stuck.", z*100, vmax);
                mode = MODE_IDLE;
                hv = 0;
            }
        }

        // ===== output the high voltage =====
        hv = clampf(hv, 0, vmax);
        HVAWG_write_all(hv);
        gpio_put(PICO_DEFAULT_LED_PIN, hv > 0);

        // ===== send data to the other core for printing =====
        loop_count += 1;
        if(stream_every > 0 && loop_count % stream_every == 0){
            telemetry_t entry = {t_us, {result0[0], result0[1], result0[2], result0[3]}, hv, z, z_target, mode};
            if(mode != MODE_RUN){ entry.z_ref = setpoint; }
            // note: this causes significant (maybe ~20us) timing jitter
            queue_try_add(&data_queue, &entry); // nonblocking
        }

        // (keep statistics on how long the work in each loop takes)
        uint32_t work_us = (uint32_t)(to_us_since_boot(get_absolute_time()) - t_loop_begin_us);
        if(work_us > work_us_max){ work_us_max = work_us; }
        work_us_sum += work_us;
        work_loops += 1;

        // ===== TIMING: DO NOT EDIT THIS SECTION =====
        // Loop length will vary slightly due to multicore communication,
        // so keep a timer to try to control average loop time, making each
        // loop slightly longer or shorter as necessary.
        loop_start_time = delayed_by_us(loop_start_time, LOOP_TIME_US);
        absolute_time_t now_time = get_absolute_time();
        if(loop_start_time < now_time){
            // If a loop took far too long (1.5-2x longer than expected or
            // more, we will have desynchronized from our loop time count
            // and should restart as quickly as possible.
            loop_start_time = now_time;
            late_loops += 1;
        }
        // (busy_wait_until instead of sleep_until: sleep_until relies on a
        // timer interrupt on the other core, which USB traffic can delay)
        busy_wait_until(loop_start_time); // takes absolute_time_t
        // ===== END TIMING SECTION =====
    }
}

// This is the starting point of the entire program (on core 0).
// - sets up USB
// - starts the control loop on the second microcontroller core
// - then prints data and reads commands forever
int main() {

    // Enable printing to whichever outputs are enabled in CMakeLists.txt
    // (currently only USB over serial)
    // (print by calling printf function)
    // USB serial port defaults to 115200 baud
    stdio_init_all();

    // We'll use queue datastructures to transfer data between the two
    // microcontroller cores (see above). The data queue doesn't need to be
    // big (to avoid unexpected delays if something weird happens); it's
    // only necessary as a thread-safe way to share data.
    queue_init(&data_queue, sizeof(telemetry_t), 2);
    queue_init(&message_queue, sizeof(message_t), 24);
    queue_init(&command_queue, sizeof(command_t), 4);

    // Launch the control loop on the second microcontroller core...
    multicore_launch_core1(control_main);
    // ...and do the USB printing and command reading on this one.
    usb_main();

    return 0;
}
