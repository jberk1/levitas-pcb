// This file is the entry point (and main source code file) for the
// second version of the LeviTAS PCB (KiCad project "levitas_v2").
// It is based on main.c, which is for the first version of the PCB.
// ("main.c" here and below means src/main.c in the separate "firmware"
// folder. This folder only has the firmware for the second version.)
//
// It contains code to read from eight FDC2112 capacitance sensors
// and write the data out via a serial port (over USB).
//
// Differences from main.c:
// - eight sensors instead of four (so two PIOs are needed; each PIO only
//   has four state machines)
// - sensors are on different pins (see table below)
// - different sensor offset and drive current (see "Sensor settings" below)
// - no high voltage output. On this PCB the MAX5715 DAC is no longer on
//   GPIO 13/14/15 (those are now sensor I2C pins) so the DAC code in main.c
//   must not be copied over as-is. See "High voltage" section below.

// include necessary standard libraries
#include "pico/stdlib.h"        // standard functions
#include <stdio.h>              // printf
#include "pico/multicore.h"     // send USB printf on second core
#include "pico/util/queue.h"    // queue for multicore communication
#include "pico/time.h"          // assorted time functions
#include "pico/stdio_usb.h"     // USB connection functions
#include "pico/bootrom.h"       // restart into BOOTSEL mode from software
// also include our custom I2C PIO library
#include "pio_i2c.h"

// ===========================================================================
// ===========================================================================
// ========== Board ==========================================================
// ===========================================================================
// ===========================================================================
// This runs on either a Raspberry Pi Pico 2 or a Pico 2 W (same
// microcontroller, same pins on the headers). The one difference that
// matters here is the LED: on the Pico 2 it is on GPIO 25, but on the
// Pico 2 W it is wired to the wireless chip instead, and GPIO 25 is one of
// the pins the Pico uses to talk to that chip.
//
// Set this to 1 only if using a plain Pico 2 (not W), and then also change
// PICO_BOARD from pico2_w to pico2 in CMakeLists.txt. Then the LED shows
// whether the sensors were found (on = all found, blinking = some missing).
// (The same information is printed over USB either way.)
#define USE_LED 0

// ===========================================================================
// ===========================================================================
// ========== Wiring =========================================================
// ===========================================================================
// ===========================================================================
// (From the KiCad netlist levitas_v2.net. The Pico plugs into sockets J1
// (RPI_L = Pico pins 1-20) and J2 (RPI_R = Pico pins 40-21).)
//
// Each FDC2112 is on its own I2C bus (they all have the same I2C address),
// with 4.7k pullups to 3V3_A. Only channel 0 of each chip is used; it
// measures the capacitance between a pair of sense electrodes on the
// Levitas3 chip (U1).
//
//  sensor  chip  SDA     SCL     electrodes (IN0A - IN0B)   PIO  state machine
//  ======  ====  ======  ======  =========================  ===  =============
//  1       U2    GPIO12  GPIO13  XS_Q4    - YS_Q1           0    0
//  2       U3    GPIO0   GPIO1   YS_Q4    - XS_Q1           0    1
//  3       U4    GPIO8   GPIO9   YS_Q2    - XS_Q3           0    2
//  4       U5    GPIO4   GPIO5   XS_Q2    - YS_Q3           0    3
//  5       U6    GPIO14  GPIO15  ZS_XP_Q4 - ZS_XP_Q1        1    0
//  6       U7    GPIO6   GPIO7   ZS_XM_Q2 - ZS_XM_Q3        1    1
//  7       U8    GPIO2   GPIO3   ZS_YP_Q1 - ZS_YP_Q2        1    2
//  8       U9    GPIO10  GPIO11  ZS_YM_Q3 - ZS_YM_Q4        1    3
//
// (Sensor numbers match the SDA1..SDA8/SCL1..SCL8 net names in the schematic.
// Data is printed in this order.)

#define NUM_SENSORS 8

// SDA pin of each sensor, in order sensor 1..8. SCL is always the next pin
// (required by the PIO program).
static const uint SENSOR_SDA_PIN[NUM_SENSORS] = {12, 0, 8, 4, 14, 6, 2, 10};

// ===========================================================================
// ===========================================================================
// ========== FDC2112 capacitance sensors ====================================
// ===========================================================================
// ===========================================================================
// (This section contains data and functions to use the capacitance sensors.)
// (adapted from main.c)

// I2C address of all FDC2112 chips (each on its own I2C bus)
// (ADDR pin is tied to 3V3_A on every chip)
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

// value of the DEVICE_ID register on an FDC2112 (or FDC2114)
#define FDC2112_DEVICE_ID 0x3054

// ===== Sensor settings =====
// (These are the same values as main.c, except for the offset and the drive
// current, which had to be tuned for this PCB. See the comments in
// fdc2112_startup() in main.c for the full reasoning behind the others.)
//
// Measurement time is (RCOUNT*16+4)/f_{REF0}. RCOUNT=33=0x0021 gives 9.04
// bits of precision in about 13us.
#define FDC2112_RCOUNT 0x0021
// sensor activation time (must have settlecount > 3)
#define FDC2112_SETTLECOUNT 0x000a
// f_{REF0}=f_{CLK}/CH0_FREQ_DIVIDER, f_{IN0}=f_{SENSOR0}/CH0_FIN_SEL
#define FDC2112_CLOCK_DIVIDERS 0x2002
// Only 12 of the sensor's 16 data bits can be read over I2C. To see the
// least significant bits, the data is bit-shifted (gain) and given an
// offset before reading.
// 0x0600 = 4-bit shift (2^4=16x multiplier) (see manual page 35)
#define FDC2112_OUTPUT_GAIN 0x0600
// offset subtracted from data (see manual page 27)
//
// The printed value is (16-bit sensor data) - (this offset), and can only
// be 0x0000 to 0x0fff. If the sensor data is below the offset the printed
// value is stuck at 0x0000; if it is more than 0x0fff above the offset the
// printed value is stuck at 0x0fff.
//
// main.c uses 0x1800 for the first PCB. The sensors on this PCB oscillate
// at a lower frequency (about 3.5-4.6MHz), so most of them were stuck at
// 0x0000 with that offset. Measured 16-bit sensor data on this PCB
// (2026-11-17, with the drive currents below), sensor 1..8:
//
//   0x1468 0x152f 0x1469 0x1563 0x1a5d 0x1af2 0x1534 0x1473
//
// 0x1000 puts all of these inside the 12-bit window (they print as
// 0x0468 ... 0x0af2), with at least 0x0460 of room on either side.
//
// To retune: if a sensor is stuck at 0x0000 lower the offset, if it is
// stuck at 0x0fff raise it (e.g., in steps of 0x0400). The offset is the
// same for all sensors here, but it is a separate register on each chip, so
// it could be made different for each sensor the same way as the drive
// current below if the sensors ever end up too far apart to share a window.
#define FDC2112_OFFSET 0x1000
// sensor drive current (see chart on manual page 36)
// This is a code from 0 (lowest current) to 31 (highest), one per sensor,
// in order sensor 1..8.
//
// main.c uses 0 (the default) for the first PCB. THAT DOES NOT WORK ON THIS
// PCB: with code 0, sensors 4, 5 and 8 do not oscillate properly (they read
// full scale, 0x0fff, no matter what the offset is) and sensors 3 and 6
// glitch to 0x0fff from time to time. From code 6 upward all eight sensors
// give steady readings.
//
// The values here were chosen (2026-11-17) so the oscillation amplitude of
// each sensor is inside the range the chip wants (it has a warning flag for
// amplitude too high / too low, which can be put into the data register by
// setting the ERROR_CONFIG register to 0x1000 / 0x0800). The range is only
// one or two codes wide: one code lower than these values and several
// sensors report "too low", one code higher and several report "too high".
// Sensor 4 is borderline (sometimes "too low" at 28, "too high" at 29).
//
// Note the reading depends on the drive current (it changes by roughly
// 0x0040 to 0x0090 per code around these values), so check the offset
// again after changing these.
static const uint8_t FDC2112_IDRIVE[NUM_SENSORS] = {28, 28, 28, 28, 26, 26, 28, 29};

// Printed in place of a capacitance value if the I2C read of that sensor
// failed (real values are 12 bits, so they are never above 0x0fff).
#define FDC2112_READ_FAILED 0xffff

typedef struct {
    // store eight capacitance values
    uint16_t data[NUM_SENSORS];
} levitation_state_t;
queue_t data_queue;

// Filled in during startup, printed by second core when USB is connected:
// value of DEVICE_ID register of each sensor (0 if sensor did not respond)
static uint16_t sensor_device_id[NUM_SENSORS];
// bit i set = sensor i+1 responded during startup
static uint sensor_ok_mask = 0;

// write a 16-bit value to the given register of every sensor in `bus`
// (returns bitmask of buses that failed)
static uint fdc2112_write_register_n(const pio_i2c_bus_t *bus, uint n, uint8_t reg, uint16_t data){
    uint8_t buf[3];
    buf[0] = reg;
    buf[1] = data >> 8; // send MSB byte first
    buf[2] = data & 0xff; // send LSB byte last
    return pio_i2c_write_blocking_n(bus, n, FDC2112_I2C_ADDR, buf, 3, false);
}

// read a 16-bit value from given register of every sensor in `bus`
// (result must hold n values) (returns bitmask of buses that failed)
static uint fdc2112_read_register_n(const pio_i2c_bus_t *bus, uint n, uint8_t reg, uint16_t* result){
    uint8_t buf_reg[1];
    buf_reg[0] = reg;
    uint8_t buf_data[2*PIO_I2C_MAX_BUSES] = {0};
    uint err = pio_i2c_write_blocking_n(bus, n, FDC2112_I2C_ADDR, buf_reg, 1, true); // shouldn't stop
    err |= pio_i2c_read_blocking_n(bus, n, FDC2112_I2C_ADDR, buf_data, 2);
    // result is MSB then LSB byte
    for(uint i=0; i<n; i++){
        result[i] = (((uint16_t)buf_data[2*i]) << 8) | ((uint16_t)buf_data[2*i+1]);
    }
    return err;
}

// read a 16-bit value from every sensor in `bus`
// (assuming register was previously indicated, e.g., by a full read)
static uint fdc2112_read_register_nowrite_n(const pio_i2c_bus_t *bus, uint n, uint16_t* result){
    uint8_t buf_data[2*PIO_I2C_MAX_BUSES] = {0};
    uint err = pio_i2c_read_blocking_n(bus, n, FDC2112_I2C_ADDR, buf_data, 2);
    // result is MSB then LSB byte
    for(uint i=0; i<n; i++){
        result[i] = (((uint16_t)buf_data[2*i]) << 8) | ((uint16_t)buf_data[2*i+1]);
    }
    return err;
}

// Configure every sensor in `bus` and start measuring.
// sensor[k] is which sensor (0..7) is on bus[k]; it is needed for the
// settings that are different for each sensor.
// (returns bitmask of buses where any of the writes failed)
static uint fdc2112_startup_n(const pio_i2c_bus_t *bus, const uint *sensor, uint n){
    uint err = 0;

    // The sensors are powered from the USB 5V supply (via the 3V3_A
    // regulator), not from the Pico, so they stay powered and keep running
    // when the Pico is reset or reprogrammed. Reset them so they are always
    // configured from the same (sleeping) state as after power on.
    // (bit 15 of RESET_DEV register)
    err |= fdc2112_write_register_n(bus, n, FDC2112_REG_RESET_DEV, 0x8000);
    sleep_ms(10);

    // ===== Measurement time behavior =====
    err |= fdc2112_write_register_n(bus, n, FDC2112_REG_RCOUNT_CH0, FDC2112_RCOUNT);
    err |= fdc2112_write_register_n(bus, n, FDC2112_REG_SETTLECOUNT_CH0, FDC2112_SETTLECOUNT);

    // ===== Small things =====
    //
    // The chip has some error reporting functions; leave them off
    err |= fdc2112_write_register_n(bus, n, FDC2112_REG_ERROR_CONFIG, 0x0000);
    //
    // Prepare to only read capacitance from a single channel on the chip
    // (this setting can choose to alternate readings between channels)
    // 0000 0010 0000 1101 = 0x020d
    err |= fdc2112_write_register_n(bus, n, FDC2112_REG_MUX_CONFIG, 0x020d);

    // ===== Gain and offset settings =====
    err |= fdc2112_write_register_n(bus, n, FDC2112_REG_CLOCK_DIVIDERS_CH0, FDC2112_CLOCK_DIVIDERS);
    err |= fdc2112_write_register_n(bus, n, FDC2112_REG_RESET_DEV, FDC2112_OUTPUT_GAIN);
    err |= fdc2112_write_register_n(bus, n, FDC2112_REG_OFFSET_CH0, FDC2112_OFFSET);

    // ===== Drive current =====
    // (different for each sensor, so write to one sensor at a time)
    for(uint k=0; k<n; k++){
        // the code goes in bits 15:11 of the register
        uint16_t drive_current = (uint16_t)FDC2112_IDRIVE[sensor[k]] << 11;
        err |= fdc2112_write_register_n(&bus[k], 1, FDC2112_REG_DRIVE_CURRENT_CH0, drive_current) << k;
    }

    // ===== Start measurement =====
    //
    // The chip is in a sleep state. Use the following command to start the
    // chip so we can read capacitance. Note that afterward, additional
    // configuration changes might not work (which is why we do this last).
    //
    // At the same time, this register also choose which oscillator to use.
    // We will use the chip's internal oscillator (CLKIN is grounded).
    //
    // Set register 0x1a to 00010100 00000001 = 0x1401
    // (sense on channel 0, no sleep mode, full current, internal oscillator)
    // - f_{CLK} = f_{INTCLK} = internal clock = 43MHz
    //   (approximately; ranges 35-55MHz with manufacturing)
    err |= fdc2112_write_register_n(bus, n, FDC2112_REG_CONFIG, 0x1401);

    return err;
}

// ===========================================================================
// ===========================================================================
// ========== High voltage ===================================================
// ===========================================================================
// ===========================================================================
// This firmware does not output high voltage.
//
// The MAX5715 DAC (U12) is still on this PCB, but its SPI pins have moved.
// If jumpers are fitted on J19, J20, J21, they are connected as follows:
//
//   DAC CS   = DATA_A = GPIO 20 = Pico pin 26
//   DAC SCLK = DATA_B = GPIO 19 = Pico pin 25
//   DAC MOSI = DATA_C = GPIO 18 = Pico pin 24
//
// WARNING: THIS CANNOT USE THE PICO'S SPI HARDWARE THE WAY main.c DOES.
// Hardware SPI0 has its clock on GPIO 18 and its data output on GPIO 19,
// which is backward compared to the wiring above. Either send the SPI data
// "by hand" (toggling GPIOs, or a PIO program on pio2), or leave the SCLK
// and MOSI jumpers off and connect them crosswise with wires.
//
// (The same three pins also go to the external high voltage board
// connectors J3, J4, J5, along with SHIFT_CLK = GPIO 17 and
// STORE_CLK = GPIO 16.)
#define HVAWG_CS_PIN 20
#define HVAWG_SCLK_PIN 19
#define HVAWG_MOSI_PIN 18

// Hold the DAC's SPI pins still, with the DAC not selected (CS is active
// low). Otherwise these pins are only weakly pulled low, which selects
// the DAC, and it could mistake electrical noise on its clock pin for a
// command to output a voltage. The DAC outputs 0V after power on and stays
// there until told otherwise.
static void HVAWG_disable(){
    gpio_init(HVAWG_CS_PIN);
    gpio_put(HVAWG_CS_PIN, 1);
    gpio_set_dir(HVAWG_CS_PIN, GPIO_OUT);
    gpio_init(HVAWG_SCLK_PIN);
    gpio_put(HVAWG_SCLK_PIN, 0);
    gpio_set_dir(HVAWG_SCLK_PIN, GPIO_OUT);
    gpio_init(HVAWG_MOSI_PIN);
    gpio_put(HVAWG_MOSI_PIN, 0);
    gpio_set_dir(HVAWG_MOSI_PIN, GPIO_OUT);
}

// ===========================================================================
// ===========================================================================
// ========== Loading new firmware without the BOOTSEL button ================
// ===========================================================================
// ===========================================================================
// To load new firmware, the Pico has to be in BOOTSEL mode. Instead of
// unplugging USB and holding the BOOTSEL button, a running program can be
// told to restart into BOOTSEL mode over USB. There are three ways:
//
// 1. picotool with the -f option ("picotool load -f -x levitas_v2.elf").
//    This is what "Run Project (USB)" in VS Code does. It works because
//    pico_enable_stdio_usb() in CMakeLists.txt adds a hidden USB "reset"
//    interface to this program (nothing in this file is needed for it).
//
// 2. Opening the serial port at 1200 baud (and closing it again).
//    Also provided by pico_enable_stdio_usb().
//
// 3. Sending the word "bootsel" over the serial port, e.g., by typing it
//    into the VS Code serial monitor. This is done by the function below,
//    as a backup in case (1) does not work on your computer.
//
// Note all three need this program (or another one with USB enabled) to
// already be running on the Pico. A brand new Pico, or one running a program
// without USB, still needs the BOOTSEL button once.

// Check whether the word "bootsel" (upper or lower case) has been received
// over USB serial, and if so restart the Pico into BOOTSEL mode.
// Does not wait: only looks at characters that have already arrived.
static void check_for_bootsel_command(){
    static const char command[] = "bootsel";
    static uint num_matched = 0; // how many letters of the command we've seen so far
    int c;
    while((c = getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT){
        if(c >= 'A' && c <= 'Z'){ c += 'a' - 'A'; } // make lower case
        if(c == command[num_matched]){
            num_matched += 1;
            if(num_matched == sizeof(command)-1){
                printf("# restarting into BOOTSEL mode\n");
                sleep_ms(100); // give the message time to be sent
                // (0, 0) = no activity LED, keep both USB drive and picotool interfaces
                reset_usb_boot(0, 0); // does not return
            }
        }else{
            // not the letter we wanted; but it could be the start of the command
            num_matched = (c == command[0]) ? 1 : 0;
        }
    }
}

// ===========================================================================
// ===========================================================================
// ========== Main code ======================================================
// ===========================================================================
// ===========================================================================
// (This section contains the main function.)

// We'll run this function on the second microcontroller core.
// It will simply, in an infinite loop, print data values out over USB serial.
// (see main.c for why this is on the second core)
void core2_main(){
    bool was_connected = false;
    while(true){
        // get data from main core
        levitation_state_t datapoint;
        queue_remove_blocking(&data_queue, &datapoint);

        // Each time a program on the computer opens the serial port, first
        // print which sensors were found during startup. These lines start
        // with '#' so programs reading the data can skip them.
        bool connected = stdio_usb_connected();
        if(connected && !was_connected){
            printf("# levitas v2: sensors found = 0x%02x (bit 0 = sensor 1; 0xff = all 8)\n", sensor_ok_mask);
            for(int i=0; i<NUM_SENSORS; i++){
                printf("# sensor %d (SDA=GPIO%u SCL=GPIO%u): ", i+1, SENSOR_SDA_PIN[i], SENSOR_SDA_PIN[i]+1);
                if(!(sensor_ok_mask & (1u << i))){
                    printf("NO RESPONSE\n");
                }else if(sensor_device_id[i] != FDC2112_DEVICE_ID){
                    printf("unexpected device id 0x%04x (expected 0x%04x)\n", sensor_device_id[i], FDC2112_DEVICE_ID);
                }else{
                    printf("ok\n");
                }
            }
        }
        was_connected = connected;

        // and print to USB:
        // - microseconds of time since program started,
        // - (8x) capacitance sensor data values (as a hexadecimal integer),
        //       in order sensor 1..8 (see wiring table at top of file)
        //       (roughly speaking, higher values = lower capacitance,
        //        but the exact correlation between data value and capacitance
        //        is inexact and somewhat unknown due to large variances in
        //        part tolerances (FDC2112 oscillator frequency, and the
        //        tolerance of the capacitor and inductor in the circuit) as well
        //        as FDC2112 sensor drift (the values sometimes change slightly
        //        over time for an unknown reason)
        //       (0xffff = could not read from that sensor)
        printf("%llu 0x%04x 0x%04x 0x%04x 0x%04x 0x%04x 0x%04x 0x%04x 0x%04x\n", to_us_since_boot(get_absolute_time()),
            datapoint.data[0],
            datapoint.data[1],
            datapoint.data[2],
            datapoint.data[3],
            datapoint.data[4],
            datapoint.data[5],
            datapoint.data[6],
            datapoint.data[7]
        );

        // restart into BOOTSEL mode if asked to over USB (see above)
        check_for_bootsel_command();
    }
}

// This is the starting point of the entire program,
// and runs on the main microcontroller core.
// - starts second core to print data
// - initializes FDC2112 capacitance sensors
// - then runs main loop
//   - read from capacitance sensors
//   - send data to second core to be printed with USB
//   - try to run at a fixed frequency
int main() {

    // Enable printing to whichever outputs are enabled in CMakeLists.txt
    // (currently only USB over serial)
    // (print by calling printf function)
    // USB serial port defaults to 115200 baud
    stdio_init_all();

    // make sure the high voltage DAC ignores its inputs (see above)
    HVAWG_disable();

    // Launch second microcontroller core, which will print data over USB.
    // We'll use a queue datastructure to transfer data between the main
    // and second microcontroller cores: the main core puts data into the
    // queue, and the second core takes it out.
    // The queue doesn't need to be big (here we have it only hold 2 things
    // at once to avoid unexpected delays if something weird happens); it's
    // only necessary as a thread-safe way to share data.
    queue_init(&data_queue, sizeof(levitation_state_t), 2);
    multicore_launch_core1(core2_main);

    // set up LED
    // (on = all sensors found, blinking = at least one sensor missing)
#if USE_LED
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, 1);
#endif

    // set up FDC2112 sensors
    // Each sensor needs its own PIO state machine. There are only four state
    // machines per PIO, so use two PIOs: sensors 1-4 on pio0, 5-8 on pio1.
    // The PIO program has to be loaded into each PIO separately.
    uint offset0 = pio_add_program(pio0, &pio_i2c_program); // load program into PIO memory, return instruction memory offset it is loaded at
    uint offset1 = pio_add_program(pio1, &pio_i2c_program);
    pio_i2c_bus_t sensor_bus[NUM_SENSORS];
    for(int i=0; i<NUM_SENSORS; i++){
        sensor_bus[i].pio = (i < 4) ? pio0 : pio1;
        sensor_bus[i].sm = i % 4;
        // start PIO state machine at given instruction offset with given SDA and SCL pin
        pio_i2c_program_init(sensor_bus[i].pio, sensor_bus[i].sm, (i < 4) ? offset0 : offset1, SENSOR_SDA_PIN[i], SENSOR_SDA_PIN[i]+1);
    }

    // wait a bit just in case fdc2112 sensors need time to initialize
    sleep_ms(500);

    // Find out which sensors are there by reading their DEVICE_ID register.
    uint id_err = fdc2112_read_register_n(sensor_bus, NUM_SENSORS, FDC2112_REG_DEVICE_ID, sensor_device_id);

    // From here on, only talk to the sensors that responded. A sensor that
    // does not respond at all (not soldered properly, I2C lines shorted, ...)
    // would otherwise slow down every read in the main loop.
    // active_bus[k] is the I2C bus of sensor number active_sensor[k] (0..7)
    pio_i2c_bus_t active_bus[NUM_SENSORS];
    uint active_sensor[NUM_SENSORS];
    uint num_active = 0;
    for(int i=0; i<NUM_SENSORS; i++){
        if(id_err & (1u << i)){
            sensor_device_id[i] = 0;
        }else{
            active_bus[num_active] = sensor_bus[i];
            active_sensor[num_active] = i;
            num_active += 1;
            sensor_ok_mask |= 1u << i;
        }
    }

    // send configuration information to sensors
    fdc2112_startup_n(active_bus, active_sensor, num_active);

    // we'll use these arrays to store data we read from the sensors
    uint16_t raw[NUM_SENSORS];      // in order of active_bus
    uint16_t result[NUM_SENSORS];   // in order sensor 1..8
    for(int i=0; i<NUM_SENSORS; i++){
        result[i] = FDC2112_READ_FAILED; // (stays this way for missing sensors)
    }

    // Make a first reading from the sensors using fdc2112_read_register_n().
    // This is necessary because reading data from the sensors requires:
    //   1. Telling sensor which of its data we want to read (specify register)
    //   2. Actually reading data
    // Later, in the main loop, we'll only do (2) and ignore (1) in order to
    // read data more quickly (via the read_register_nowrite_n() function).
    // However, we still need to do (1) at some point before that. This
    // function call does both (1) and (2).
    fdc2112_read_register_n(active_bus, num_active, FDC2112_REG_DATA_CH0, raw);

    // We'll use these variables to keep main loop running
    // at a fixed frequency (determined by LOOP_TIME_US).
    // WARNING: THE MINIMUM LOOP TIME IS FIXED BY CAPACITIVE SENSOR
    // READS. SETTING A LOWER LOOP TIME HERE WON'T DO ANYTHING. CONFIRM
    // IT'S ACTUALLY WORKING VIA OSCILLOSCOPE (E.G., ON I2C SDA PINS).
    const uint64_t LOOP_TIME_US = 150; // try to make loop this long on average
    absolute_time_t loop_start_time = get_absolute_time();

    while(true){

        // ===== LED =====
        // blink (about 2 times per second) if any sensor is missing
#if USE_LED
        if(num_active < NUM_SENSORS){
            gpio_put(PICO_DEFAULT_LED_PIN, (to_ms_since_boot(get_absolute_time()) / 250) % 2);
        }
#endif

        // ===== read capacitance sensors =====
        // read quickly from the chip registers (all sensors at the same time)
        uint err = fdc2112_read_register_nowrite_n(active_bus, num_active, raw);
        // Put values in order sensor 1..8.
        // Note: main.c throws away values above 0x07d0 here (as a spike
        // filter). That limit was specific to the values seen on the first
        // PCB, so it is not done here; all values are passed on as-is.
        for(uint k=0; k<num_active; k++){
            result[active_sensor[k]] = (err & (1u << k)) ? FDC2112_READ_FAILED : raw[k];
        }

        // send data to second core for printing
        levitation_state_t entry;
        for(int i=0; i<NUM_SENSORS; i++){
            entry.data[i] = result[i];
        }
        // note: this causes significant (maybe ~20us) timing jitter
        queue_try_add(&data_queue, &entry); // nonblocking

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
        }
        sleep_until(loop_start_time); // takes absolute_time_t
        // ===== END TIMING SECTION =====
    }

    return 0;
}
