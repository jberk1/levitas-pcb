/**
 * Part of a library to run I2C master from a PIO state machine.
 * (use with pio_i2c.c and pio_i2c.pio)
 *
 * Originally from (then slightly modified)
 * https://github.com/raspberrypi/pico-examples/tree/master/pio/i2c
 */

/**
 * Copyright (c) 2021 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef _PIO_I2C_H
#define _PIO_I2C_H

#include "pio_i2c.pio.h"

// ----------------------------------------------------------------------------
// Low-level functions

void pio_i2c_start(PIO pio, uint sm);
void pio_i2c_stop(PIO pio, uint sm);
void pio_i2c_restart_stop(PIO pio, uint sm);

bool pio_i2c_check_error(PIO pio, uint sm);
void pio_i2c_resume_after_error(PIO pio, uint sm);

static void pio_i2c_put16_nocheck(PIO pio, uint sm, uint16_t data);
static void pio_i2c_put16_or_err(PIO pio, uint sm, uint16_t data);
uint8_t pio_i2c_get(PIO pio, uint sm);

// ----------------------------------------------------------------------------
// Transaction-level functions

uint pio_i2c_write_blocking(PIO pio, uint sm, uint8_t addr, uint8_t *txbuf, uint len, bool nostop);
uint pio_i2c_write_blocking_4(PIO pio, uint sm0, uint sm1, uint sm2, uint sm3, uint8_t addr, uint8_t *txbuf, uint len, bool nostop);
uint pio_i2c_read_blocking(PIO pio, uint sm, uint8_t addr, uint8_t *rxbuf, uint len);
uint pio_i2c_read_blocking_4(PIO pio, uint sm0, uint sm1, uint sm2, uint sm3, uint8_t addr, uint8_t *rxbuf0,  uint8_t *rxbuf1, uint8_t *rxbuf2, uint8_t *rxbuf3, uint len);

// ----------------------------------------------------------------------------
// Multi-bus transaction-level functions
// (same idea as the _4 functions above, but for any number of I2C buses,
// which may be spread over several PIOs since each PIO only has four
// state machines. Used by main_v2.c to talk to eight sensors at once.)

// One I2C bus = one state machine on one PIO.
typedef struct {
    PIO pio;
    uint sm;
} pio_i2c_bus_t;

// Most buses the _n functions accept (RP2350: 3 PIOs x 4 state machines).
#define PIO_I2C_MAX_BUSES 12

// Give up on a bus if a transaction has not finished after this long.
// (A healthy 400kHz transaction of a few bytes takes around 0.1ms. This only
// triggers if a bus is physically stuck, e.g., SCL shorted to ground.)
#define PIO_I2C_TIMEOUT_US 5000

// Both return a bitmask of the buses that failed (bit i = bus[i]), 0 if all ok.
// A failure on one bus does not disturb the transactions on the other buses.
uint pio_i2c_write_blocking_n(const pio_i2c_bus_t *bus, uint n, uint8_t addr, const uint8_t *txbuf, uint len, bool nostop);
// rxbuf must hold n*len bytes: bus[i] puts its bytes at rxbuf[i*len].
uint pio_i2c_read_blocking_n(const pio_i2c_bus_t *bus, uint n, uint8_t addr, uint8_t *rxbuf, uint len);

#endif
