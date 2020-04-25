#pragma once

/*
 * Application considerations:
 * The ATtiny48/88 has three 8-bit ports available.
 * Two have special functionality of interest to the application:
 *     PORTB: used by ISP
 *     PORTC: used by TWI/I²C
 *
 */


#define PRODUCT_ID keyboardio-model-101

// Define the LED driver class we're using
//#define LED_DRIVER_ISSI31FL3743B

#define LED_DRIVER_WS2812B

// KEY_REPORT_SIZE_BYTES is the number of bytes of data in a key report we send to the host
#define KEY_REPORT_SIZE_BYTES 4


// Debouncer config

//#define DEBOUNCER "debounce-integrator.h"
//#define DEBOUNCER "debounce-counter.h"
//#define DEBOUNCER "debounce-none.h"
//#define DEBOUNCER "debounce-split-counters-and-lockouts.h"
#define DEBOUNCER "debounce-split-counters.h"
//#define DEBOUNCER "debounce-state-machine.h"
#define DEBOUNCE_STATE_MACHINE "config/debounce-state-machines/chatter-defense.h"
//#define DEBOUNCE_STATE_MACHINE "config/debounce-state-machines/simple.h"


// Actual hardware configuration

// ROWS
#define PORT_ROWS PORTC
#define DDR_ROWS DDRC
#define PIN_ROWS PINC
#define MASK_ROWS  (_BV(0)|_BV(1)|_BV(2)|_BV(3))
#define COUNT_ROWS 4
#define PIN_ORDER_ROWS 3,2,1,0



// COLS
#define PORT_COLS PORTD
#define DDR_COLS DDRD
#define PIN_COLS PIND
#define MASK_COLS  (_BV(0)|_BV(1)|_BV(2)|_BV(3)|_BV(4)|_BV(5)|_BV(6)|_BV(7))
#define COUNT_COLS 8


// AD01: lower two bits of device address
#define AD01() ((PINB & _BV(0)) |( PINB & _BV(1)))
