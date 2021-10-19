#ifdef LED_DRIVER_ISSI31FL3743B

#include <stdint.h>
#include <avr/io.h>

#include <string.h>
#include <util/delay.h>
#include "main.h"
#include "led-api.h"
#include "wire-protocol.h"


/* SPI LED driver to send data to ISSI31FL3743B
 *
 * Preformatted data is sent to the micro and then
 * passed in via led_update_buffer(). The device
 * continuously outputs SPI data, refilling the SPI
 * output buffer from the SPI transfer complete interrupt.
 *
 * Data is double buffered (see notes below), however an update can
 * occur during any byte in the chain (we just guarantee it won't
 * happen mid-LED). The LED refresh rate is high enough that this
 * shouldn't matter.
 */

/* Model 01 left led ordering
3 4 11 12 19 20
2 5 10 13 18 21 26
1 6 9  14 17 22 25
0 7 8  15 16 23 24
  	27 28 29 30
		 31
*/

/* Model 100 left led ordering
1 7  13 19 25 31
2 8  14 20 26 32 35
3 9  15 21 27 33 29
4 10 16 22 28 36 23
	6 12 18 24
		30
*/


/* model 01 right hand
  11 12 19 20 27 28
5 10 13 18 21 26 29
6 9  14 17 22 25 30
7 8  15 16 23 24 31
1 2 3 4
      0
*/
/* model 100 right hand led ordering

   31 25 19 13 7 1
35 32 26 20 14 8 2
29 33 27 21 15 9 3
23 36 28 22 16 10 4
24 18 12 6
	30

*/

// This array maps from the 36 LED indexes of the Model 100
// to the 32 in our internal buffer (which happen to map to the Model 01s)
// The LED driver's registers are one-indexed, so we have a 37 byte array instead of a 36 byte array
// This is a waste of a byte of flash, but I'd rather waste the flash than waste the compute on
// every led transmit
#define NO_LED 0
static const uint8_t led_map_left[37] = {
    0,	3,	2,	1,	27,	NO_LED,	 0,
    4,	5,	6,	28,	NO_LED,	 7,
    11,	10,	9,	29,	NO_LED,	 8,
    12,	13,	14,	30,	24,	 15,
    19,	18,	17,	31,	25,	 16,
    20,	21,	22,	NO_LED,	26,	 23
};

static const uint8_t led_map_right[37] = {
    0,   28,	29,	30,	4,	NO_LED,	31,
    27,	26,	25,	3,	NO_LED,	24,
    20,	21,	22,	2,	NO_LED,	23,
    19,	18,	17,	1,	7,	16,
    12,	13,	14,	0,	6,	15,
    11,	10,	9,	NO_LED,	5,	8
};


uint8_t const *led_map;


#define Addr_Write_Page0 0x50
#define Addr_Write_Page1 0x51
#define Addr_Write_Page2 0x52

#define Addr_Read_Page0 0b110100
#define Addr_Read_Page1 0b110101
#define Addr_Read_Page2 0b110110

#define Pixel_Max 200

// this is the new one
// the ISSI chip's LED buffer has 108 positions we care about.
// We skip the last 3 bytes of SW6

// These are all the pins wired up on the left side.
// The values are really groups of three values representing RGBs (1 =CS1,CS2,CS3, 2=CS4, CS5, CS6, etc)
// SW1: 1,2,3,4,6
// SW2: 1,2,3,4,6
// SW3: 1,2,3,4,6
// SW4: 1,2,3,4,5,6
// SW5: 1,2,3,4,5,6
// SW6: 1,2,3,4,5

#define LED_DRIVER_PIXEL_REGISTER_COUNT 36
// RGB Pixels
#define LED_DATA_BYTES LED_DRIVER_PIXEL_REGISTER_COUNT * 3

#define WAIT_SPI_TRANSMIT()      while (!(SPSR & _BV(SPIF))) ;
#define ENABLE_LED_WRITES SPCR |= _BV(SPIE);
#define DISABLE_LED_WRITES SPCR &= ~_BV(SPIE);


#define END_ISSI_SPI_TXN HIGH(PORTB,2);
#define BEGIN_ISSI_SPI_TXN LOW(PORTB,2);


// Using DISABLE_INTERRUPTS could cause interrupt recursion because led_update
// functions are called from TWI callbacks which run within TWI interrupt. So
// disable only LED SPI transfer interrupt.
//
// And to avoid redundancy, leave re-ENABLE_LED_WRITES to led_data_ready().
#define PROTECT_LED_WRITES(...)  do { DISABLE_LED_WRITES; __VA_ARGS__ /*ENABLE_LED_WRITES;*/ } while (0)

static uint8_t led_spi_frequency = LED_SPI_FREQUENCY_DEFAULT;

typedef union {
    uint8_t each[NUM_LEDS][LED_DATA_SIZE];
    uint8_t whole[LED_BUFSZ];
    uint8_t bank[NUM_LED_BANKS][LED_BANK_SIZE];
} led_buffer_t;

/* (No volatile because all writes are outside the interrupt and in PROTECT_LED_WRITES) */
static led_buffer_t led_buffer = {.whole={0}};


#define LED_BRIGHTNESS_MAX 0xFF
static uint8_t global_brightness = LED_BRIGHTNESS_MAX;

/* (No volatile because no data race and we do only atomic operations (assignment should be atomic)) */
static uint8_t leds_dirty = 1;

/* This function triggers a led update. */
void led_data_ready() {
    leds_dirty = 1;
    ENABLE_LED_WRITES;
}

/* Update the transmit buffer with LED_BUFSZ bytes of new data */
void led_update_bank(uint8_t *buf, const uint8_t bank) {
    /* Double-buffering here is wasteful, but there isn't enough RAM on
       ATTiny48 to single buffer 32 LEDs and have everything else work
       unmodified. However there's enough RAM on ATTiny88 to double
       buffer 32 LEDs! And double buffering is simpler, less likely to
       flicker. */

    PROTECT_LED_WRITES({
        memcpy((uint8_t *)led_buffer.bank[bank], buf, LED_BANK_SIZE);
    // Only do our update if we're updating bank 4
    // this way we avoid 3 wasted LED updates
    if (bank == NUM_LED_BANKS-1) {
    led_data_ready();
    }
    });
}



void SPI_WriteByte(uint8_t Dev_Add,uint8_t Reg_Add,uint8_t Reg_Dat) {
    //writing an LED register

    BEGIN_ISSI_SPI_TXN

    SPDR = Dev_Add;
    WAIT_SPI_TRANSMIT();
    SPDR = Reg_Add;
    WAIT_SPI_TRANSMIT();
    SPDR = Reg_Dat;
    WAIT_SPI_TRANSMIT();


    END_ISSI_SPI_TXN
}




/* Update the transmit buffer with LED_BUFSZ bytes of new data
 *
 * TODO: This MAY run afoul of Arduino's data size limit for an i2c transfer
 *
 * */

void led_update_all(uint8_t *buf) {
    PROTECT_LED_WRITES({
        memcpy((uint8_t *)led_buffer.whole, buf, LED_BUFSZ);
    led_data_ready();
    });
}


void led_set_one_to(uint8_t led, uint8_t *buf) {
    PROTECT_LED_WRITES({
        memcpy((uint8_t *)led_buffer.each[led], buf, LED_DATA_SIZE);
    led_data_ready();
    });

}

void led_set_global_brightness(uint8_t brightness) {
    // Legal brightness inputs are 0 to 0xff

    global_brightness = brightness;
    SPI_WriteByte(Addr_Write_Page2,0x01,brightness);//GCC
}


void led_set_all_pwm_to(uint8_t brightness) {
    BEGIN_ISSI_SPI_TXN

    SPDR = Addr_Write_Page0;
    WAIT_SPI_TRANSMIT();
    SPDR = 1;
    WAIT_SPI_TRANSMIT();
    for(int i=0; i<LED_DATA_BYTES; i++) {
        SPDR=brightness;
        WAIT_SPI_TRANSMIT();
    }

    END_ISSI_SPI_TXN

}

void led_set_all_to( uint8_t *buf) {
    PROTECT_LED_WRITES({
        for(int8_t led=31; led>=0; led--) {
            memcpy((uint8_t *)led_buffer.each[led], buf, LED_DATA_SIZE);
        }
    led_data_ready();
    });

}

uint8_t led_get_spi_frequency() {
    return led_spi_frequency;
}

void led_set_spi_frequency(uint8_t frequency) {
    led_spi_frequency = frequency;


    /* Enable SPI master, MSB first
     * fOSC/16 speed (512KHz), the default
      Measured at about 300 Hz of LED updates */


    // This is the default SPI "on" incant
    // But without the interrupt (SPIE) (enabled later with ENABLE_LED_WRITES)

    SPCR = _BV(SPE) | _BV(MSTR);

    // Which speeds are "double speed"
    switch(frequency) {
    //
    case LED_SPI_FREQUENCY_4MHZ:
    case LED_SPI_FREQUENCY_1MHZ:
    case LED_SPI_FREQUENCY_256KHZ:
    case LED_SPI_FREQUENCY_128KHZ:
        SPSR |= _BV(SPI2X);
        break;
    case LED_SPI_FREQUENCY_2MHZ:
    case LED_SPI_FREQUENCY_512KHZ:
    case LED_SPI_FREQUENCY_64KHZ:
        SPSR ^= _BV(SPI2X);
        break;
    }

    // Slightly less code to get us the same values for SPI speed
    switch(frequency) {
    case LED_SPI_OFF:
        SPCR = 0x00;
        break;
    // These values want SPR0 but not SPR1
    case LED_SPI_FREQUENCY_1MHZ:
    case LED_SPI_FREQUENCY_512KHZ:
        SPCR |= _BV(SPR0);
        break;

    // These values want SPR0 AND SPR1, so
    // no break at the end of this case. let it cascade through
    case LED_SPI_FREQUENCY_128KHZ:
    case LED_SPI_FREQUENCY_64KHZ:
        SPCR |= _BV(SPR0);
    // fall through

    // This value wants ONLY SPR1 set, not SPR0
    case LED_SPI_FREQUENCY_256KHZ:
        SPCR |= _BV(SPR1);

    }
}

/* Sets all leds off, without using the interrupt */
static void led_turn_all_off_synchronous() {
    /* make sure we disabled the interrupt ! */
    DISABLE_LED_WRITES;
    BEGIN_ISSI_SPI_TXN
    SPDR = Addr_Write_Page1;
    WAIT_SPI_TRANSMIT();
    SPDR = 1; // Start at register 1;
    WAIT_SPI_TRANSMIT();
    for (uint8_t i = 0; i < LED_DATA_BYTES ; i++) {
        SPDR = 0;
        WAIT_SPI_TRANSMIT();
    }
    END_ISSI_SPI_TXN
}







void led_init() {

    if (AD01()) {
        led_map = led_map_right;
    } else {
        led_map = led_map_left;
    }

    /* Set MOSI, SCK, SS all to outputs */
    DDRB = _BV(5)|_BV(3)|_BV(2)|_BV(7);
    PORTB &= ~(_BV(5)|_BV(3)|_BV(2));

    HIGH(PORTB,7);

    led_set_spi_frequency(LED_SPI_FREQUENCY_FOR_INIT);
    led_turn_all_off_synchronous();

    led_set_global_brightness(0xff);
    led_set_all_pwm_to(0xff);




    // reg 0x00 0b0101xxxx = sw1-sw6 active
    // 	      0bxxxxx11x = open detection disable? maybe?
    // 	      0bxxxxxxx1 = software shutdown off
    SPI_WriteByte(Addr_Write_Page2,0x00,0b01011111);

    // Reg 0x01 - Global Current control (global brightness scaling. 00 to 0xff)
    SPI_WriteByte(Addr_Write_Page2,0x01,0xFF);//GCC

    // Reg 0x02 - Pull Down/Up Resistor Selection
    // PHC 	0b0xxxxxxx - No phase delay
    // 		0b1xxxxxxx - 180 degree phase delay
    // SWx Pull down Resistor Selection Bit
    // SWPDR
    //		0bx000xxxx No pull down resistor
    //		0bx001xxxx 0.5kΩ only in SWx off time
    //		0bx010xxxx 1.0kΩ only in SWx off time
    //		0bx011xxxx 2.0kΩ only in SWx off time
    //		0bx100xxxx 1.0kΩ all the time
    //		0bx101xxxx 2.0kΩ all the time
    //		0bx110xxxx 4.0kΩ all the time
    //		0bx111xxxx 8.0kΩ all the time
    // D3 - Always 0
    // 		0bxxxx0xxx
    // CSPUR
    // CSy Pull up Resistor Selection Bit
    //		0bxxxxx000 No pull up resistor
    //		0bxxxxx001 0.5kΩ only in CSx off time
    //		0bxxxxx010 1.0kΩ only in CSx off time
    //		0bxxxxx011 2.0kΩ only in CSx off time
    //		0bxxxxx100 1.0kΩ all the time
    //		0bxxxxx101 2.0kΩ all the time
    //		0bxxxxx110 4.0kΩ all the time
    //		0bxxxxx111 8.0kΩ all the time
    // SPI_WriteByte(Addr_Write_Page2,0x02,0b11110111);
    SPI_WriteByte(Addr_Write_Page2,0x02,0b00000000);




    led_set_spi_frequency(LED_SPI_FREQUENCY_DEFAULT);
    ENABLE_LED_WRITES;
}


enum phases {
    ADDR,
    INDEX,
    VALUE
};

enum pixels {
    GREEN,
    BLUE,
    RED
};

static uint8_t pixel = GREEN;
static uint8_t index = 1; /* next led data register to to transmit */
static uint8_t phase = ADDR;
/* Each time a byte finishes transmitting, queue the next one */


void issi31fl3743b_send_byte() {
    switch(phase) {
    case ADDR:
        // If we're just starting up the interrupt handler cycle for an update
        // to the LEDs
        leds_dirty=0;
        // Toggle the pin that tells the LED controller when we've ended a command
        BEGIN_ISSI_SPI_TXN
        SPDR = Addr_Write_Page1;
        phase = INDEX;
        break;
    case INDEX:
        SPDR= 1;
        phase = VALUE;
        break;
    case VALUE:

        // Where there's a "NO_KEY" gap in the matrix, we paste in the value
        // of LED 0. This is a tiny bit wasteful of power in -theory-, but unless
        // we see ghosting, it -should- be fine. It's more efficient than adding more conditionals in the code
        SPDR =  led_buffer.each[led_map[index]][GREEN];
	WAIT_SPI_TRANSMIT();	
        SPDR =  led_buffer.each[led_map[index]][BLUE];
	WAIT_SPI_TRANSMIT();	
        SPDR =  led_buffer.each[led_map[index]][RED];
        break;
    }

    // We're all done.
    if (phase == VALUE) {
            if (index > LED_DRIVER_PIXEL_REGISTER_COUNT) {
                phase = ADDR;
                index = 1;
            } else {
                index++;
            }
        
    }
    if (phase == ADDR && index == 1 && leds_dirty ==0) {
        // and we get told that there's no new data for the LEDs
        // end the transation and turn off the interrupt handler
        // There should be no `leds_dirty` race condition here because
        // we are not multi-threaded: `led_data_ready` should never be
        // able to run here, ISR() (not naked) disables the global
        // interrupt flag for the time of the call, and we are using
        // PROTECT_LED_WRITES and not DISABLE_INTERRUPTS.
        WAIT_SPI_TRANSMIT(); // We need to wait for the last byte to get transmitted or the last LED will look flaky
        DISABLE_LED_WRITES
        END_ISSI_SPI_TXN
    }
}


ISR(SPI_STC_vect) {
	issi31fl3743b_send_byte();
}
#endif
