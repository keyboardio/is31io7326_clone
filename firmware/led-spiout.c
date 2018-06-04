#include <stdint.h>
#include <avr/io.h>

#include <string.h>
#include <util/delay.h>
#include "main.h"
#include "led-spiout.h"
#include "wire-protocol.h"


/* SPI LED driver to send data to APA102 LEDs
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

#define BRIGHTNESS_MASK 0b11100000

#define ENABLE_LED_WRITES SPCR |= _BV(SPIE);
#define DISABLE_LED_WRITES SPCR &= ~_BV(SPIE);

static uint8_t led_spi_frequency = LED_SPI_FREQUENCY_DEFAULT;

typedef union {
    uint8_t each[NUM_LEDS][LED_DATA_SIZE];
    uint8_t whole[LED_BUFSZ];
    uint8_t bank[NUM_LED_BANKS][LED_BANK_SIZE];
} led_buffer_t;

static volatile led_buffer_t led_buffer;

static volatile enum {
    START_FRAME,
    DATA,
    END_FRAME
} led_phase;

static volatile uint8_t global_brightness = BRIGHTNESS_MASK | 31; /* max is 31 */

static volatile uint8_t index; /* next byte to transmit */
static volatile uint8_t subpixel = 0;
static volatile uint8_t leds_dirty = 1;


void led_data_ready() {
    if(leds_dirty <2) {
        leds_dirty++;
    }
    ENABLE_LED_WRITES;
}

/* this function updates our led data state. We use this to
 * make sure that we update the LEDs  if there's any data that hasn't yet been
 * sent. Because there's a potential race condition in the LED update state
 * machine that could result in a skipped LED update if the LED buffer was
 * updated during the 'start' frame, we let the dirty state variable
 * go to 2.
 *
 * This means that there's a slight chance we'll update all the LEDs twice
 * if we receive an update during the start frame, but that's better than
 * missing a frame.
 */


/* Update the transmit buffer with LED_BUFSZ bytes of new data */
void led_update_bank(uint8_t *buf, const uint8_t bank) {
    /* Double-buffering here is wasteful, but there isn't enough RAM on
       ATTiny48 to single buffer 32 LEDs and have everything else work
       unmodified. However there's enough RAM on ATTiny88 to double
       buffer 32 LEDs! And double buffering is simpler, less likely to
       flicker. */

    DISABLE_INTERRUPTS({
        memcpy((uint8_t *)led_buffer.bank[bank], buf, LED_BANK_SIZE);
    });
    // Only do our update if we're updating bank 4
    // this way we avoid 3 wasted LED updates
    //if (bank == NUM_LED_BANKS-1) {
        led_data_ready();
    // }
}

/* Update the transmit buffer with LED_BUFSZ bytes of new data
 *
 * TODO: This MAY run afoul of Arduino's data size limit for an i2c transfer
 *
 * */

void led_update_all(uint8_t *buf) {
    DISABLE_INTERRUPTS({
        memcpy((uint8_t *)led_buffer.whole, buf, LED_BUFSZ);
    });
    led_data_ready();
}


void led_set_one_to(uint8_t led, uint8_t *buf) {
    DISABLE_INTERRUPTS({
        memcpy((uint8_t *)led_buffer.each[led], buf, LED_DATA_SIZE);
    });
    led_data_ready();

}

void led_set_global_brightness(uint8_t brightness) {
    // Legal brightness inputs are 0 to 31.
    // But the output we want has the 3 high bytes set anyway
    // So ORing our input with the brightness mask limits
    // the input to 31.

    global_brightness = BRIGHTNESS_MASK | brightness;
    led_data_ready();
}

void led_set_all_to( uint8_t *buf) {
    DISABLE_INTERRUPTS({
        for(int8_t led=31; led>=0; led--) {
            memcpy((uint8_t *)led_buffer.each[led], buf, LED_DATA_SIZE);
        }
    });
    led_data_ready();

}

void led_set_all_off (void) {
    memset((uint8_t*)led_buffer.whole, 0, sizeof(led_buffer.whole));
    led_data_ready();
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

    SPCR = _BV(SPE) | _BV(MSTR) | _BV(SPIE);

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


void led_init() {

    // Make sure all our LEDs start off dark
    led_set_all_off();

    /* Set MOSI, SCK, SS all to outputs */
    DDRB = _BV(5)|_BV(3)|_BV(2);
    PORTB &= ~(_BV(5)|_BV(3)|_BV(2));

    led_set_spi_frequency(led_spi_frequency);

    /* Start transmitting the first byte of the start frame */
    led_phase = START_FRAME;
    leds_dirty = 1;
    SPDR = 0x0;
    index = 1;
    subpixel = 0;
}

/* Each time a byte finishes transmitting, queue the next one */
ISR(SPI_STC_vect) {

    switch(led_phase) {
    case START_FRAME:
        SPDR = 0;
        if(++index == 8) {
            led_phase = DATA;
            index = 0;
            if (leds_dirty > 0) {
                leds_dirty--;
            }
        }
        break;
    case DATA:
        if (++subpixel == 1) {
            SPDR = global_brightness;
        } else {
            SPDR = led_buffer.whole[index++];
            subpixel %= 4; // reset the subpixel once it goes past brightness,r,g,b
        }

        if (index == LED_BUFSZ) {
            led_phase = END_FRAME;
            index = 0;
            subpixel = 0;
        }
        break;

    case END_FRAME:
        // The pwm reset frame needs to be 32 bits of 0 for SK9822 based LEDS
        // After that, we need at num_leds/2 more bits of 0
        // For up to 64 LEDs, that means 64 bits of 0
        SPDR = 0x00;
        if(++index == 8) { /* NB: increase this number if ever >64 LEDs */
            led_phase = START_FRAME;
            index = 0;
            if (leds_dirty ==0) {
                DISABLE_LED_WRITES;
            }
        }
        break;
    }
}
