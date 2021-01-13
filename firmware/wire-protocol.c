#include "wire-protocol.h"
#include <string.h>
#include "main.h"
#include "ringbuf.h"
#include "twi-slave.h"
#include "keyscanner.h"
#include "led-api.h"



void twi_init(void) {

    // Assert comm_en so we can use the interhand transcievers
    // (Until comm_en on the i2c transcievers is pulled high,
    //  they're disabled)
    HIGH(PORTC,7);
    SET_OUTPUT(DDRC,7);

    TWI_Rx_Data_Callback = twi_data_received;
    TWI_Tx_Data_Callback = twi_data_requested;

    // TODO: set TWI_Tx_Data_Callback and TWI_Rx_Data_Callback
    TWI_Slave_Initialise(TWI_BASE_ADDRESS | AD01());
    sei();
}

static uint8_t twi_command = TWI_CMD_NONE;

void twi_data_received(uint8_t *buf, uint8_t bufsiz) {
    // if the upper four bits of the byte say this is an LED cmd
    // this is the most common case. It's also the only case where
    // we can't just compare buf[0] to a static value
    if (__builtin_expect( ((buf[0] & 0xf0) == TWI_CMD_LED_BASE),EXPECT_TRUE))  {
        led_update_bank(&buf[1], buf[0] & 0x0f); // the lowest four bits are the bank #
        return;
    }

    twi_command = buf[0];
    if (bufsiz > 1)
        twi_command = TWI_CMD_NONE;

    switch (buf[0]) {
    case TWI_CMD_LED_UPDATE_ALL:
        led_update_all(&buf[1]);
        break;

    case TWI_CMD_KEYSCAN_INTERVAL:
        if (bufsiz == 2 ) // SET the delay
            keyscanner_set_interval(buf[1]);
        break;

    case TWI_CMD_LED_SPI_FREQUENCY:
        if (bufsiz == 2 )
            led_set_spi_frequency(buf[1]);
        break;


    case TWI_CMD_LED_SET_ALL_TO:
        if (bufsiz == 4 )
            led_set_all_to(&buf[1]);

        break;

    case TWI_CMD_LED_SET_ONE_TO:
        if (bufsiz == 5 )
            led_set_one_to(buf[1],&buf[2]);
        break;

    case TWI_CMD_LED_GLOBAL_BRIGHTNESS:
        led_set_global_brightness(buf[1]);
        break;

    case TWI_CMD_VERSION:
    case TWI_CMD_KEYDATA_SIZE:
        break;

    }
}

uint8_t key_substate;

void twi_data_requested(uint8_t *buf, uint8_t *bufsiz) {
    if (__builtin_expect(*bufsiz != 0, EXPECT_TRUE)) {

        // Almost every command has a bufsiz of 1.
        // populate the default and only set it when we want to change it.
        *bufsiz=1;
        switch (twi_command) {
        case TWI_CMD_NONE:
            // Keyscanner Status Register
            if (ringbuf_empty()) {
                // Nothing in the ring buffer is the same thing as all keys released
                // Really, we _should_ be able to return a single byte here, but
                // Jesse is too clueless to figure out how to get I2C to signal
                // a 'short' response
                buf[0]=TWI_REPLY_NONE;
            } else {
                buf[0]=TWI_REPLY_KEYDATA;
                for(int i = 1; i<= KEY_REPORT_SIZE_BYTES; i++) {
                    ringbuf_pop_to(buf+i);
                }
                *bufsiz=(KEY_REPORT_SIZE_BYTES+1);
            }
            break;
        case TWI_CMD_VERSION:
            buf[0] = DEVICE_VERSION;
            break;
        case TWI_CMD_KEYDATA_SIZE:
            buf[0] = KEY_REPORT_SIZE_BYTES;
            break;
        case TWI_CMD_KEYSCAN_INTERVAL:
            buf[0] = keyscanner_get_interval();
            break;
        case TWI_CMD_LED_SPI_FREQUENCY:
            buf[0] = led_get_spi_frequency();
            break;
        default:
            buf[0] = 0x01;
            break;
        }
    }
}
