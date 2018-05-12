/* APA102 LED driver, receives data as a precanned buffer then outputs it via SPI interrupts */
#pragma once

/* Number of LEDs in the chain.
   Max is ~8 (maybe less) on ATTiny48, 32 on ATTiny88. */
#define NUM_LEDS 32
#define NUM_LEDS_PER_BANK 8
#define NUM_LED_BANKS NUM_LEDS/NUM_LEDS_PER_BANK
#define LED_DATA_SIZE 3
#define LED_BUFSZ (LED_DATA_SIZE *NUM_LEDS)
#define LED_BANK_SIZE (LED_DATA_SIZE*NUM_LEDS_PER_BANK)




/* Call to begin transmitting LED data */
void led_init(void);

/* Call to change the speed at which we update the LEDs */
void led_set_spi_frequency(uint8_t frequency);

uint8_t  led_get_spi_frequency(void);

/* Call this when you have new preformatted data for one bank of the LEDs */
void led_update_bank(uint8_t *buf, const uint8_t bank);

/* Call this when you have new preformatted data for all the LEDs */
void led_update_all(uint8_t *buf);


/* Call this when you want to set only a single LED */
void led_set_one_to(uint8_t led, uint8_t * led_data);

/* For debugging only, call this when you want to get only a single LED */
uint8_t *led_get_one_addr_unsafe(uint8_t led);

/* Call this when you want to set every LED to the same value */
void led_set_all_to(uint8_t * led_data);

/* Call this when you want to set every LED to off */
void led_set_all_off(void);


/* Call this with a value between 0 and 31 to set the LED's global brightness */
void led_set_global_brightness(uint8_t global_brightness);
