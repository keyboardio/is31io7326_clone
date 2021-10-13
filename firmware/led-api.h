/* APA102 LED driver, receives data as a precanned buffer then outputs it via SPI interrupts */
#pragma once


/* Call to begin transmitting LED data */
void led_init(void);

/* Call to change the speed at which we update the LEDs */
void led_set_spi_frequency(uint8_t frequency);

uint8_t led_get_spi_frequency(void);

/* Call this when you have new preformatted data for one bank of the LEDs */
void led_update_bank(uint8_t *buf, const uint8_t bank);

/* Call this when you have new preformatted data for all the LEDs */
void led_update_all(uint8_t *buf);


/* Call this when you want to set only a single LED */
void led_set_one_to(uint8_t led, uint8_t * led_data);

/* Call this when you want to set every LED to the same value */
void led_set_all_to(uint8_t * led_data);

/* Call this when you want to set every LED to off */
void led_set_all_off(void);


/* Call this with a value between 0 and 31 to set the LED's global brightness */
void led_set_global_brightness(uint8_t global_brightness);

void led_data_ready(void);
