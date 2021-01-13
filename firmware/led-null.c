#ifdef LED_DRIVER_NULL

#include <stdint.h>


// Using DISABLE_INTERRUPTS could cause interrupt recursion because led_update
// functions are called from TWI callbacks which run within TWI interrupt. So
// disable only LED SPI transfer interrupt.
//
// And to avoid redundancy, leave re-ENABLE_LED_WRITES to led_data_ready().

void led_data_ready() { }

/* Update the transmit buffer with LED_BUFSZ bytes of new data */
void led_update_bank(__attribute__((unused)) uint8_t *buf, const __attribute__((unused)) uint8_t bank) { }

void led_update_all(__attribute__((unused)) uint8_t *buf) {}
void led_set_one_to(__attribute__((unused)) uint8_t led, __attribute__((unused)) uint8_t *buf) {}
void led_set_global_brightness(__attribute__((unused)) uint8_t brightness) {}
void led_set_all_to( __attribute__((unused)) uint8_t *buf) {}
int8_t led_get_spi_frequency() {return 0;}
void led_set_spi_frequency(__attribute__((unused)) uint8_t frequency) {}
void led_init() {}
#endif
