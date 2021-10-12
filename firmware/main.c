#include "wire-protocol.h"
#include "keyscanner.h"
#include "led-api.h"
#include <util/delay.h>

static inline void setup(void) {
	_delay_ms(100);
    led_init();
    keyscanner_init();
    twi_init();
}

int main(void) {
    setup();
    while(1) {
        keyscanner_main();
    }
    __builtin_unreachable();
}
