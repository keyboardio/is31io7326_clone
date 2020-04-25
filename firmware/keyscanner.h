#pragma once


//Signal port (rows)
#define PORT_OUTPUT PORT_ROWS
#define DDR_OUTPUT DDR_ROWS
#define PIN_OUTPUT PIN_ROWS
#define MASK_OUTPUT MASK_ROWS
#define COUNT_OUTPUT COUNT_ROWS
#define PIN_ORDER_OUTPUT PIN_ORDER_ROWS


//Scanning port (cols)
#define PORT_INPUT PORT_COLS
#define DDR_INPUT DDR_COLS
#define PIN_INPUT PIN_COLS
#define MASK_INPUT MASK_COLS
#define COUNT_INPUT COUNT_COLS



// Set data direction as output on the output pins
// Default to all output pins high
#define CONFIGURE_OUTPUT_PINS \
    PINS_HIGH(DDR_OUTPUT, MASK_OUTPUT); \
    PINS_HIGH(PORT_OUTPUT, MASK_OUTPUT);

// Set the data direction for our inputs to be "input"
// Turn on the pullups on the inputs
#define CONFIGURE_INPUT_PINS \
    PINS_LOW(DDR_INPUT, MASK_INPUT); \
    PINS_HIGH(PORT_INPUT, MASK_INPUT);


// When a key is pressed the input pin will read LOW
// Active pins on are low. So the debouncer inverts them before working with them
#define KEYSCANNER_CANONICALIZE_PINS(pins) ~pins


// AD01: lower two bits of device address
#define AD01() ((PINB & _BV(0)) |( PINB & _BV(1)))

void keyscanner_init(void);
void keyscanner_main(void);
void keyscanner_record_state(void);
void keyscanner_ringbuf_update(uint8_t row1, uint8_t row2, uint8_t row3, uint8_t row4);
void keyscanner_timer1_init(void);

void keyscanner_set_interval(uint8_t interval);
uint8_t keyscanner_get_interval();


