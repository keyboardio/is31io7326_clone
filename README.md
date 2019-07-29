# Keyboardio Keyscanner firmware

Firmware for an ATTiny88 to act as a keymatrix scanner and LED driver.

Key reporting and rollover detection are tested and working, all options in the configuration register are saved but do not yet influence behaviour.

## Getting started:

1. Install an avr toolchain (if you're on OS X, try [CrossPack for AVR®](https://www.obdev.at/products/crosspack/))
2. `cd firmware`
3. `make`
4. `make flash` and `make fuse` as necessary to program your MCU

### Common issues:

---

`avrdude: no programmer has been specified on the command line or the config file`

`avrdude` needs to know what kind of programmer you're using. You can uncomment/edit one of the `PROGRAMMER` lines in `firmware/Makefile`, or add a `default_programmer` directive to your `~/.avrduderc`.

For more information, check out [`firmware/Makefile`](firmware/Makefile).

---

`avrdude: AVR Part "attiny48" not found.`

`avrdude` can't find a part config for the attiny48. You can resolve this by appending [`doc/attiny48.avrduderc`](doc/attiny48.avrduderc) to `~/.avrduderc`:

```
cat doc/attiny48.avrduderc >> ~/.avrduderc
```

---

## Porting to different hardware

It's possible to configure this project for different AVRs. Check out [`config/attiny48.h`](firmware/config/attiny48.h) to get familiar with the required definitions and application considerations.
