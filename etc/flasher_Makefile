UNAME_S := $(shell uname -s)
ifeq ($(UNAME_S),Linux)
  DEVICE_PORT=/dev/ttyACM1
  ARDUINO_PATH=/usr/local/arduino
  ARDUINO_LOCAL_LIB_PATH=${HOME}/Arduino
  STTY_F_OPT=-F
endif
ifeq ($(UNAME_S),Darwin)
  DEVICE_PORT=$(shell ls /dev/tty.usbmodem* | head -n1)
  ARDUINO_PATH=/Applications/Arduino.app/Contents/Java
  ARDUINO_LOCAL_LIB_PATH=${HOME}/Documents/Arduino
  STTY_F_OPT=-f
endif

ARDUINO_TOOLS_PATH=${ARDUINO_PATH}/hardware/tools
ARDUINO_BUILDER=${ARDUINO_PATH}/arduino-builder
ARDUINO_IDE_VERSION=100607
BOARD_HARDWARE_PATH=${ARDUINO_LOCAL_LIB_PATH}/hardware
BOARD=model01
MCU=atmega32u4
FQBN=keyboardio:avr:${BOARD}
BUILD_PATH := $(abspath ./build)

SKETCH=attiny_flasher.ino

build:
	mkdir ${BUILD_PATH}
	${ARDUINO_BUILDER} -compile \
          -verbose \
          -hardware "${ARDUINO_PATH}/hardware" \
          -hardware "${BOARD_HARDWARE_PATH}" \
          -tools "${ARDUINO_PATH}/tools-builder" \
          -tools "${ARDUINO_PATH}/hardware/tools" \
          -fqbn "${FQBN}" \
          -libraries "${BOARD_HARDWARE_PATH}/.." \
          -libraries "." \
          -build-path "${BUILD_PATH}" \
          -ide-version "${ARDUINO_IDE_VERSION}" \
          -warnings all \
          -prefs "compiler.cpp.extra_flags=-std=c++11 -Woverloaded-virtual -Wno-unused-parameter -Wno-unused-variable -Wno-ignored-qualifiers" \
          ${SKETCH}
	cp ${BUILD_PATH}/${SKETCH}.hex .


flash: build
	stty ${STTY_F_OPT} ${DEVICE_PORT} 1200 hupcl
	sleep 2
	avrdude -p${MCU} -cavr109 -D -P ${DEVICE_PORT} -b57600 "-Uflash:w:${SKETCH}.hex:i"
