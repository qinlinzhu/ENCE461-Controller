PERIPHERALS = pwm 
DRIVERS = pacer usb_serial nrf24 panic adxl345 piezo ledtape

SRC = hat.c
OPT = -O0

include ../../boards/board.mk
