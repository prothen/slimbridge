#! /bin/sh
esptool.py --baud 921600 --port /dev/ttyUSB0 write_flash 0x00000 ./resources/binaries/firmware-1.2.2.bin

