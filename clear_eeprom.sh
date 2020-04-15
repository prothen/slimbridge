#! /bin/sh
esptool.py --port /dev/ttyUSB0 -b 115200 write_flash 0x000000 ./resources/binaries/blank_1MB.bin
