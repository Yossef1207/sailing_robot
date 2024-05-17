#!/bin/sh
/usr/bin/avrdude -c wiring -p atmega2560 -P /dev/ttyACM0 -b115200  -U flash:w:/home/ubuntu/APMboat/APMHexFile/APMboat.hex:i

