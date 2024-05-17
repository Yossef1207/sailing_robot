#!/bin/sh
if [ $# -ne 1 ]; then
    echo "Usage: ./flashArduino.sh <device>"
    echo "Example: ./flashArduino.sh USB0"
    exit 1
fi

arduino-cli compile --fqbn arduino:avr:nano:cpu=atmega328old nano_pwm_converter
arduino-cli upload -p /dev/tty$1 --fqbn arduino:avr:nano:cpu=atmega328old nano_pwm_converter
