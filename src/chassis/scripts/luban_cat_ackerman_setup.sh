#!/bin/bash

echo 0 > /sys/class/pwm/pwmchip1/export
echo 0 > /sys/class/pwm/pwmchip3/export
echo 0 > /sys/class/pwm/pwmchip4/export

chmod 666 /sys/class/pwm/pwmchip*/pwm*/period
chmod 666 /sys/class/pwm/pwmchip*/pwm*/duty_cycle
chmod 666 /sys/class/pwm/pwmchip*/pwm*/polarity
chmod 666 /sys/class/pwm/pwmchip*/pwm*/enable
