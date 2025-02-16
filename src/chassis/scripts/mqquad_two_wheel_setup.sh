#!/bin/bash

echo 1 > /sys/class/pwm/pwmchip0/export
echo 3 > /sys/class/pwm/pwmchip0/export
echo 4 > /sys/class/pwm/pwmchip0/export

chmod 666 /sys/class/pwm/pwmchip*/pwm*/period
chmod 666 /sys/class/pwm/pwmchip*/pwm*/duty_cycle
chmod 666 /sys/class/pwm/pwmchip*/pwm*/polarity
chmod 666 /sys/class/pwm/pwmchip*/pwm*/enable
