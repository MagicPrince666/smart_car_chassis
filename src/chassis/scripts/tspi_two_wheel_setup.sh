#!/bin/bash

echo 0 > /sys/class/pwm/pwmchip8/export
echo 0 > /sys/class/pwm/pwmchip14/export
echo 0 > /sys/class/pwm/pwmchip15/export

chmod 666 /sys/class/pwm/pwmchip*/pwm*/period
chmod 666 /sys/class/pwm/pwmchip*/pwm*/duty_cycle
chmod 666 /sys/class/pwm/pwmchip*/pwm*/polarity
chmod 666 /sys/class/pwm/pwmchip*/pwm*/enable
